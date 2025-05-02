import os, json, threading, time, subprocess
import speech_recognition as sr
import openai
from google.cloud import texttospeech
from core.actions import move_right, move_left, move_forward, move_backward, stop as stop_movement, dance, come

# === Global State ===
context = ""
personality = ""
conversation_msgs = []

_convo_started = threading.Event()
_running = threading.Event()
_abort = threading.Event()
_loop_thread = None

# === TTS Client ===
tts_client = texttospeech.TextToSpeechClient()
VOICE_PARAMS = texttospeech.VoiceSelectionParams(language_code="en-US", name="en-US-Wavenet-I")
AUDIO_CONFIG = texttospeech.AudioConfig(audio_encoding=texttospeech.AudioEncoding.LINEAR16)

# === UAH Tour Guide Context ===
UAH_TOUR_GUIDE_CONTEXT = """
You are a friendly, knowledgeable tour guide for The University of Alabama in Huntsville. Your job is to provide engaging facts, building functions, and campus highlights to visitors. Speak in a warm, upbeat tone. Mention campus landmarks and directions based on the official map. Below is your guide:

Academic and Administrative Buildings
Charger Union (CGU) – The student hub! Includes dining areas, the bookstore, game rooms, lounges, and events
Salmon Library (LIB) – UAH’s academic library. Great for quiet study, with extensive research resources and staff.
Shelby Center (SST) – Core building for science and technology. Includes chemistry/biology labs and lecture halls.
Engineering Building (ENG) – Main location for engineering majors. Features specialized labs and classrooms.
Olin B. King Technology Hall (OKT) – Home to computer science, software, and cybersecurity programs.
Morton Hall (MOR) – Humanities, English, and social sciences. A historic and central lecture building.
Roberts Hall (ROB) – Math and education departments, located near Spragins and the library.
Business Administration Building (BAB) – Where business students attend classes and connect with mentors.
Student Services Building (SSB) – Financial aid, admissions, registrar, and student advising all in one place.
Nursing Building (NUR) – Features hands-on simulation labs and modern facilities for future nurses.
Wilson Hall (WIL) – Psychology, visual arts, and the Wilson Theatre are located here.
Von Braun Research Hall (VBH) – Research-focused building for physics and engineering students.
I²C (Invention to Innovation Center) – A startup and entrepreneurship hub. Hosts student founders and tech mentors.
SWIRLL (SWI) – Severe Weather research center used by meteorology students. Lightning and radar research happens here.
Cramer Research Hall (CRH) – A multidisciplinary research facility near SWIRLL and Engineering.
Johnson Research Center (JRC) – Another advanced research center in proximity to Cummings Research Park.
Central Receiving & Shipping (REC) – For campus logistics and deliveries.
Business Services Building (BSB) – Administrative building for UAH operations.
University Fitness Center (UFC) – Full gym with weights, cardio, indoor pool, and group classes.
Residence Halls (CGV, NCH, CCH, FFH, BEV) – On-campus student housing options, all walkable to class and the Charger Union.
Intermodal Parking Facility (IMF) – A large parking deck near the heart of campus.

Places to Eat on Campus
Located mostly in Charger Union(CU) or Charger Village (CV), these are your go-to spots:
Charger Café – The main dining hall. Hot meals, salad bar, breakfast, and daily chef’s specials. in north of campus, not in CU or CV
Dunkin' Donuts – Coffee, donuts, bagels, and breakfast items. A student favorite for morning fuel. in CU
Chick-fil-A – Classic chicken sandwiches, nuggets, fries, and shakes. in CV
Mein Bowl – Asian cuisine, including rice bowls, stir fry, and egg rolls. in CU
Burrito Bowl – Tex-Mex bowls and wraps made to order. in CV
C-Store – A mini convenience store for snacks, drinks, and grab-and-go items in CV.
Charger Brew – A cozy café (currently with limited hours) serving coffee and espresso drinks. right next to charger cafe
The Den – (May have limited hours) a casual late-night spot serving burgers and more. in CU
Most dining locations accept ChargerBucks, meal swipes, or Grubhub orders. Seating is available inside Charger Union and outside near the lawn for nice weather.

Campus Life & Industry Connections
Redstone Arsenal – A top destination for UAH interns. Home to NASA Marshall Space Flight Center and Army R&D programs.
Cummings Research Park – The second-largest research park in the U.S. Partners with UAH for co-ops and employment.
""".strip()

# === Helpers ===
def _speak(text):
    if _abort.is_set():
        return
    path = "/tmp/robodog_tts.wav"
    try:
        synthesis_input = texttospeech.SynthesisInput(text=text)
        response = tts_client.synthesize_speech(input=synthesis_input, voice=VOICE_PARAMS, audio_config=AUDIO_CONFIG)
        with open(path, "wb") as out:
            out.write(response.audio_content)
        if not _abort.is_set():
            subprocess.run(["aplay", "-D", "plughw:3,0", path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as e:
        print(f"[TTS] Error: {e}")

def _recognize(prompt="Listening…", timeout=12):
    result = [None]
    def worker():
        if _abort.is_set() or not _convo_started.is_set():
            return
        r = sr.Recognizer()
        try:
            with sr.Microphone(device_index=2) as source:
                print(prompt, flush=True)
                r.adjust_for_ambient_noise(source, duration=1)
                audio = r.listen(source, phrase_time_limit=10, timeout=5)
                if not _abort.is_set() and _convo_started.is_set():
                    result[0] = r.recognize_google(audio)
        except Exception as e:
            print(f"[Speech] {e}")
    t = threading.Thread(target=worker, daemon=True)
    t.start()
    t.join(timeout)
    return result[0]

def _llm():
    try:
        response = openai.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=conversation_msgs,
            max_tokens=500,
            temperature=0.7
        )
        js = json.loads(response.choices[0].message.content.strip())
        return js.get("action", "none"), js.get("response", "")
    except Exception as e:
        print(f"[LLM] {e}")
        return "none", "Sorry, something went wrong."

def _system_prompt(ctx, pers):
    return f"""
You are RoboDog, an advanced robotic dog developed at UAH. You must use the following personality and context to respond accurately and in character.

Become the personality chosen by the user — use the dialect, speech pattern, and tone of: {pers}.

The user may also give you context for the conversation. This is info the user wants you to know when talking to them. If they gave nothing, proceed as usual. Here is the context: {ctx}

Your job is to conversate with the user in the chosen personality using the context as background knowledge. If the user gives a command or you infer one, you must decide on one of these exact actions:
"right", "left", "forward", "backward", "stop", "dance", "come", or "none".

Respond ONLY with this JSON format:
{{
  "action": "action",
  "response": "your reply"
}}

DO NOT use any emojis in your response.
""".strip()

# === Public API ===
def set_personality(p):
    global personality, context
    personality = p.strip()
    print(f"[VoiceControl] Personality = '{personality}'", flush=True)
    if "uah" in personality.lower():
        context = UAH_TOUR_GUIDE_CONTEXT
        print("[VoiceControl] UAH tour guide context set.", flush=True)

def set_context(c):
    global context
    context = c.strip()
    print("[VoiceControl] Context set.", flush=True)

def begin_convo():
    _convo_started.set()

def end_convo():
    _convo_started.clear()
    _reset_state()
    print("[VoiceControl] Conversation ended – idle.", flush=True)

def is_running():
    return _running.is_set()

def start():
    global _loop_thread
    if _running.is_set():
        return
    _running.set()
    _abort.clear()
    _loop_thread = threading.Thread(target=_loop, daemon=True)
    _loop_thread.start()

def stop():
    global _loop_thread
    if not _running.is_set():
        return
    _running.clear()
    _abort.set()
    _convo_started.clear()
    if _loop_thread and _loop_thread.is_alive():
        _loop_thread.join()
    _loop_thread = None
    _reset_state()

def reset_after_convo():
    _reset_state()

def _reset_state():
    global context, personality, conversation_msgs
    context = ""
    personality = ""
    conversation_msgs = []

# === Main Conversation Loop ===
def _loop():
    if not openai.api_key:
        openai.api_key = os.getenv("OPENAI_API_KEY")
    if not openai.api_key:
        print("[VoiceControl] OPENAI_API_KEY not set")
        return

    while _running.is_set() and not _abort.is_set():
        while (_running.is_set() and not _abort.is_set()
               and (not _convo_started.is_set() or not personality)):
            time.sleep(0.1)

        if not _running.is_set() or _abort.is_set():
            break

        print(f"[VoiceControl] Context: {context}")
        print(f"[VoiceControl] Personality: {personality}")
        conversation_msgs[:] = [{"role": "system", "content": _system_prompt(context, personality)}]

        _, intro = _llm()
        _speak(intro)
        conversation_msgs.append({
            "role": "assistant",
            "content": json.dumps({"action": "none", "response": intro})
        })

        while _running.is_set() and not _abort.is_set() and _convo_started.is_set():
            print("===============================")
            user_text = _recognize("Speech input:")
            if not _running.is_set() or _abort.is_set() or not _convo_started.is_set():
                break
            if not user_text:
                continue
            print(f"Speech input: {user_text}")
            if "goodbye" in user_text.lower():
                _speak("Goodbye!")
                end_convo()
                break

            conversation_msgs.append({"role": "user", "content": user_text})
            action, reply = _llm()
            print(f"Response: {reply}\nAction: {action}")
            _speak(reply)
            conversation_msgs.append({
                "role": "assistant",
                "content": json.dumps({"action": action, "response": reply})
            })

            {
                "right": move_right,
                "left": move_left,
                "forward": move_forward,
                "backward": move_backward,
                "stop": stop_movement,
                "dance": dance,
                "come": come
            }.get(action, lambda: None)()

