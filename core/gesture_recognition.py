import threading, time, cv2, numpy as np
import mediapipe as mp
import tflite_runtime.interpreter as tflite
import subprocess
import os
from core.actions import stop as stop_movement, come, dance

_running = threading.Event()
DATA_DIR = os.path.join(os.path.dirname(__file__), "gesture_data")

def speak(_=None):
    try:
        subprocess.run(
            ["aplay", "-D", "plughw:CARD=UACDemoV10,DEV=0", os.path.join(DATA_DIR, "bark.wav")],
            check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        print("[Gesture] Played bark sound.")
    except Exception as e:
        print(f"[Gesture] Error playing bark: {e}")

def _loop():
    os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
    mp_hands = mp.solutions.hands

    interpreter = tflite.Interpreter(model_path=os.path.join(DATA_DIR, "gesture_model.tflite"))
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    input_index = input_details[0]['index']
    output_index = output_details[0]['index']
    print("[Gesture] Model loaded successfully.")

    data_mean = np.load(os.path.join(DATA_DIR, "data_mean.npy")).astype(np.float32)
    data_std = np.load(os.path.join(DATA_DIR, "data_std.npy")).astype(np.float32)
    print("[Gesture] Normalization data loaded.")

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    if not cap.isOpened():
        print("[Gesture] ERROR: Cannot open camera.")
        return

    hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1,
                           min_detection_confidence=0.7, min_tracking_confidence=0.6)

    print("[Camera] Ready. Backend:", cap.getBackendName())
    print("[Gesture] Gesture recognition started.")

    labels = {
        0: "Palm",
        1: "Thumb Up",
        2: "OK",
        3: "Fist",
        4: "Rock On"
    }

    current_gesture = None
    gesture_start_time = None
    debounce_duration = 0.25  # faster gesture response
    camera_ready = False

    while _running.is_set():
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        if not camera_ready:
            print("[Gesture] ✅ First valid frame received — camera is active.")
            camera_ready = True

        frame = cv2.flip(frame, 1)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(frame_rgb)
        detected_gesture = None

        if result.multi_hand_landmarks:
            hand_landmarks = result.multi_hand_landmarks[0]
            landmarks = [coord for lm in hand_landmarks.landmark for coord in (lm.x, lm.y, lm.z)]
            lm_np = np.array(landmarks)
            lm_np = (lm_np - data_mean) / (data_std + 1e-6)
            input_data = lm_np.reshape(1, -1).astype(np.float32)

            if input_details[0]['dtype'] == np.int8:
                scale, zp = input_details[0]['quantization']
                input_data = np.round(input_data / scale + zp).astype(np.int8)

            interpreter.set_tensor(input_index, input_data)
            interpreter.invoke()
            output = interpreter.get_tensor(output_index)[0]
            class_id = int(np.argmax(output))
            confidence = output[class_id]

            if confidence > 0.7:
                detected_gesture = labels.get(class_id)

        now = time.time()
        if detected_gesture == current_gesture:
            if gesture_start_time and now - gesture_start_time >= debounce_duration:
                print(f"[Gesture] Debounced: {current_gesture}")
                if current_gesture == "Palm":
                    stop_movement()
                elif current_gesture == "Fist":
                    come()
                elif current_gesture == "Thumb Up":
                    speak()
                elif current_gesture == "Rock On":
                    dance()
                gesture_start_time = None
        else:
            current_gesture = detected_gesture
            gesture_start_time = now if detected_gesture else None

        time.sleep(0.005)  # faster loop

    cap.release()
    print("[Gesture] Stopped.")

def start():
    if _running.is_set():
        return
    _running.set()
    threading.Thread(target=_loop, daemon=True).start()
    print("[Gesture] toggled ON")

def stop():
    _running.clear()
    print("[Gesture] toggled OFF")

