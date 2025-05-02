from core import gesture_recognition as gesture
from core import user_tracking       as tracking
from core import voice_control       as voice

_modes = {
    "GESTURE": gesture,
    "FOLLOW":  tracking,
    "VOICE":   voice,
}

_active          = set()
_received_ctx    = None
_received_pers   = None
_convo_started   = False

def toggle(mode: str):
    global _received_ctx, _received_pers, _convo_started
    m = _modes.get(mode.upper())
    if not m:
        print(f"[ModeMgr] Unknown mode '{mode}'", flush=True)
        return
    if mode.upper() in _active:          # turning OFF
        m.stop()
        if mode.upper() == "VOICE":
            voice.reset_after_convo()    # clear persona + ctx
            _received_ctx  = None
            _received_pers = None
            _convo_started = False
        _active.remove(mode.upper())
        print(f"[ModeMgr] {mode} mode stopped")
    else:                                # turning ON
        m.start()
        _active.add(mode.upper())
        print(f"[ModeMgr] {mode} mode started")

def handle_command(msg: str):
    global _received_ctx, _received_pers, _convo_started
    msg = msg.strip()

    if msg.startswith("MODE:"):
        mode = msg.split(":", 1)[1].strip().upper()
        if mode == "OFF":
            print("[ModeMgr] MODE:OFF — clearing all modes")
            for am in list(_active):
                toggle(am)
        else:
            toggle(mode)

    elif msg.startswith("PERS:"):
        _received_pers = msg.split(":", 1)[1].strip()
        voice.set_personality(_received_pers)

    elif msg.startswith("CTX:"):
        _received_ctx = msg.split(":", 1)[1].strip()
        voice.set_context(_received_ctx)

    elif msg == "START_CONVO":
        _convo_started = True
        voice.begin_convo()
        if not voice.is_running():
            voice.start()

    elif msg == "END_CONVO":
        if "VOICE" in _active:
            voice.end_convo()
            _convo_started = False

