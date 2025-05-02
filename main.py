#!/usr/bin/env python3
# main.py

print("[Debug] main.py starting...")

import socket
import threading
import sys
import ctypes
import os

# ✅ Fix for sudo + Google Cloud + Bluetooth
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = "/home/ubuntu/google_credentials.json"
os.environ["DBUS_SESSION_BUS_ADDRESS"] = "unix:path=/run/user/1000/bus"
os.environ["OPENAI_API_KEY"] = "GET YOUR OWN"
# ✅ Suppress ALSA warnings
try:
    asound = ctypes.CDLL("libasound.so")
    asound.snd_lib_error_set_handler(None)
except Exception as e:
    print(f"[ALSA] Suppression failed: {e}")

# ✅ Import your mode manager
try:
    from mode_manager import handle_command
except Exception as e:
    print(f"[ImportError] Failed to import mode_manager: {e}")
    sys.exit(1)

def handle_client(client_sock, client_info):
    print(f"[Link] Client connected: {client_info}")
    try:
        while True:
            data = client_sock.recv(1024)
            if not data:
                break
            msg = data.decode('utf-8', errors='ignore').strip()
            print(f"[Link] Received: {msg}")
            handle_command(msg)
            client_sock.send(f"ACK:{msg}\n".encode('utf-8'))
    except Exception as e:
        print(f"[Link] Client error: {e}")
    finally:
        print(f"[Link] Client disconnected: {client_info}")
        client_sock.close()

def main():
    print("[Main] Starting RoboDog connection handler…")

    try:
        server_sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    except AttributeError:
        print("[ERROR] Your Python build does not support AF_BLUETOOTH", file=sys.stderr)
        sys.exit(1)

    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server_sock.bind(("D8:3A:DD:D0:CC:19", 6))
    except OSError as e:
        print(f"[ERROR] Bind failed: {e}", file=sys.stderr)
        sys.exit(1)

    server_sock.listen(1)
    print("[Link] Listening on D8:3A:DD:D0:CC:19:3. Waiting for connection…")

    try:
        while True:
            client_sock, client_info = server_sock.accept()
            threading.Thread(target=handle_client, args=(client_sock, client_info), daemon=True).start()
    except KeyboardInterrupt:
        print("\n[Main] Shutting down.")
    finally:
        server_sock.close()

if __name__ == "__main__":
    print(f"[Debug] __name__ = {__name__}")
    main()

