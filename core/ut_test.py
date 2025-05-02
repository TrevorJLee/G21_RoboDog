#!/usr/bin/env python3
import serial
import struct
import time
import math
import numpy as np
import glob
import threading
import random
from collections import deque

# === Constants ===
UWB_BAUD = 115200
LIDAR_BAUD = 230400
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_THRESHOLD_FT = 2.0
BASELINE_FT = 1.0
METERS_TO_FEET = 3.28084
alpha = 0.4

# === Shared State ===
prev_vx = prev_yaw = 0.0
r1_history = deque(maxlen=2)  # ⬅️ increased for better smoothing
r2_history = deque(maxlen=2)
uwb_data = {'A1': [], 'A2': []}
lidar_buffer = bytearray()
points_buffer = []
lock = threading.Lock()

def parse_packet(packet):
    if packet[0] != 0x54 or packet[1] != 0x2C:
        return []

    start_angle = struct.unpack_from("<H", packet, 4)[0] / 100.0
    end_angle = struct.unpack_from("<H", packet, 42)[0] / 100.0
    angle_step = (end_angle - start_angle) / 11.0 if end_angle > start_angle else ((end_angle + 360) - start_angle) / 11.0

    points = []
    for i in range(12):
        offset = 6 + i * 3
        dist_mm = struct.unpack_from("<H", packet, offset)[0]
        confidence = packet[offset + 2]
        angle = (start_angle + i * angle_step) % 360
        dist_ft = dist_mm / 1000.0 * 3.28084
        rel_angle = angle if angle <= 180 else angle - 360
        if confidence > 0 and dist_ft > 0.05 and -60 <= rel_angle <= 60:
            points.append((rel_angle, dist_ft))
    return points

def read_lidar():
    try:
        ser = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=1)
        print(f"[LIDAR] Connected on {LIDAR_PORT} @ {LIDAR_BAUD} baud")
        buffer = bytearray()
        while True:
            buffer.extend(ser.read(1))
            if len(buffer) >= 47:
                if buffer[0] == 0x54 and buffer[1] == 0x2C:
                    packet = buffer[:47]
                    with lock:
                        points = parse_packet(packet)
                        points_buffer.extend(points)
                    buffer = buffer[47:]
                else:
                    buffer.pop(0)
    except Exception as e:
        print(f"[LIDAR] Error: {e}")

def detect_uwb_anchors():
    print("[UWB] Scanning /dev/ttyACM* ports for anchors...")
    ports = glob.glob("/dev/ttyACM*")
    detected = []

    for port in ports:
        try:
            ser = serial.Serial(port, UWB_BAUD, timeout=1)
            start = time.time()
            while time.time() - start < 2:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if "Distance : " in line:
                    print(f"[UWB] {port} responded with: {line}")
                    detected.append(port)
                    break
            ser.close()
        except Exception as e:
            print(f"[UWB] {port} error: {e}")

    if len(detected) < 2:
        raise RuntimeError("[UWB] Not enough anchors detected. Need 2.")

    print(f"[UWB] Assigned Anchor 1 → {detected[0]} | Anchor 2 → {detected[1]}")
    return detected[0], detected[1]

def read_uwb(serial_port, key):
    try:
        ser = serial.Serial(serial_port, UWB_BAUD, timeout=1)
        print(f"[UWB] Reading from {serial_port} as {key}")
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if "Distance : " in line:
                try:
                    dist_m = float(line.split("Distance : ")[1])
                    dist_ft = dist_m * METERS_TO_FEET
                    if 0 < dist_ft <= 100:
                        with lock:
                            uwb_data[key].append(dist_ft)
                except:
                    pass
            time.sleep(0.005)
    except Exception as e:
        print(f"[UWB] Failed to read {serial_port}: {e}")

def main():
    a1_port, a2_port = detect_uwb_anchors()
    threading.Thread(target=read_uwb, args=(a1_port, 'A1'), daemon=True).start()
    threading.Thread(target=read_uwb, args=(a2_port, 'A2'), daemon=True).start()
    threading.Thread(target=read_lidar, daemon=True).start()

    print("[System] Threads started. Waiting for UWB data...\n")
    while True:
        with lock:
            if len(uwb_data['A1']) > 0 and len(uwb_data['A2']) > 0:
                break
        time.sleep(0.1)
    print("[System] UWB anchors responsive. Starting tracking...\n")

    global prev_vx, prev_yaw
    while True:
        time.sleep(0.75)

        # === Closest LIDAR object
        with lock:
            front_points = [(a, d) for a, d in points_buffer if -60 <= a <= 60]
        if front_points:
            angle, dist = min(front_points, key=lambda x: x[1])
            print(f"[Closest] {dist:.2f} ft at {angle:.2f}°")
            if dist <= LIDAR_THRESHOLD_FT:
                vx = 0.30
                yaw = math.radians(angle) * 1.5
                print(f"[Obstacle] Distance: {dist:.2f} ft | Angle: {angle:.2f}°")
                print(f"[Obstacle] Velocity: {vx:.2f} ft/s | Yaw Rate: {yaw:.2f} rad/s")
                print("[MODE] LIDAR override active\n")
                points_buffer.clear()
                continue
            points_buffer.clear()
        else:
            print("[Closest] No valid object in front 120°")

        # === UWB Tracking
        with lock:
            d1s = uwb_data['A1'][-5:]
            d2s = uwb_data['A2'][-5:]
        if d1s and d2s:
            r1 = np.median(d1s)
            r2 = np.median(d2s)
            r1_history.append(r1)
            r2_history.append(r2)
            r1f = np.median(r1_history)
            r2f = np.median(r2_history)
            diff = r2f - r1f

            print(f"[UWB Raw] r1 = {r1f:.2f} ft | r2 = {r2f:.2f} ft | Δ={diff:.2f} ft")

            # === Clamp small deltas to 0 to avoid noise-induced drift
            if abs(diff) < 0.1:
                angle = 0.0
            else:
                angle = math.degrees(math.atan2(diff, BASELINE_FT))

            angle = max(min(angle, 45), -45)
            avg_dist = (r1f + r2f) / 2
            vx_raw = 0.30 if avg_dist > 1 else 0.0
            yaw_raw = math.radians(angle) * 1.5
            vx = alpha * vx_raw + (1 - alpha) * prev_vx
            yaw = alpha * yaw_raw + (1 - alpha) * prev_yaw
            prev_vx, prev_yaw = vx, yaw

            print(f"[UWB] A1={r1f:.2f} ft | A2={r2f:.2f} ft | Δ={diff:.2f} ft")
            print(f"[Follow] Distance: {avg_dist:.2f} ft | Angle: {angle:.2f}°")
            print(f"[Follow] Velocity: {vx:.2f} ft/s | Yaw Rate: {yaw:.2f} rad/s")
            print("[MODE] Following UWB target\n")
        else:
            print("[UWB] Waiting for both anchors to return data...\n")

if __name__ == "__main__":
    main()

