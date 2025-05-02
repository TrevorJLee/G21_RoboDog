#!/usr/bin/env python3
import serial
import struct
import time
import math

PORT = "/dev/ttyUSB0"
BAUD = 230400
FIELD_OF_VIEW_DEGREES = 120  # ±60°

def parse_packet(packet):
    if packet[0] != 0x54 or packet[1] != 0x2C:
        return []

    # Extract angles
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
        if confidence > 0 and dist_ft > 0.05:
            points.append((angle, dist_ft))
    return points

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"[LIDAR] Connected on {PORT} @ {BAUD} baud")

        buffer = bytearray()
        points_buffer = []

        last_print_time = time.time()
        while True:
            buffer.extend(ser.read(1))
            if len(buffer) >= 47:
                if buffer[0] == 0x54 and buffer[1] == 0x2C:
                    packet = buffer[:47]
                    buffer = buffer[47:]

                    points = parse_packet(packet)
                    filtered = [(a if a <= 180 else a - 360, d) for a, d in points if -60 <= (a if a <= 180 else a - 360) <= 60]
                    points_buffer.extend(filtered)

                    if time.time() - last_print_time >= 1.0:
                        if points_buffer:
                            angle, distance = min(points_buffer, key=lambda x: x[1])
                            print(f"Closest object: {distance:.2f} ft at {angle:.2f}°")
                            points_buffer.clear()
                        else:
                            print("No valid targets in front 120°")
                        last_print_time = time.time()
                else:
                    buffer.pop(0)
    except Exception as e:
        print(f"[Error] {e}")

if __name__ == "__main__":
    main()

