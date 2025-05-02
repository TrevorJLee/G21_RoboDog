import threading
import time
import math
import numpy as np
import serial
import struct
from collections import deque
from QuadrupedRobot.StanfordQuadruped.src.Command import Command
from QuadrupedRobot.StanfordQuadruped.pupper.HardwareInterface import HardwareInterface
from QuadrupedRobot.StanfordQuadruped.pupper.Config import Configuration
from src.Controller import Controller
from src.State import State, BehaviorState
from src.MovementScheme import MovementScheme
from src.MovementGroup import MovementGroups
from pupper.Kinematics import four_legs_inverse_kinematics

_running = threading.Event()

ANCHOR1_PORT = "/dev/ttyACM1"
ANCHOR2_PORT = "/dev/ttyACM0"
LIDAR_PORT = "/dev/ttyUSB0"
UWB_BAUD = 115200
LIDAR_BAUD = 230400
BASELINE_FT = 1.0
METERS_TO_FEET = 3.28084
LIDAR_THRESHOLD_FT = 2.0
UWB_STOP_DISTANCE_FT = 3.0
alpha = 0.4
r1_history = deque(maxlen=2)
r2_history = deque(maxlen=2)
prev_vx = prev_yaw = 0.0

command = Command()
state = State()
config = Configuration()
controller = Controller(config, four_legs_inverse_kinematics)
hardware_interface = HardwareInterface()

uwb_data = {'A1': [], 'A2': []}
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
        dist_ft = dist_mm / 1000.0 * METERS_TO_FEET
        rel_angle = angle if angle <= 180 else angle - 360
        if confidence > 0 and dist_ft > 0.05 and -60 <= rel_angle <= 60:
            points.append((rel_angle, dist_ft))
    return points

def read_lidar():
    try:
        ser = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=1)
        print(f"[LIDAR] Connected on {LIDAR_PORT} @ {LIDAR_BAUD} baud")
        buffer = bytearray()
        while _running.is_set():
            buffer.extend(ser.read(1))
            if len(buffer) >= 47:
                if buffer[0] == 0x54 and buffer[1] == 0x2C:
                    packet = buffer[:47]
                    with lock:
                        points_buffer.extend(parse_packet(packet))
                    buffer = buffer[47:]
                else:
                    buffer.pop(0)
    except Exception as e:
        print(f"[LIDAR] Error: {e}")

def read_distance(serial_port, key):
    try:
        ser = serial.Serial(serial_port, UWB_BAUD, timeout=1)
        print(f"[UWB] Opened {serial_port}")
        while _running.is_set():
            if ser.in_waiting > 0:
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

def start():
    if _running.is_set():
        return
    _running.set()
    print("[UserTracking] toggled ON", flush=True)
    threading.Thread(target=_loop, daemon=True).start()

def stop():
    _running.clear()
    print("[UserTracking] toggled OFF", flush=True)
    halt_motion()

def halt_motion():
    command.horizontal_velocity[:] = 0.0
    command.yaw_rate = 0.0
    state.behavior_state = BehaviorState.REST
    controller.run(state, command)
    hardware_interface.set_actuator_postions(state.joint_angles)
    print("[Follow] Stopping robot.")

def _loop():
    global prev_vx, prev_yaw
    threading.Thread(target=read_distance, args=(ANCHOR1_PORT, 'A1'), daemon=True).start()
    threading.Thread(target=read_distance, args=(ANCHOR2_PORT, 'A2'), daemon=True).start()
    threading.Thread(target=read_lidar, daemon=True).start()
    print("[UWB] Reader threads started.")

    state.behavior_state = BehaviorState.TROT
    mg = MovementGroups()
    mg.gait_uni(v_x=0.0, v_y=0.0, time_uni=2, time_acc=1)
    movement_ctl = MovementScheme(mg.MovementLib)

    def gait_loop():
        while _running.is_set():
            movement_ctl.runMovementScheme()
            fl = movement_ctl.getMovemenLegsLocation()
            att = movement_ctl.getMovemenAttitude()
            spd = movement_ctl.getMovemenSpeed()
            controller.run(state, command, fl, att, spd)
            hardware_interface.set_actuator_postions(state.joint_angles)
            time.sleep(config.dt)

    threading.Thread(target=gait_loop, daemon=True).start()
    print("[UserTracking] Movement control loop started.")

    while _running.is_set():
        time.sleep(0.75)

        # LIDAR override
        with lock:
            front = [(a, d) for a, d in points_buffer if -60 <= a <= 60]
        if front:
            angle, dist = min(front, key=lambda x: x[1])
            print(f"[Closest] {dist:.2f} ft at {angle:.2f}°")
            if dist <= LIDAR_THRESHOLD_FT:
                vx = 0.30
                yaw = math.radians(angle) * 1.5
                command.horizontal_velocity[0] = vx
                command.horizontal_velocity[1] = 0.0
                command.yaw_rate = yaw
                print(f"[Obstacle] Distance: {dist:.2f} ft | Angle: {angle:.2f}°")
                print(f"[Obstacle] Velocity: {vx:.2f} ft/s | Yaw Rate: {yaw:.2f} rad/s")
                print("[MODE] LIDAR override active\n")
                points_buffer.clear()
                continue
            points_buffer.clear()
        else:
            print("[Closest] No valid object in front 120°")

        # UWB logic
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

            if abs(diff) < 0.1:
                angle = 0.0
            else:
                angle = math.degrees(math.atan2(diff, BASELINE_FT))

            angle = max(min(angle, 45), -45)
            avg_dist = (r1f + r2f) / 2

            if avg_dist <= UWB_STOP_DISTANCE_FT:
                halt_motion()
                print(f"[Follow] Target within {UWB_STOP_DISTANCE_FT} ft — robot stopped\n")
                continue

            vx_raw = 0.30 if avg_dist > 1 else 0.0
            yaw_raw = math.radians(angle) * 1.5
            vx = alpha * vx_raw + (1 - alpha) * prev_vx
            yaw = alpha * yaw_raw + (1 - alpha) * prev_yaw
            prev_vx, prev_yaw = vx, yaw

            print(f"[UWB] A1={r1f:.2f} ft | A2={r2f:.2f} ft | Δ={diff:.2f} ft")
            print(f"[Follow] Distance: {avg_dist:.2f} ft | Angle: {angle:.2f}°")
            print(f"[Follow] Velocity: {vx:.2f} ft/s | Yaw Rate: {yaw:.2f} rad/s")
            print("[MODE] Following UWB target\n")

            command.horizontal_velocity[0] = vx
            command.horizontal_velocity[1] = 0.0
            command.yaw_rate = yaw
        else:
            print("[UWB] Incomplete distance data\n")
            halt_motion()

