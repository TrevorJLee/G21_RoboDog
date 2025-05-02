import threading
import time
import numpy as np

from QuadrupedRobot.StanfordQuadruped.src.Command import Command
from QuadrupedRobot.StanfordQuadruped.pupper.HardwareInterface import HardwareInterface
from QuadrupedRobot.StanfordQuadruped.pupper.Config import Configuration
from src.Controller import Controller
from src.State import State, BehaviorState
from src.MovementScheme import MovementScheme
from src.MovementGroup import MovementGroups
from pupper.Kinematics import four_legs_inverse_kinematics

command = Command()
state = State()
config = Configuration()
controller = Controller(config, four_legs_inverse_kinematics)
hardware_interface = HardwareInterface()

def _walk_for_5s(action_name, vx=0.0, vy=0.0, yaw=0.0):
    print(f"[Action] {action_name}")
    state.behavior_state = BehaviorState.TROT
    command.horizontal_velocity[0] = vx
    command.horizontal_velocity[1] = vy
    command.yaw_rate = yaw

    mg = MovementGroups()
    mg.gait_uni(v_x=vx, v_y=vy, time_uni=2, time_acc=1)
    movement_ctl = MovementScheme(mg.MovementLib)

    # Stabilization phase
    for _ in range(10):
        movement_ctl.runMovementScheme()
        foot_location = movement_ctl.getMovemenLegsLocation()
        attitude = movement_ctl.getMovemenAttitude()
        speed = movement_ctl.getMovemenSpeed()
        controller.run(state, command, foot_location, attitude, speed)
        hardware_interface.set_actuator_postions(state.joint_angles)
        time.sleep(config.dt)

    # Walking phase
    start_time = time.time()
    while time.time() - start_time < 5.0:
        movement_ctl.runMovementScheme()
        foot_location = movement_ctl.getMovemenLegsLocation()
        attitude = movement_ctl.getMovemenAttitude()
        speed = movement_ctl.getMovemenSpeed()
        command.horizontal_velocity[0] = speed[0]
        command.horizontal_velocity[1] = speed[1]
        controller.run(state, command, foot_location, attitude, speed)
        hardware_interface.set_actuator_postions(state.joint_angles)
        time.sleep(config.dt)

    stop()

def move_right():
    _walk_for_5s("Moving right", vy=-0.15)

def move_left():
    _walk_for_5s("Moving left", vy=0.15)

def move_forward():
    _walk_for_5s("Moving forward", vx=0.15)

def move_backward():
    _walk_for_5s("Moving backward", vx=-0.15)

def stop():
    print("[Action] Stopping and laying down")
    state.behavior_state = BehaviorState.REST
    command.horizontal_velocity[:] = 0.0
    command.yaw_rate = 0.0
    controller.run(state, command)
    hardware_interface.set_actuator_postions(state.joint_angles)

def dance():
    print("[Action] Dancing — Twist Shimmy Mode")

    state.behavior_state = BehaviorState.REST
    command.horizontal_velocity[:] = 0.0
    command.yaw_rate = 0.0

    # Start in rest pose
    controller.run(state, command)
    hardware_interface.set_actuator_postions(state.joint_angles)
    time.sleep(1.0)

    base_pose = np.copy(state.joint_angles)

    # Perform the twist shimmy
    duration = 6.0
    start_time = time.time()

    while time.time() - start_time < duration:
        t = time.time() - start_time
        twist = 0.25 * np.sin(8 * t)    # Hips twist motion
        counter = 0.1 * np.sin(8 * t + np.pi)  # Calves counter-motion

        pose = np.copy(base_pose)
        pose[0] += twist     # Hip twist
        pose[1] += 0.0       # Mid leg still
        pose[2] += counter   # Calf reacts

        hardware_interface.set_actuator_postions(pose)
        time.sleep(config.dt)

    # Reset to neutral
    controller.run(state, command)
    hardware_interface.set_actuator_postions(state.joint_angles)
    time.sleep(1.0)

    print("[Action] Dance complete.")

def come():
    _walk_for_5s("Coming to user", vx=0.15)

