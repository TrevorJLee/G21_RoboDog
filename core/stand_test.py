#!/usr/bin/env python3

import time
from QuadrupedRobot.StanfordQuadruped.src.Command import Command
from QuadrupedRobot.StanfordQuadruped.pupper.HardwareInterface import HardwareInterface
from QuadrupedRobot.StanfordQuadruped.pupper.Config import Configuration
from src.Controller import Controller
from src.State import State, BehaviorState
from pupper.Kinematics import four_legs_inverse_kinematics

print("[Test] Initializing stand pose test...")

# Initialize control system
command = Command()
state = State()
config = Configuration()
controller = Controller(config, four_legs_inverse_kinematics)
hardware_interface = HardwareInterface()

# Force static posture
state.behavior_state = BehaviorState.REST
command.horizontal_velocity[:] = 0.0
command.yaw_rate = 0.0

# Run the control loop once to apply joint angles
controller.run(state, command)
hardware_interface.set_actuator_postions(state.joint_angles)

print("[Test] RoboDog should now be in standing/rest pose based on config.")
time.sleep(5)
print("[Test] Done.")

