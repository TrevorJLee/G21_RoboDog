import time
from QuadrupedRobot.StanfordQuadruped.src.Command import Command
from QuadrupedRobot.StanfordQuadruped.pupper.Config import Configuration
from QuadrupedRobot.StanfordQuadruped.pupper.HardwareInterface import HardwareInterface
from src.Controller import Controller
from src.State import State, BehaviorState
from src.MovementScheme import MovementScheme
from src.MovementGroup import MovementGroups
from pupper.Kinematics import four_legs_inverse_kinematics

print("[TrotTest] Starting TROT-in-place test...")

command = Command()
state = State()
config = Configuration()
controller = Controller(config, four_legs_inverse_kinematics)
hardware_interface = HardwareInterface()

# TROT-in-place setup
state.behavior_state = BehaviorState.TROT
command.horizontal_velocity[0] = 0.0
command.horizontal_velocity[1] = 0.0
command.yaw_rate = 0.0

# Generate movement group
mg = MovementGroups()
mg.gait_uni(v_x=0.0, v_y=0.0, time_uni=2, time_acc=1)  # zero velocity = in-place
movement_ctl = MovementScheme(mg.MovementLib)

runtime = 5.0
start = time.time()
print(f"[TrotTest] Executing for {runtime} seconds...")

while time.time() - start < runtime:
    movement_ctl.runMovementScheme()
    foot_location = movement_ctl.getMovemenLegsLocation()
    attitude = movement_ctl.getMovemenAttitude()
    speed = movement_ctl.getMovemenSpeed()

    controller.run(state, command, foot_location, attitude, speed)
    hardware_interface.set_actuator_postions(state.joint_angles)

    # Print just the back calves
    print(f"[Back Legs] Right: {state.joint_angles[2][2]: .3f} | Left: {state.joint_angles[2][3]: .3f}")

    time.sleep(config.dt)

# Return to REST
print("[TrotTest] Resetting to REST...")
state.behavior_state = BehaviorState.REST
command.horizontal_velocity[:] = 0.0
command.yaw_rate = 0.0
controller.run(state, command)
hardware_interface.set_actuator_postions(state.joint_angles)
time.sleep(1.0)

print("[TrotTest] Complete.")

