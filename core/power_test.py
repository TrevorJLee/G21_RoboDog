import time
import numpy as np
from QuadrupedRobot.StanfordQuadruped.pupper.Config import ServoParams, PWMParams
from QuadrupedRobot.StanfordQuadruped.pupper.HardwareInterface import send_servo_command, angle_to_pwm

axis = 2  # calf
leg = 3   # back-right

servo_params = ServoParams()
pwm_params = PWMParams()

print("[PowerTest] WIDE SWEEP on back-right calf (axis 2, leg 2)")

# Sweep from -90° to 0° relative to absolute vertical (simulating full curl to extension)
sweep_deg = np.linspace(-60, 30, 80)

start_time = time.time()

for i, delta in enumerate(sweep_deg):
    angle_deg = servo_params.neutral_angle_degrees[axis, leg] + delta
    angle_rad = np.radians(angle_deg)

    pwm = angle_to_pwm(angle_rad, servo_params, axis, leg)
    send_servo_command(pwm_params, servo_params, angle_rad, axis, leg)

    print(f"[{i:02d}] Angle: {angle_deg:+.1f}° | PWM: {pwm:.1f} μs")
    time.sleep(0.04)

print(f"[PowerTest] Sweep complete in {time.time() - start_time:.2f} seconds.")

