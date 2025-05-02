import os
import time
import sys
import select
from PIL import Image

# Add full path to access Mangdang modules
sys.path.append("/home/ubuntu/Robotics/QuadrupedRobot")

from Mangdang.LCD.ST7789 import ST7789

cartoons_folder = "/home/ubuntu/Robotics/QuadrupedRobot/Mangdang/LCD/cartoons/"
RFCOMM_DEV = "/dev/rfcomm0"

# Determine hardware version and initialize display
with open("/home/ubuntu/.hw_version", "r") as hw_f:
    hw_version = hw_f.readline()

if hw_version == 'P1\n':
    disp = ST7789(14, 15, 47)
else:
    disp = ST7789(27, 24, 26)

disp.begin()
disp.clear()

def show_image(pic_name):
    path = os.path.join(cartoons_folder, pic_name)
    img = Image.open(path)
    img = img.resize((320, 240))
    disp.display(img)

def wait_for_connection():
    print("[Link] Starting up…", flush=True)
    show_image("logo.png")  # Step 1: initial logo

    print("[Link] Waiting for /dev/rfcomm0 …", flush=True)
    show_image("notconnect.png")  # Step 2: waiting state

    while not os.path.exists(RFCOMM_DEV):
        time.sleep(0.5)

    print("[Link] /dev/rfcomm0 found — client connected!", flush=True)
    show_image("logo.png")  # Step 3: re-show logo
    return open(RFCOMM_DEV, "r+b", buffering=0)

def receive_lines(port):
    while True:
        r, *_ = select.select([port], [], [], 0.5)
        if r:
            line = port.readline().decode().strip()
            if line:
                yield line

