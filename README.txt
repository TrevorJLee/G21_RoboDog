RoboDog - Interactive Quadruped Robot

RoboDog is a fully integrated robotic quadruped system powered by Raspberry Pi and the Stanford Pupper framework. It combines Bluetooth voice control, gesture recognition, UWB-based tracking, and LiDAR obstacle avoidance to create a responsive, semi-autonomous robotic dog.
Features

    Voice Control (Android + Pi Backend)

        Bluetooth RFCOMM link for real-time commands.

        Google Cloud Text-to-Speech and Speech-to-Text integration.

        Uses ChatGPT for conversation with configurable personality and context.

        Actions include: forward, backward, right, left, come, stop, dance.

        Built-in UAH (University of Alabama in Huntsville) tour guide mode.

    Gesture Recognition

        Uses Mediapipe and TensorFlow Lite for hand gesture detection.

        Gesture mapping:

            Palm: Stop

            Fist: Come

            Thumb Up: Speak ("Woof!")

            Rock On: Dance

    UWB Tracking

        Dual-Anchor system (via /dev/ttyACM*).

        Smooth trilateration to track and follow a moving target.

    LiDAR Obstacle Avoidance

        360° LiDAR scans; focuses on 120° front cone.

        Overrides UWB tracking if an obstacle is within 2 feet.

    Android Bluetooth Link

        Shows notconnect.png while waiting for connection.

        Displays logo.png once connected.

Directory Structure

G21Controller/
├── core/
│ ├── actions.py # Movement logic (forward, back, stop, dance, etc.)
│ ├── gesture_recognition.py # Mediapipe + TFLite gesture control
│ ├── user_tracking.py # UWB + LiDAR tracking control loop
│ ├── voice_control.py # Voice mode: GPT + STT/TTS logic
│ ├── lidar_test.py # LiDAR test script (standalone)
│ ├── ut_test.py # UWB + LiDAR combined test
│ ├── stand_test.py # Test standing pose
│ ├── power_test.py # Servo sweep for calibration
│ ├── trot.py # Trot gait test
│ └── gesture_data/
│ ├── bark.wav, gesture_model.tflite, etc.
│
├── interface/
│ └── android_link.py # Handles Bluetooth + display images
│
├── main.py # Main Bluetooth RFCOMM server entry point
├── mode_manager.py # Command dispatcher for modes
├── utils.py # Helpers (e.g., ACK formatting)
├── input.wav / output.wav # Voice temp files
Setup

    Python Environment:

        Python 3.10+

        Install required libraries:

            openai

            google-cloud-texttospeech

            speechrecognition

            numpy

            opencv-python

            mediapipe

            tflite-runtime

            pillow

            pyserial

    Hardware:

        Raspberry Pi (tested on Ubuntu 22.04)

        Bluetooth RFCOMM configured + SPP service advertised

        LiDAR connected at /dev/ttyUSB0

        UWB Anchors at /dev/ttyACM0 and /dev/ttyACM1

        Servo/motor hardware powered and calibrated

    Environment Variables (already set in main.py):

        GOOGLE_APPLICATION_CREDENTIALS

        OPENAI_API_KEY

        DBUS_SESSION_BUS_ADDRESS

How to Run

    Start RoboDog Bluetooth handler:
    sudo PYTHONPATH=/home/ubuntu/Robotics python3 main.py

    Use the Android app to:

        Connect via Bluetooth.

        Select a mode: Voice, Gesture, or Follow.

        Voice mode: set personality/context and start a conversation.

        Gesture mode: perform gestures in view of the camera.

        Follow mode: move around with UWB tag; LiDAR avoids obstacles.

    For standalone tests:

        lidar_test.py – test LiDAR readings.

        ut_test.py – test combined UWB + LiDAR behavior.

        gesture_recognition.py – manually start gesture control.

Authors & Notes

    Built by Trevor Lee, Dylan Grimes, and Wes Love at UAH (University of Alabama in Huntsville).

    Based on MangDang MiniPupper Quadruped framework.

    Fully modular: each mode (voice, gesture, tracking) runs independently and can be toggled via Bluetooth.
