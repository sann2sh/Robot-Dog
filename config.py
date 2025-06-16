# config.py

# Serial communication settings
SERIAL_PORT_REC = "COM19"  # or "/dev/ttyUSB0" for Linux/macOS
SERIAL_PORT_SEND = "COM6"  # or "/dev/ttyUSB1" for Linux/macOS
# Baud rates for receiving and sending data
BAUD_RATE_REC = 2000000  # Baud rate for receiving MPU data from Arduino/Teensy
BAUD_RATE_SEND = 9600  # Baud rate for sending Motor Angle commands to Arduino

# ==== Robot Leg Parameters ====

NUM_LEGS = 4
NUM_JOINTS = 3

JOINT_NAMES = ["knee", "Hip-Leg", "Hip-Body"]
JOINT_RANGES = [(30, 180), (-150, 30), (-20, 20)]
# Joint angles initial position
JOINT_ANGLES_INIT = [
    (JOINT_RANGES[i][1] + JOINT_RANGES[i][0]) / 2 for i in range(NUM_JOINTS)
]

HIP_KNEE_LENGTH = 100  # Hip to knee
KNEE_FOOT_LENGTH = 156  # Knee to foot
