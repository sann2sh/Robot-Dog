# config.py

# Serial communication settings
SERIAL_PORT = "COM8"  # or "/dev/ttyUSB0" for Linux/macOS
BAUD_RATE = 9600

# ==== Robot Leg Parameters ====

NUM_LEGS = 4
NUM_JOINTS = 3

JOINT_NAMES = ["Knee", "Hip-Leg", "Hip-Body"]
JOINT_RANGES = [(0, 150), (-247, -43), (-43, 37)]
# Joint angles initial position
JOINT_ANGLES_INIT = [(JOINT_RANGES[i][1] + JOINT_RANGES[i][0]) / 2 for i in range(NUM_JOINTS)]

HIP_KNEE_LENGTH = 100  # Hip to knee
KNEE_FOOT_LENGTH = 156  # Knee to foot
