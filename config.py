# config.py

# Serial communication settings
SERIAL_PORT_REC = "COM19"  # or "/dev/ttyUSB0" for Linux/macOS
SERIAL_PORT_SEND = "COM8"  # or "/dev/ttyUSB1" for Linux/macOS
# Baud rates for receiving and sending data
BAUD_RATE_REC = 2000000  # Baud rate for receiving MPU data from Arduino/Teensy
BAUD_RATE_SEND = 2000000  # Baud rate for sending Motor Angle commands to Arduino

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

FOOT_POSITIONS_REST = [[-95, -195], [-95, -195], [95, -195], [95, -195]]
FOOT_POSITIONS_WALK = [[-50, -195], [-50, -195], [50, -195], [50, -195]]

STEP_LENGTH_X = 100  # Step length in X direction
STEP_LENGTH_Y = 70  # Step length in Y direction
