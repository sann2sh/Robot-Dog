#include "LegControl.h"

Adafruit_PWMServoDriver servoBoard = Adafruit_PWMServoDriver(0x40);

void initializeLeg() {
  servoBoard.begin();
  servoBoard.setPWMFreq(60);

  for (int leg = 1; leg <= 4; leg++) {
    for (int joint = 1; joint <= 3; joint++) {
      int pulse = 0;
      switch (joint) {
        case KNEE_JOINT:
          pulse = (KNEE_JOINT_PULSE_MIN + KNEE_JOINT_PULSE_MAX) / 2;
          break;
        case HIP_LEG_JOINT:
          pulse = (HIP_LEG_JOINT_PULSE_MIN + HIP_LEG_JOINT_PULSE_MAX) / 2;
          break;
        case HIP_BODY_JOINT:
          pulse = (HIP_BODY_JOINT_PULSE_MIN + HIP_BODY_JOINT_PULSE_MAX) / 2;
          break;
      }
      servoBoard.setPWM((leg-1) * 3 + joint - 1, 0, pulse);
    }
  }
}

void moveLeg(int leg, int joint, int angle) {
  int angleMin, angleMax;
  int pulseMin, pulseMax;

  switch (leg) {
    case LEG_1:
      
    case LEG_3:
      break;  // No specific action needed for leg selection
    default:
      return;  // Invalid leg
  }

  switch (joint) {
    case KNEE_JOINT:
      angleMin = KNEE_JOINT_ANGLE_MIN;
      angleMax = KNEE_JOINT_ANGLE_MAX;
      pulseMin = KNEE_JOINT_PULSE_MIN;
      pulseMax = KNEE_JOINT_PULSE_MAX;
      if (leg == 2 || leg == 4) {
        angle = angleMax + angleMin - angle;   // Invert within range
      }
      break;

    case HIP_LEG_JOINT:
      angleMin = HIP_LEG_JOINT_ANGLE_MIN;
      angleMax = HIP_LEG_JOINT_ANGLE_MAX;
      pulseMin = HIP_LEG_JOINT_PULSE_MIN;
      pulseMax = HIP_LEG_JOINT_PULSE_MAX;
      if (leg == 1  || leg == 3) {
        angle = angleMax + angleMin - angle;   // Invert within range
      }
      break;

    case HIP_BODY_JOINT:
      angleMin = HIP_BODY_JOINT_ANGLE_MIN;
      angleMax = HIP_BODY_JOINT_ANGLE_MAX;
      pulseMin = HIP_BODY_JOINT_PULSE_MIN;
      pulseMax = HIP_BODY_JOINT_PULSE_MAX;
      angle = angleMax + angleMin - angle;  // Reverse angle within range
      break;

    default:
      return;
  }

  // Constrain and map
  angle = constrain(angle, angleMin, angleMax);
  int pulse = map(angle, angleMin, angleMax, pulseMin, pulseMax);
  servoBoard.setPWM(leg * 3 + joint - 1, 0 , pulse);
}
