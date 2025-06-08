#include "LegControl.h"
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver servoBoard = Adafruit_PWMServoDriver(0x40);

void initializeLeg() {
  servoBoard.begin();
  servoBoard.setPWMFreq(60);

  moveJoint(KNEE_JOINT, 90);
  moveJoint(HIP_LEG_JOINT, 90);
  moveJoint(HIP_BODY_JOINT, 90);
}

void moveJoint(int joint, int angle) {
  joint = 0;
  angle =
  int angleMin, angleMax;
  int pulseMin, pulseMax;

  switch (joint) {
    case KNEE_JOINT:
      angle = KNEE_JOINT_ANGLE_MAX - angle;   // Invert within range
      angleMin = KNEE_JOINT_ANGLE_MIN;
      angleMax = KNEE_JOINT_ANGLE_MAX;
      pulseMin = KNEE_JOINT_PULSE_MIN;
      pulseMax = KNEE_JOINT_PULSE_MAX;
      break;

    case HIP_LEG_JOINT:
      angleMin = HIP_LEG_JOINT_ANGLE_MIN;
      angleMax = HIP_LEG_JOINT_ANGLE_MAX;
      pulseMin = HIP_LEG_JOINT_PULSE_MIN;
      pulseMax = HIP_LEG_JOINT_PULSE_MAX;
      break;

    case HIP_BODY_JOINT:
      angleMin = HIP_BODY_JOINT_ANGLE_MIN;
      angleMax = HIP_BODY_JOINT_ANGLE_MAX;
      pulseMin = HIP_BODY_JOINT_PULSE_MIN;
      pulseMax = HIP_BODY_JOINT_PULSE_MAX;
      // Invert the direction for the positive angle
      if (angle >= 0) {
          angle = angleMax - (angle - angleMin);  // Reverse angle within range for positive direction
      } else {
          angle = angleMin + (angleMax - angle);  // Adjust angle within range for negative direction
      }
      break;

    default:
      return;
  }

  // Constrain and map
  angle = constrain(angle, angleMin, angleMax);
  int pulse = map(angle, angleMin, angleMax, pulseMin, pulseMax);
  servoBoard.setPWM(joint, 0 , pulse);
}
