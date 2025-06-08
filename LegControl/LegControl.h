#ifndef LEGCONTROL_H
#define LEGCONTROL_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

// Joint channel mappings
#define KNEE_JOINT 2
#define HIP_LEG_JOINT    1
#define HIP_BODY_JOINT   0

#define LEG_1 0
#define LEG_2 1
#define LEG_3 2
#define LEG_4 3

// Joint Pulse Limits
#define KNEE_JOINT_PULSE_MIN        220
#define KNEE_JOINT_PULSE_MAX        657
#define HIP_LEG_JOINT_PULSE_MIN     100
#define HIP_LEG_JOINT_PULSE_MAX     700
#define HIP_BODY_JOINT_PULSE_MIN    300
#define HIP_BODY_JOINT_PULSE_MAX    540

// Joint Angle Limits
#define KNEE_JOINT_ANGLE_MIN        0// 75 mid point
#define KNEE_JOINT_ANGLE_MAX        150
#define HIP_LEG_JOINT_ANGLE_MIN     -247// -145 mid point
#define HIP_LEG_JOINT_ANGLE_MAX     -43
#define HIP_BODY_JOINT_ANGLE_MIN    -43
#define HIP_BODY_JOINT_ANGLE_MAX    37

void initializeLeg();
void moveLeg(int leg, int joint, int angle);

#endif
