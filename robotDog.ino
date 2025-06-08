#include "LegControl.h"

void setup() {
  Serial.begin(9600);
  Serial.println("Robot Dog Leg Control");

  initializeLeg();  // Setup servos
}

void loop() {
  if (Serial.available()) {
    int knee = Serial.parseInt();
    int hipLeg = Serial.parseInt();
    int hipBody = Serial.parseInt();

    moveJoint(KNEE_JOINT, knee);
    moveJoint(HIP_LEG_JOINT, hipLeg);
    moveJoint(HIP_BODY_JOINT, hipBody);

    Serial.read();

    Serial.print(knee);
    Serial.print("  ");
    Serial.print(hipLeg);
    Serial.print("  ");
    Serial.print(hipBody);
    Serial.println();
  }
}
