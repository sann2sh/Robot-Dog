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

    moveLeg(LEG_1, KNEE_JOINT, knee);
    moveLeg(LEG_1, HIP_LEG_JOINT, hipLeg);
    moveLeg(LEG_1, HIP_BODY_JOINT, hipBody);

    Serial.read();

    Serial.print(knee);
    Serial.print("  ");
    Serial.print(hipLeg);
    Serial.print("  ");
    Serial.print(hipBody);
    Serial.println();
  }
}
