#include <LegControl.h>

#define NUM_LEGS 4
#define NUM_JOINTS 3

void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for serial on Leonardo-type boards
  Serial.println("Ready to receive joint angles...");
  initializeLeg();  // Setup servos

}

void loop() {
  if (Serial.available()) {
    int angles[NUM_LEGS][NUM_JOINTS];

    for (int i = 0; i < NUM_LEGS; i++) {
      angles[i][KNEE_JOINT] = Serial.parseInt();
      angles[i][HIP_LEG_JOINT] = Serial.parseInt();
      angles[i][HIP_BODY_JOINT] = Serial.parseInt();
    }
    Serial.read();

    for (int leg = 0; leg < NUM_LEGS; leg++) {
      moveLeg(leg, KNEE_JOINT, angles[leg][KNEE_JOINT]);
      moveLeg(leg, HIP_LEG_JOINT, angles[leg][HIP_LEG_JOINT]);
      moveLeg(leg, HIP_BODY_JOINT, angles[leg][HIP_BODY_JOINT]);
    }
  }
}
