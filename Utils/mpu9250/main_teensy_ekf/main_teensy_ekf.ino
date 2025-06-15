#include "mpu_ahrs_ekf.h"

unsigned long Time;

void setup() {
  Serial.begin(2000000);
  imu_setup();
  ahrs_setup();
}

void loop() {
  Time = millis();
  
  ahrs_update();
  Serial.print("quat\t");
  Serial.print(q[0]);
  Serial.print("\t");
  Serial.print(q[1]);
  Serial.print("\t");
  Serial.print(q[2]);
  Serial.print("\t");
  Serial.println(q[3]);

  while(millis()-Time<4);
}
