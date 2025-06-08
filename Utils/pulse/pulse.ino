#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); // Format: "0:300"
    int sep = input.indexOf(':');
    if (sep != -1) {
      int servoNum = input.substring(0, sep).toInt();
      int pulse = input.substring(sep + 1).toInt();
      pwm.setPWM(servoNum, 0, pulse);
    }
  }
}
