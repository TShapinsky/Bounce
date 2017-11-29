#include <math.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#define POT_PIN A0
#define SENSE_PIN 2

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *speaker = AFMS.getMotor(3);

String inputString;

void setup() {
  AFMS.begin();
  Serial.begin(115200);
  speaker -> setSpeed(255);
  speaker -> run(BACKWARD);
  inputString.reserve(200);
}

long waitTime = 0;

void loop() {
  while(digitalRead(SENSE_PIN) == LOW);
    long timeInit = micros();
    long fireTime = timeInit + waitTime + (((long)analogRead(POT_PIN)) - 512) * 20;
    while (micros() < fireTime);
    speaker -> run(FORWARD);
    delay(100);
    speaker -> run(BACKWARD);
  while(digitalRead(SENSE_PIN) == HIGH);
}

boolean inTransaction;
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    if (inChar == ' ') continue;
    if (inChar == '{') {
      inTransaction = true;
      inputString = "";
    }
    if (!inTransaction) {
      continue;
    }
    if (inChar == '}') {
      waitTime = inputString.substring(1).toInt();
    } else {
      inputString = inputString + inChar;
    }
  }
}



