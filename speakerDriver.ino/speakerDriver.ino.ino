#include <math.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#define POT_PIN A0
#define SENSE_PIN A1
#define THRESHHOLD 512

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *speaker = AFMS.getMotor(3);

String inputString;

void setup() {
  AFMS.begin();
  //Serial.begin(115200);
  speaker -> setSpeed(255);
  speaker -> run(BACKWARD);
  inputString.reserve(200);
}

long waitTime = 0;
int prime = 0;
long lastHit = -1;

void busyDelay(long waitInMillis) {
  long timeInit = millis();
  long fireTime = timeInit + waitInMillis;
  while(millis() < fireTime);
}

void loop() {
  while(analogRead(SENSE_PIN)<THRESHHOLD);
  long timeInit = micros();
  long lag = analogRead(POT_PIN)*4;
  if(lastHit == -1 || (timeInit-lastHit)>2000000) {
    waitTime = 6970+lag;
  } else {
    waitTime = 3/(9.81*(timeInit-lastHit));
  }
  long fireTime = timeInit + waitTime - lag;
  while (micros() < fireTime);
  speaker -> run(FORWARD);
  Serial.begin(115200);
  busyDelay(100);
  speaker -> run(BACKWARD);
  busyDelay(150);
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
      //prime = 1;
    } else {
      inputString = inputString + inChar;
    }
  }
}



