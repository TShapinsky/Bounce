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
  Serial.begin(115200);
  speaker -> setSpeed(255);
  speaker -> run(BACKWARD);
  inputString.reserve(200);
}

long waitTime = 0;
int prime = 0;

void loop() {
  //while(!prime);
  speaker -> setSpeed(255);
  speaker -> run(BACKWARD); 
  while(analogRead(SENSE_PIN) < THRESHHOLD);
    long timeInit = micros();
    //long fireTime = timeInit + 6090;
    long fireTime = timeInit + waitTime + map(analogRead(POT_PIN),0,1024,-500,500);
    while (micros() < fireTime);
    speaker -> run(FORWARD);
    delay(100);
    speaker -> run(BACKWARD);
    //Serial.println(((long)analogRead(POT_PIN) * 30));
    //prime = 0;
  while(analogRead(SENSE_PIN) >= THRESHHOLD);
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



