#include <math.h>
#include <Wire.h>

#define POT_PIN A0
#define SENSE_PIN A2
#define SPK_F 2
#define SPK_B 3
#define THRESHHOLD 512
#define FORWARD 1
#define BACKWARD 0
#define NEUTRAL 2

String inputString;
void driveSpeaker(int direction);

void setup() {
  //Serial.begin(115200);
  pinMode(SPK_F, OUTPUT);
  pinMode(SPK_B, OUTPUT);
  driveSpeaker(NEUTRAL);
  inputString.reserve(200);
}

void driveSpeaker(int direction){
  if(direction == FORWARD){
    digitalWrite(SPK_F, HIGH);
    digitalWrite(SPK_B, LOW);
  }else if(direction == BACKWARD){
    digitalWrite(SPK_F, LOW);
    digitalWrite(SPK_B, HIGH);
  }else{
    digitalWrite(SPK_F, LOW);
    digitalWrite(SPK_B, LOW);
  }
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
    waitTime = 6970-lag;
  } else {
    waitTime = 3/(9.81*(timeInit-lastHit));
  }
  long fireTime = timeInit;// + waitTime + lag;
  lastHit = timeInit + waitTime;
  while (micros() < fireTime);
  driveSpeaker(FORWARD);
  busyDelay(100);
  driveSpeaker(BACKWARD);
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



