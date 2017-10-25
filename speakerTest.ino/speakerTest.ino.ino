#include <Math.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#define POT0_PIN A0

#define IR_PIN_DOWN  A1
#define IR_PIN_UP    A2

#define UPPER 0
#define LOWER 1

#define SENSOR_TIMEOUT 1000*100       //100 ms
#define PHASE_TIMEOUT  1000*1000*1000 //1  s

#define DEBUG_PLOT 0

typedef enum {
  NO_BALL,
  UP,
  BETWEEN,
  DOWN,
  BELOW
} phase_t;

String phaseNames[] = {"No ball", "up", "between", "down", "below"};

double ballDiameter = 4.2;
double sensorDist   = 23;
double speakerDist  = 13.5;
long lag = 0;

phase_t phase = NO_BALL;

float speeds[2];
long enterTimes[2];
long exitTimes[2];
bool inFrontOfSensor[2];
long lastSense;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *speaker = AFMS.getMotor(2);

void setup() {
  // put your setup code here, to run once:
  AFMS.begin();
  Serial.begin(9600);
  speaker->setSpeed(0);
  speaker->run(BACKWARD);
  lastSense = micros();
  #if !DEBUG_PLOT
    Serial.println("settup complete");
  #endif
}

double rescale(double in, double minB, double maxB, double minA, double maxA) {
  return (in-minB)/maxB*(maxA-minA)+minA;
}

void resetState() {
  for(int i = 0; i<2; i++) {
    inFrontOfSensor[i] = false;
  }
  speaker->setSpeed(0);
  lastSense = micros();
  phase     = NO_BALL;
}

void ballEntered(int sensor) {
  enterTimes[sensor] = micros();
  inFrontOfSensor[sensor] = true;
  if(sensor == UPPER) {
    phase = UP;
  } else if(sensor == LOWER) {
    phase = DOWN;
  }
  lastSense = micros();
}

void ballLeft(int sensor) {
  speeds[sensor] = ballDiameter/(micros() - enterTimes[sensor]);
  inFrontOfSensor[sensor] = false;
  exitTimes[sensor] = micros();
  if(sensor == UPPER) {
    phase = BETWEEN;
  } else if (sensor == LOWER) {
    phase = BELOW;
  }
  lastSense = micros();
}
int counter = 0;

void pollSensors() {
  int upperVal = analogRead(IR_PIN_UP);
  int lowerVal = analogRead(IR_PIN_DOWN);
  int BALL_THRESHHOLD_UPPER = 10;
  int BALL_THRESHHOLD_LOWER = 10;
  #if DEBUG_PLOT
    if(counter == 100) {
    Serial.print(upperVal);
    Serial.print("\t");
    Serial.print(BALL_THRESHHOLD_UPPER);
    Serial.print("\t");
    Serial.print(lowerVal);
    Serial.print("\t");
    Serial.print(BALL_THRESHHOLD_LOWER);
    Serial.println();
    counter = 0;
    } else {
      counter++;
    }
  #endif
  if(upperVal < BALL_THRESHHOLD_UPPER) {
    ballEntered(UPPER);
  } else if (upperVal >= BALL_THRESHHOLD_UPPER && inFrontOfSensor[UPPER]) {
    ballLeft(UPPER);
  }
  if(lowerVal < BALL_THRESHHOLD_LOWER) {
    ballEntered(LOWER);
  } else if (lowerVal >= BALL_THRESHHOLD_LOWER && inFrontOfSensor[LOWER]) {
    ballLeft(LOWER);
  }
}

void pollTimeouts() {
  for(int i = 0; i < 2; i++) {
    if(inFrontOfSensor[i] && (micros() - enterTimes[i]) > SENSOR_TIMEOUT) {
      resetState();
      #if !DEBUG_PLOT
        Serial.print("State reset, timeout on sensor ");
        Serial.println(i);
      #endif
    }
  }
  if(micros() - lastSense > PHASE_TIMEOUT && phase != NO_BALL) {
    resetState();
    #if !DEBUG_PLOT
      Serial.println("State reset, timeout on ball");
    #endif
  }
}

long fireTime;
float speedOverall;

void loop() {
  if(phase == BELOW) {
    while(micros() < fireTime);
    speaker->run(FORWARD);
    delay(100);
    speaker->run(BACKWARD);
    speaker->setSpeed(0);
    delay(100);
    resetState();
    Serial.println("Fired, resetting");
    return;
  }
  phase_t lastPhase = phase;
  pollSensors();
  pollTimeouts();
  if(phase == BELOW && lastPhase == DOWN) {//edge detection!
    double a = 9.81e-10;
    double lag = rescale(analogRead(POT0_PIN),0,1024,0,100000);
    Serial.print("Lag = ");
    Serial.println(lag);
    double v = sensorDist/(exitTimes[LOWER] - exitTimes[UPPER]);
    double d = speakerDist - ballDiameter;
    double t = d/v;
    if(t>1e6) {
      Serial.println("missfire");
      resetState();
      return;
    }
    fireTime = micros() + t - lag;
    return;
  } if(phase == BETWEEN && lastPhase == UP) {
    Serial.println("priming");
    speaker->setSpeed(255);
  }
}
