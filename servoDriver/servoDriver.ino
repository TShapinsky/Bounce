#include <math.h>
#include <Servo.h>
#define PINS {3,5,6,9,10,11}
#define MINS {620,850,810,820,540,800}
#define MAXS {2080,2310,2270,2280,2000,2260}
#define N_INPUTS 6
#define N_DELIMS (N_INPUTS-1)
#define READ_THETAS 1
#define READ_INTS 1
#define RADIANS(x) ((x)/180.0 * PI)
#define DEGREES(x) ((x)/PI * 180.0)

const int pins[] = PINS;
const int mins[] = MINS;
const int maxs[] = MAXS;
Servo servos[6];
#if(READ_INTS)
uint16_t params[N_INPUTS];
uint16_t thetas[N_INPUTS];
#else
float params[N_INPUTS];
float thetas[6];
#endif
String inputString;
bool newParams = false;



float remap(float a, float minB, float maxB, float minA, float maxA) {
  return (a-minB)/(maxB-minB)*(maxA-minA)+minA;
}

#if READ_INTS
void moveToAngle(int servo, uint16_t angle) {
  servos[servo].writeMicroseconds(angle);
}
#else
void moveToAngle(int servo, float angle) {
  if(servo%2==0) {
    servos[servo].writeMicroseconds(remap(angle,RADIANS(-70),RADIANS(70),mins[servo],maxs[servo]));
  } else {
    servos[servo].writeMicroseconds(remap(angle,RADIANS(-70),RADIANS(70),maxs[servo],mins[servo]));
  }
}
#endif

void moveToAngles() {
  #if READ_INTS
  for(int i = 0; i<6; ++i) {
    if(thetas[i] > maxs[i]) {
      thetas[i] = maxs[i];
    }
    if(thetas[i] < mins[i]) {
      thetas[i] = mins[i];
    }
    moveToAngle(i,thetas[i]);
  }
  #else
  for(int i = 0; i<6; ++i) {
    if(thetas[i] > PI/2) {
      thetas[i] = PI/2;
    }
    if(thetas[i] < -PI/2) {
      thetas[i] = -PI/2;
    }
    moveToAngle(i,thetas[i]);
  }
  #endif
}

long startTime;
void setup() {
  float thetaB = 60 * 2 * PI / 360;
  float thetaT = 5 * 2 * PI / 360;
  float rb = 4;
  float rt = 3;
  for(int i = 0; i<6; ++i) {
    servos[i].attach(pins[i],mins[i],maxs[i]);
  }
  startTime = millis();
  pinMode(13,OUTPUT);
  Serial.begin(115200);
  inputString.reserve(200);
  params[0] = 0;
  params[1] = 0;
  params[2] = 0;
  params[3] = 0;
  params[4] = 0;
  params[5] = 0;
  newParams = true;
}

void loop() {
  if(newParams) {
      #if READ_THETAS
        for(int i = 0; i<6; ++i) {
          thetas[i] = params[i];
        }
        moveToAngles();
      #else
        if(calculateAngles()) {
           moveToAngles();
           digitalWrite(13, LOW);
        } else {
          digitalWrite(13, HIGH);
        }
      #endif
      newParams = false;
  }
}

bool inTransaction;
int currentParam = 0;
int currentByte = 0;
void serialEvent() {
  if(newParams) {
      while(Serial.available()) Serial.read();
      return;
  }
  while (Serial.available()) {
    #if READ_INTS
      uint16_t inByte = (uint16_t)Serial.read();
      if(currentByte == 0) {
        params[currentParam] = inByte<<8;
        currentByte++;
      }else{
        params[currentParam] |= inByte;
        currentParam++;
        currentByte = 0;
        if(currentParam == N_INPUTS) {
          newParams = true;
          currentParam = 0;
          while(Serial.available()) Serial.read();
          return;
        }
      }


    #else
      // get the new byte:
      char inChar = (char)Serial.read();
      if(inChar == ' ') continue;
      if(inChar == '{') {
        inTransaction = true;
        inputString = "";
      }
      if(!inTransaction) {
        continue;
      }
      if(inChar == '}') {
        String vals = inputString.substring(1);
        int delimIndicies[N_DELIMS] = {0};
        float values[N_INPUTS];
        delimIndicies[0] = vals.indexOf(',');
        if(delimIndicies[0] == -1) {
          goto kickout;
        }
        values[0] = vals.substring(0,delimIndicies[0]).toFloat()/10000.0;
        for(int i = 1; i<N_DELIMS; ++i) {
          delimIndicies[i] = vals.indexOf(',',delimIndicies[i-1]+1);
          values[i] = vals.substring(delimIndicies[i-1]+1,delimIndicies[i]).toFloat()/10000.0;
        }
        values[N_INPUTS-1] = vals.substring(delimIndicies[N_DELIMS-1]+1).toFloat()/10000.0;
        for(int i = 0; i<N_INPUTS; ++i) {
          params[i] = values[i];
        }
        newParams = true;
        kickout:
        inTransaction = false;
        inputString = "";
      } else {
        inputString = inputString + inChar;
      }
    #endif
  }
}
