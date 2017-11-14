#include <MatrixMath.h>
#include <math.h>
#include <Servo.h>
#define PINS {3,5,6,9,10,11}
#define MINS {620,850,810,820,540,800}
#define MAXS {2080,2310,2270,2280,2000,2260}
#define CROSS(a,b) {           \
  (a)[1]*(b)[2]-(a)[2]*(b)[1], \
  (a)[2]*(b)[0]-(a)[0]*(b)[2], \
  (a)[0]*(b)[1]-(a)[1]*(b)[0], \
  0                            \
  }
#define NORM(a)    (sqrt((a)[0]*(a)[0] + (a)[1]*(a)[1] + (a)[2]*(a)[2]))
#define INDEX(y,x,m,n) ((y)*(n)+(x))
#define H (12+5.0/8.0)
#define EPSILON .001
#define MAX_ITERATIONS 100
#define EQ(a,b) (abs((a)-(b))<EPSILON)
#define N_INPUTS 6
#define N_DELIMS (N_INPUTS-1)
#define READ_THETAS 1

const int pins[] = PINS;
const int mins[] = MINS;
const int maxs[] = MAXS;
Servo servos[6];
float params[N_INPUTS];
float nextParams[N_INPUTS];
float thetas[6];
String inputString;
bool newParams = false;

float getLength(int index, float x0, float x1, float x2, float x3, float x4, float x5) {
  float x = params[0];
  float y = params[1];
  float ux = params[3];
  float uy = params[4];
  float uz = params[5];
  float THUNK0 = pow(ux,2);
  float THUNK1 = pow(uz,2);
  float THUNK2 = sqrt(THUNK0+THUNK1);
  float THUNK3 = 3.405049132851517*uz/THUNK2;
  float THUNK4 = ux*uy;
  float THUNK5 = THUNK2*sqrt(THUNK0+pow(uy,2)+THUNK1);
  float THUNK6 = 0.6141989928900345*THUNK4/THUNK5;
  float THUNK7 = cos(x0);
  float THUNK8 = -0.6141989928900345*THUNK0-0.6141989928900345*THUNK1/THUNK5;
  float THUNK9 = 3.405049132851517*ux/THUNK2;
  float THUNK10 = uy*uz;
  float THUNK11 = 0.6141989928900345*THUNK10/THUNK5;
  float THUNK12 = cos(x1);
  float THUNK13 = 1.*x;
  float THUNK14 = cos(x3);
  float THUNK15 = cos(x4);
  switch(index) {
    case 0: return sqrt(pow(-3.1934686764551174+THUNK3+THUNK6+x+0.5*THUNK7,2)+pow(1.84375+THUNK8+y+0.8660254037844386*THUNK7,2)+pow(-12.43+THUNK9-THUNK11+sin(x0),2));
    case 1: return sqrt(pow(-3.1934686764551174+THUNK3-THUNK6+x+0.5*THUNK12,2)+pow(1.84375+THUNK8-1.*y+0.8660254037844386*THUNK12,2)+pow(-12.43+THUNK9+THUNK11+sin(x1),2));
    case 2: return sqrt(pow(-3.6875+3.255958546628605*THUNK0+3.255958546628605*THUNK1/THUNK5+y,2)+pow(1.17061263560417*uz/THUNK2+3.255958546628605*THUNK4/THUNK5-THUNK13+cos(x2),2)+pow(-12.43-1.17061263560417*ux/THUNK2+3.255958546628605*THUNK10/THUNK5+sin(x2),2));
    case 3: return sqrt(pow(3.1934686764551174-2.2344364972473456*uz/THUNK2-2.6417595537385705*THUNK4/THUNK5+x+0.5*THUNK14,2)+pow(-1.84375+2.6417595537385705*THUNK0+2.6417595537385705*THUNK1/THUNK5+y+0.8660254037844386*THUNK14,2)+pow(-12.43-2.2344364972473456*ux/THUNK2+2.6417595537385705*THUNK10/THUNK5+sin(x3),2));
    case 4: return sqrt(pow(1.84375+-2.641759553738569*THUNK0-2.641759553738569*THUNK1/THUNK5+y-0.8660254037844386*THUNK15,2)+pow(3.1934686764551174-2.2344364972473474*uz/THUNK2+2.641759553738569*THUNK4/THUNK5+x+0.5*THUNK15,2)+pow(-12.43-2.2344364972473474*ux/THUNK2-2.641759553738569*THUNK10/THUNK5+sin(x4),2));
    case 5: return sqrt(pow(3.6875+-3.2559585466286043*THUNK0-3.2559585466286043*THUNK1/THUNK5+y,2)+pow(1.1706126356041722*uz/THUNK2-3.2559585466286043*THUNK4/THUNK5-THUNK13+cos(x5),2)+pow(-12.43-1.1706126356041722*ux/THUNK2-3.2559585466286043*THUNK10/THUNK5+sin(x5),2));
    default: return -1;
  }
}

float getLength_(int index, float x0, float x1, float x2, float x3, float x4, float x5) {
  float x = params[0];
  float y = params[1];
  float z = params[2];
  float a = params[3];
  float b = params[4];
  float g = params[5];
  float THUNK0 = 1.*x;
  float THUNK1 = cos(x0);
  float THUNK2 = 0.5*THUNK1;
  float THUNK3 = cos(a);
  float THUNK4 = cos(g);
  float THUNK5 = 3.405049132851517*THUNK4;
  float THUNK6 = sin(g);
  float THUNK7 = 0.6141989928900345*THUNK6;
  float THUNK8 = THUNK5+THUNK7;
  float THUNK9 = sin(a);
  float THUNK10 = sin(b);
  float THUNK11 = THUNK4*THUNK9*THUNK10;
  float THUNK12 = 3.405049132851517*THUNK11;
  float THUNK13 = THUNK10*THUNK6;
  float THUNK14 = THUNK9*THUNK13;
  float THUNK15 = 0.6141989928900345*THUNK14;
  float THUNK16 = cos(b);
  float THUNK17 = 0.6141989928900345*THUNK4+3.405049132851517*THUNK6;
  float THUNK18 = THUNK4*THUNK10;
  float THUNK19 = 0.6141989928900345*THUNK18;
  float THUNK20 = THUNK5-THUNK7;
  float THUNK21 = 3.405049132851517*THUNK13;
  float THUNK22 = cos(x1);
  float THUNK23 = 1.17061263560417*THUNK4;
  float THUNK24 = 3.255958546628605*THUNK6;
  float THUNK25 = cos(x3);
  float THUNK26 = 2.2344364972473456*THUNK4;
  float THUNK27 = 2.6417595537385705*THUNK6;
  float THUNK28 = cos(x4);
  float THUNK29 = 2.2344364972473474*THUNK4;
  float THUNK30 = 2.641759553738569*THUNK6;
  float THUNK31 = 1.1706126356041722*THUNK4;
  float THUNK32 = 3.2559585466286043*THUNK6;
  switch(index) {
    case 0: return sqrt(1.*pow(-3.1934686764551174+THUNK0+THUNK2+THUNK3*THUNK8,2)+pow(1.84375+y+0.8660254037844386*THUNK1+THUNK12+THUNK15+THUNK16*-THUNK17,2)+pow(z-THUNK19+THUNK16*THUNK9*-THUNK20+THUNK21-1.*sin(x0),2));
    case 1: return sqrt(1.*pow(-3.1934686764551174+THUNK0+0.5*THUNK22+THUNK3*THUNK20,2)+pow(-1.84375+y-0.8660254037844386*THUNK22+THUNK12-THUNK15+THUNK16*THUNK17,2)+pow(z+THUNK19+THUNK16*THUNK9*-THUNK8+THUNK21-1.*sin(x1),2));
    case 2: return sqrt(pow(x-1.*cos(x2)+THUNK3*-THUNK23-THUNK24,2)+pow(-3.6875+y-1.17061263560417*THUNK11+THUNK16*3.255958546628605*THUNK4-1.17061263560417*THUNK6-3.255958546628605*THUNK14,2)+pow(z+3.255958546628605*THUNK18-1.17061263560417*THUNK13+THUNK16*THUNK9*THUNK23+THUNK24-1.*sin(x2),2));
    case 3: return sqrt(1.*pow(3.1934686764551174+THUNK0+0.5*THUNK25+THUNK3*-THUNK26-THUNK27,2)+pow(-1.84375+y+0.8660254037844386*THUNK25-2.2344364972473456*THUNK11+THUNK16*2.6417595537385705*THUNK4-2.2344364972473456*THUNK6-2.6417595537385705*THUNK14,2)+pow(z+2.6417595537385705*THUNK18-2.2344364972473456*THUNK13+THUNK16*THUNK9*THUNK26+THUNK27-1.*sin(x3),2));
    case 4: return sqrt(pow(1.84375+y-0.8660254037844386*THUNK28-2.2344364972473474*THUNK11+THUNK16*-2.641759553738569*THUNK4-2.2344364972473474*THUNK6+2.641759553738569*THUNK14,2)+1.*pow(3.1934686764551174+THUNK0+0.5*THUNK28+THUNK3*-THUNK29+THUNK30,2)+pow(z-2.641759553738569*THUNK18+THUNK16*THUNK9*THUNK29-THUNK30-2.2344364972473474*THUNK13-1.*sin(x4),2));
    case 5: return sqrt(pow(3.6875+y-1.1706126356041722*THUNK11+THUNK16*-3.2559585466286043*THUNK4-1.1706126356041722*THUNK6+3.2559585466286043*THUNK14,2)+pow(x-1.*cos(x5)+THUNK3*-THUNK31+THUNK32,2)+pow(z-3.2559585466286043*THUNK18+THUNK16*THUNK9*THUNK31-THUNK32-1.1706126356041722*THUNK13-1.*sin(x5),2));
  }
}

boolean angleSeach(int index, float tLow, float bLow, float tHigh, float bHigh, int iterations) {
  if(iterations >= MAX_ITERATIONS) {
    return false;
  }
  float tMid = (tLow+tHigh)/2;
  thetas[index] = tMid;
  float bMid = getLength(index, thetas[0], thetas[1], thetas[2], thetas[3], thetas[4], thetas[5]);
  if(bLow < H && bHigh > H) {
    if(abs(bMid-H)<EPSILON) {
      return true;
    }
    if(bMid > H) {
      return angleSeach(index, tLow, bLow, tMid, bMid, iterations+1);
    } else {
      return angleSeach(index, tMid, bMid, tHigh, bHigh, iterations+1);
    }
  } else {
     if(abs(bMid-H)<EPSILON) {
      return true;
    }
    if(bMid < H) {
      return angleSeach(index, tLow, bLow, tMid, bMid, iterations+1);
    } else {
      return angleSeach(index, tMid, bMid, tHigh, bHigh, iterations+1);
    }
  }
}

boolean calculateAngle(int index) {
  float tLow = -PI/2;
  float tHigh = PI/2;
  float tMid  = 0;
  thetas[index] = tLow;
  float bLow = getLength(index, thetas[0], thetas[1], thetas[2], thetas[3], thetas[4], thetas[5]);
  thetas[index] = tHigh;
  float bHigh = getLength(index, thetas[0], thetas[1], thetas[2], thetas[3], thetas[4], thetas[5]);
  if(bLow < H && bHigh < H || bLow > H && bHigh > H) {
    return false;
  }
  if(bLow < H && bHigh > H) {
    thetas[index] = tMid;
    float bMid = getLength(index, thetas[0], thetas[1], thetas[2], thetas[3], thetas[4], thetas[5]);
    if(abs(bMid-H)<EPSILON) {
      return true;
    }
    if(bMid > H) {
      return angleSeach(index, tLow, bLow, tMid, bMid, 1);
    } else {
      return angleSeach(index, tMid, bMid, tHigh, bHigh, 1);
    }
  } else {
    thetas[index] = tMid;
    float bMid = getLength(index, thetas[0], thetas[1], thetas[2], thetas[3], thetas[4], thetas[5]);
     if(abs(bMid-H)<EPSILON) {
      return true;
    }
    if(bMid < H) {
      return angleSeach(index, tLow, bLow, tMid, bMid, 1);
    } else {
      return angleSeach(index, tMid, bMid, tHigh, bHigh, 1);
    }
  }
}

boolean calculateAngles() {
  boolean success = true;
  for(int index = 0; index<6; ++index) {
    bool val = calculateAngle(index);
    success = success && val;
    if(success == false) {
      break;
    }
  }
  return success;
}

void moveToAngle(int servo, float angle) {
  if(servo%2==0) {
    servos[servo].write((angle+PI/2)*360/(2*PI));
  } else {
    servos[servo].write((-angle+PI/2)*360/(2*PI));
  }
}

void moveToAngles() {
  for(int i = 0; i<6; ++i) {
    if(thetas[i] > PI/2) {
      thetas[i] = PI/2;
    }
    if(thetas[i] < -PI/2) {
      thetas[i] = -PI/2;
    }
    moveToAngle(i,thetas[i]);
  }
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
  Serial.begin(9600);
  inputString.reserve(200);
  params[0] = 0;
  params[1] = 0;
  params[2] = 12.43;
  params[3] = 0;
  params[4] = 0;
  params[5] = 0;
  newParams = true;
}

void loop() {
  /*float w = .002;
  long t = millis()-startTime;
  float r = 1.5;
  float phi = 10.0/360.0*2*PI;
  params[0] = -r*cos(t*w);
  params[1] = -r*sin(t*w);
  params[2] = sin(phi)*cos(t*w);
  params[3] = sin(phi)*sin(t*w);
  params[4] = cos(phi);*/
  if(newParams) {
      #if READ_THETAS
        for(int i = 0; i<6; ++i) {
          thetas[i] = params[i];
          moveToAngles();
        }
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

boolean inTransaction;
void serialEvent() {
  if(newParams) {
      while(Serial.available()) Serial.read();
      return;
  }
  while (Serial.available()) {
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
      values[0] = vals.substring(0,delimIndicies[0]).toFloat();
      for(int i = 1; i<N_DELIMS; ++i) {
        delimIndicies[i] = vals.indexOf(',',delimIndicies[i-1]+1);
        values[i] = vals.substring(delimIndicies[i-1]+1,delimIndicies[i]).toFloat();
      }
      values[N_INPUTS-1] = vals.substring(delimIndicies[N_DELIMS-1]+1).toFloat();
      for(int i = 0; i<N_INPUTS; ++i) {
        params[i] = values[i];
      }
      newParams = true;
      Serial.println(inputString+"");
      kickout:
      inTransaction = false;
      inputString = "";
    } else {
      inputString = inputString + inChar;
    }
  }
}


