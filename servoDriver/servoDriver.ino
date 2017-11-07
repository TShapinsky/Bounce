#include <MatrixMath.h>
#include <math.h>
#include <Servo.h>
#define PINS {3,5,6,9,10,11}
#define MINS {630,610,540,570,600,600}
#define MAXS {2380,2400,2340,2360,2330,2330}
#define CROSS(a,b) {           \
  (a)[1]*(b)[2]-(a)[2]*(b)[1], \
  (a)[2]*(b)[0]-(a)[0]*(b)[2], \
  (a)[0]*(b)[1]-(a)[1]*(b)[0], \
  0                            \
  }
#define NORM(a)    (sqrt((a)[0]*(a)[0] + (a)[1]*(a)[1] + (a)[2]*(a)[2]))
#define INDEX(y,x,m,n) ((y)*(n)+(x))
#define H 12
#define EPSILON .1
#define EQ(a,b) (abs((a)-(b))<EPSILON)

const int pins[] = PINS;
const int mins[] = MINS;
const int maxs[] = MAXS;
Servo servos[6];

bool thetasValid = true;
float topPts[4*6];
float botPts[4*6];
float topTransformed[4*6];
float offsets[4*6];
float offsetsT[6*4];
float tMat[4*4];
float nMat[4*4];
float fullTransform[4*4];
float dists[6];
float thetas[6];

void zero(float *mat, int m, int n) {
  for(int y = 0; y<m; ++y) {
    for(int x = 0; x<n; ++x) {
      mat[INDEX(x,y,m,n)] = 0;
    }
  }
}

void normalize(float *v) {
  float norm = NORM(v);
  v[0] /= norm;
  v[1] /= norm;
  v[2] /= norm;
}

void translate(float *mat, float x, float y, float z) {
  mat[INDEX(0,0,4,4)] = 1;
  mat[INDEX(1,1,4,4)] = 1;
  mat[INDEX(2,2,4,4)] = 1;
  mat[INDEX(3,3,4,4)] = 1;
  mat[INDEX(0,3,4,4)] = x;
  mat[INDEX(1,3,4,4)] = y;
  mat[INDEX(2,3,4,4)] = z;
}

void buildMatTranspose(float *mat, float *i, float *j, float *k) {
  memcpy(mat+0, i, 4*sizeof(float));
  memcpy(mat+4, j, 4*sizeof(float));
  memcpy(mat+8, k, 4*sizeof(float));
  float l[] = {0,0,0,1};
  memcpy(mat+12,l, 4*sizeof(float));
}

void buildMat(float *mat, float *i, float *j, float *k) {
  float transpose[4*4];
  buildMatTranspose(transpose, i, j, k);
  Matrix.Transpose(transpose, 4, 4, mat);
}

void normal(float *mat, float *u) {
  const float l[] = {0,1,0,0};
  const float i[] = {1,0,0,0};
  const float j[] = {0,1,0,0};
  const float k[] = {0,0,1,0};
  float itick[] = CROSS(l,u);
  normalize(itick);
  float jtick[] = CROSS(u,itick);
  normalize(jtick);
  float *ktick = u;
  normalize(ktick);
  buildMat(mat, itick, jtick, ktick);
}

void normalS(float *mat, float ux, float uy, float uz) {
  float u[] = {ux, uy, uz, 0};
  normal(mat, u);
}

void buildModel(float *model, float r, float theta) {
  float phi = (2*PI-3*theta)/3;
  for(int i = 0; i<3; ++i) {
    model[INDEX(0,2*i,4,6)] = r * cos(i*(phi+theta) - theta/2);
    model[INDEX(1,2*i,4,6)] = r * sin(i*(phi+theta) - theta/2);
    model[INDEX(2,2*i,4,6)] = 0;
    model[INDEX(3,2*i,4,6)] = 1;
    model[INDEX(0,2*i+1,4,6)] = r * cos(i*(phi+theta) + theta/2);
    model[INDEX(1,2*i+1,4,6)] = r * sin(i*(phi+theta) + theta/2);
    model[INDEX(2,2*i+1,4,6)] = 0;
    model[INDEX(3,2*i+1,4,6)] = 1;
  }
}

void calculateTop(float x, float y, float ux, float uy, float uz) {
  translate(tMat, x, y, H);
  normalS(nMat, ux, uy, uz);
  Matrix.Multiply(tMat, nMat, 4, 4, 4, fullTransform);
  Matrix.Multiply(fullTransform, topPts, 4, 4, 6, topTransformed);
}

void servoFrame(int servo) {
  float point[] = {botPts[INDEX(0,servo,4,6)], botPts[INDEX(1,servo,4,6)], 0, 1};
  float jtick[] = {point[0], point[1], 0, 0};
  normalize(jtick);
  float ktick[] = {0,0,1,0};
  float itick[] = CROSS(jtick, ktick);
  translate(tMat, -point[0], -point[1], 0);
  buildMatTranspose(nMat, itick, jtick, ktick);
  Matrix.Multiply(nMat, tMat, 4, 4, 4, fullTransform);
}

void calculateDists(float x, float y, float ux, float uy, float uz) {
  calculateTop(x,y,ux,uy,uz);
  Matrix.Subtract(topTransformed, botPts, 4, 6, offsets);
  Matrix.Transpose(offsets, 4, 6, offsetsT);
  for(int i = 0; i < 6; ++i) {
    dists[i] = NORM(offsetsT + i*4);
  }
}

float calculateTheta(int servo) {
  servoFrame(servo);
  float topPoint[] = {
    topTransformed[INDEX(0,servo,4,6)], 
    topTransformed[INDEX(1,servo,4,6)],
    topTransformed[INDEX(2,servo,4,6)], 
    1
    };
  float topPointServoFrame[4];
  Matrix.Multiply(fullTransform, topPoint, 4, 4, 1, topPointServoFrame);
  float x = topPointServoFrame[0];
  float y = topPointServoFrame[1];
  float z = topPointServoFrame[2];
  float k0 = 12;
  float l = 1;
  float theta0, theta1, theta2, theta3;
  float d0, d1, d2, d3;
  float cos0, cos1;
  bool  cos0Valid=false, cos1Valid=false;
 
 
  cos0 = (x - pow(k0,2)*x + pow(x,3) + x*pow(y,2) + x*pow(z,2) - 
     sqrt(-pow(z,2) + 2*pow(k0,2)*pow(z,2) - pow(k0,4)*pow(z,2) + 2*pow(x,2)*pow(z,2) + 2*pow(k0,2)*pow(x,2)*pow(z,2) - 
       pow(x,4)*pow(z,2) - 2*pow(y,2)*pow(z,2) + 2*pow(k0,2)*pow(y,2)*pow(z,2) - 2*pow(x,2)*pow(y,2)*pow(z,2) - 
       pow(y,4)*pow(z,2) + 2*pow(z,4) + 2*pow(k0,2)*pow(z,4) - 2*pow(x,2)*pow(z,4) - 2*pow(y,2)*pow(z,4) - pow(z,6)))/
   (2.*(pow(x,2) + pow(z,2)));

  cos1 = (x - pow(k0,2)*x + pow(x,3) + x*pow(y,2) + x*pow(z,2) + 
     sqrt(-pow(z,2) + 2*pow(k0,2)*pow(z,2) - pow(k0,4)*pow(z,2) + 2*pow(x,2)*pow(z,2) + 2*pow(k0,2)*pow(x,2)*pow(z,2) - 
       pow(x,4)*pow(z,2) - 2*pow(y,2)*pow(z,2) + 2*pow(k0,2)*pow(y,2)*pow(z,2) - 2*pow(x,2)*pow(y,2)*pow(z,2) - 
       pow(y,4)*pow(z,2) + 2*pow(z,4) + 2*pow(k0,2)*pow(z,4) - 2*pow(x,2)*pow(z,4) - 2*pow(y,2)*pow(z,4) - pow(z,6)))/
   (2.*(pow(x,2) + pow(z,2)));
   
   if(cos0 >= -1 && cos0 <= 1) {
    theta0 = acos(cos0);
    d0 = sqrt(pow(y,2) + pow(x-cos(theta0),2) + pow(z-sin(theta0),2));
    theta1 = -acos(cos0);
    d1 = sqrt(pow(y,2) + pow(x-cos(theta1),2) + pow(z-sin(theta1),2));
    cos0Valid = true;
   }
   if(cos1 >= -1 && cos1 <= 1) {
    theta2 = acos(cos1);
    d2 = sqrt(pow(y,2) + pow(x-cos(theta2),2) + pow(z-sin(theta2),2));
    theta3 = -acos(cos1);
    d3 = sqrt(pow(y,3) + pow(x-cos(theta0),3) + pow(z-sin(theta0),3));
    cos1Valid = true;
   }
   if(cos0Valid) {
    if(theta0 <= PI/2 && theta0 >= -PI/2 && EQ(d0,k0)) {
      return theta0;
    } else if(theta1 <= PI/2 && theta1 >= -PI/2 && EQ(d1,k0)) {
      return theta1; 
    }
   }
   if(cos1Valid) {
    if(theta2 <= PI/2 && theta2 >= -PI/2 && EQ(d2,k0)) {
      return theta2;
    } else if(theta3 <= PI/2 && theta3 >= -PI/2 && EQ(d3,k0)) {
      return theta3; 
    }
   }
   digitalWrite(13, HIGH);
   thetasValid = false;
   return 0;
}

void calculateThetas() {
  digitalWrite(13,LOW);
  thetasValid = true;
  for(int i = 0; i<6; ++i) {
    thetas[i] = calculateTheta(i);
  }
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
    moveToAngle(i,thetas[i]);
  }
}

String inputString;
void setup() {
  float thetaB = 60 * 2 * PI / 360;
  float thetaT = 5 * 2 * PI / 360;
  float rb = 4;
  float rt = 3;
  for(int i = 0; i<6; ++i) {
    servos[i].attach(pins[i],mins[i],maxs[i]);
  }
  pinMode(13,OUTPUT);
  buildModel(botPts, rb, thetaB);
  buildModel(topPts, rt, thetaT);
  Serial.begin(9600);
  inputString.reserve(200);
}

bool newThetas = false;
void loop() {
  if(newThetas) {
    moveToAngles();
    newThetas = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    if(inChar == ' ') return;
    
    if(inChar == '}') {
      String vals = inputString.substring(1);
      int delimIndicies[] = {0,0,0,0,0};
      float values[6];
      delimIndicies[0] = vals.indexOf(',');
      if(delimIndicies[0] == -1) {
        goto kickout;
      }
      values[0] = vals.substring(0,delimIndicies[0]).toFloat();
      for(int i = 1; i<5; ++i) {
        delimIndicies[i] = vals.indexOf(',',delimIndicies[i-1]+1);
        values[i] = vals.substring(delimIndicies[i-1]+1,delimIndicies[i]).toFloat();
      }
      values[5] = vals.substring(delimIndicies[4]+1).toFloat();
      for(int i = 0; i<6; ++i) {
        thetas[i] = values[i];
        newThetas = true;
      }
      kickout:
      inputString = "";
    } else {
      inputString = inputString + inChar;
    }
  }
}


