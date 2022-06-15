#include <Arduino.h>
#include <Encoder.h>

#define RPWM1 5
#define LPWM1 6
#define RPWM2 9
#define LPWM2 10
#define EN1 7
#define EN2 8
#define ENCA1 2
#define ENCB1 4
#define ENCA2 3
#define ENCB2 12
#define PPR 375
#define PPS 819
#define MAX_RPS 2.17
#define MAX_RPM 166.40
#define MAX_PWM 255

Encoder myEnc1(ENCA1, ENCB1), myEnc2(ENCA2, ENCB2);

struct Variable
{
  int valEncoder = 0;
  float proportional;
  float integral;
  float derivative;
  float PID;
  float oldPosition = 0;
  float RPM, RPS;
  float oldData = 0;
  unsigned long currTime;
  unsigned long lastTime;
};
Variable Motor1, Motor2, Enc1, Enc2, Test, Motion;

enum Status{STOP, CW, CCW};
enum Driver{ENABLE_ALL, ENABLE_R, ENABLE_L, DISABLE};
enum ENC{M1, M2};

void DriverStatus(int mode){
  if(mode == ENABLE_ALL){
    digitalWrite(EN1, HIGH);
    digitalWrite(EN2, HIGH);
  }
  else if(mode == ENABLE_R){
    digitalWrite(EN1, HIGH);
    digitalWrite(EN2, LOW);
  }
  else if(mode == ENABLE_L){
    digitalWrite(EN1, LOW);
    digitalWrite(EN2, HIGH);
  }
  else if(mode == DISABLE){
    digitalWrite(EN1, LOW);
    digitalWrite(EN2, LOW);
  }
  else{
    Serial.println("WARNING! Please, argument is 1 or 0");
    delay(2000);
  }
}

void MotorStatus(int condition1, int condition2, float val1, float val2){
  val1 = map(val1, 0, MAX_RPM, 0, MAX_PWM); //mapping RPS to PWM
  val2 = map(val2, 0, MAX_RPM, 0, MAX_PWM);
  if(condition1 == CW && condition2 == CW){
    analogWrite(RPWM1, val1);
    delayMicroseconds(100);
    analogWrite(LPWM1, 0);

    analogWrite(RPWM2, val2);
    delayMicroseconds(100);
    analogWrite(LPWM2, 0);
  }
  else if(condition1 == CCW && condition2 == CCW){
    analogWrite(RPWM1, 0);
    delayMicroseconds(100);
    analogWrite(LPWM1, val1);

    analogWrite(RPWM2, 0);
    delayMicroseconds(100);
    analogWrite(LPWM2, val2);
  }
  else if(condition1 == CW && condition2 == CCW){
    analogWrite(RPWM1, val1);
    delayMicroseconds(100);
    analogWrite(LPWM1, 0);

    analogWrite(RPWM2, 0);
    delayMicroseconds(100);
    analogWrite(LPWM2, val2);
  }
  else if(condition1 == CCW && condition2 == CW){
    analogWrite(RPWM1, 0);
    delayMicroseconds(100);
    analogWrite(LPWM1, val1);

    analogWrite(RPWM2, val2);
    delayMicroseconds(100);
    analogWrite(LPWM2, 0);
  }
  else if(condition1 == CW && condition2 == STOP){
    analogWrite(RPWM1, val1);
    delayMicroseconds(100);
    analogWrite(LPWM1, 0);

    analogWrite(RPWM2, 0);
    delayMicroseconds(100);
    analogWrite(LPWM2, 0);
  }
  else if(condition1 == CCW && condition2 == STOP){
    analogWrite(RPWM1, 0);
    delayMicroseconds(100);
    analogWrite(LPWM1, val1);

    analogWrite(RPWM2, 0);
    delayMicroseconds(100);
    analogWrite(LPWM2, 0);
  }
  else if(condition1 == STOP && condition2 == CW){
    analogWrite(RPWM1, 0);
    delayMicroseconds(100);
    analogWrite(LPWM1, 0);

    analogWrite(RPWM2, val2);
    delayMicroseconds(100);
    analogWrite(LPWM2, 0);
  }
  else if(condition1 == STOP && condition2 == CCW){
    analogWrite(RPWM1, 0);
    delayMicroseconds(100);
    analogWrite(LPWM1, 0);

    analogWrite(RPWM2, 0);
    delayMicroseconds(100);
    analogWrite(LPWM2, val2);
  }
  else if(condition1 == STOP && condition2 == STOP){
    analogWrite(RPWM1, 0);
    delayMicroseconds(100);
    analogWrite(LPWM1, 0);

    analogWrite(RPWM2, 0);
    delayMicroseconds(100);
    analogWrite(LPWM2, 0);
  }
  else{
    Serial.println("WARNING! Please, argument is STOP, RIGHT, or LEFT");
    delay(2000);
  }
}

void MotorStatus(int condition1, int condition2){
  int value = 255;
    if(condition1 == CW && condition2 == CW){
    analogWrite(RPWM1, value);
    delayMicroseconds(100);
    analogWrite(LPWM1, 0);

    analogWrite(RPWM2, value);
    delayMicroseconds(100);
    analogWrite(LPWM2, 0);
  }
  else if(condition1 == CCW && condition2 == CCW){
    analogWrite(RPWM1, 0);
    delayMicroseconds(100);
    analogWrite(LPWM1, value);

    analogWrite(RPWM2, 0);
    delayMicroseconds(100);
    analogWrite(LPWM2, value);
  }
  else if(condition1 == CW && condition2 == CCW){
    analogWrite(RPWM1, value);
    delayMicroseconds(100);
    analogWrite(LPWM1, 0);

    analogWrite(RPWM2, 0);
    delayMicroseconds(100);
    analogWrite(LPWM2, value);
  }
  else if(condition1 == CCW && condition2 == CW){
    analogWrite(RPWM1, 0);
    delayMicroseconds(100);
    analogWrite(LPWM1, value);

    analogWrite(RPWM2, value);
    delayMicroseconds(100);
    analogWrite(LPWM2, 0);
  }
  else if(condition1 == CW && condition2 == STOP){
    analogWrite(RPWM1, value);
    delayMicroseconds(100);
    analogWrite(LPWM1, 0);

    analogWrite(RPWM2, 0);
    delayMicroseconds(100);
    analogWrite(LPWM2, 0);
  }
  else if(condition1 == CCW && condition2 == STOP){
    analogWrite(RPWM1, 0);
    delayMicroseconds(100);
    analogWrite(LPWM1, value);

    analogWrite(RPWM2, 0);
    delayMicroseconds(100);
    analogWrite(LPWM2, 0);
  }
  else if(condition1 == STOP && condition2 == CW){
    analogWrite(RPWM1, 0);
    delayMicroseconds(100);
    analogWrite(LPWM1, 0);

    analogWrite(RPWM2, value);
    delayMicroseconds(100);
    analogWrite(LPWM2, 0);
  }
  else if(condition1 == STOP && condition2 == CCW){
    analogWrite(RPWM1, 0);
    delayMicroseconds(100);
    analogWrite(LPWM1, 0);

    analogWrite(RPWM2, 0);
    delayMicroseconds(100);
    analogWrite(LPWM2, value);
  }
  else if(condition1 == STOP && condition2 == STOP){
    analogWrite(RPWM1, 0);
    delayMicroseconds(100);
    analogWrite(LPWM1, 0);

    analogWrite(RPWM2, 0);
    delayMicroseconds(100);
    analogWrite(LPWM2, 0);
  }
  else{
    Serial.println("WARNING! Please, argument is STOP, RIGHT, or LEFT");
    delay(2000);
  }
}

float toRPM1(){
  float newPosition = myEnc1.read();
  
  Enc1.currTime = millis();
  if((Enc1.currTime - Enc1.lastTime) >= 1000){
    newPosition -= Motor1.oldPosition;
    Motor1.RPS = (newPosition/PPR)/1;
    Motor1.RPS = fabs(Motor1.RPS);
    Motor1.RPM = Motor1.RPS*60;
    
    Motor1.oldPosition += newPosition;
    Enc1.lastTime = Enc1.currTime;
  }
  return Motor1.RPM;
}

float toRPM2(){
  float newPosition = myEnc2.read();
  
  Enc2.currTime = millis();
  if((Enc2.currTime - Enc2.lastTime) >= 1000){
    newPosition -= Motor2.oldPosition;
    Motor2.RPS = (newPosition/PPR)/1;
    Motor2.RPS = fabs(Motor2.RPS);
    Motor2.RPM = Motor2.RPS*60;
    
    Motor2.oldPosition += newPosition;
    Enc2.lastTime = Enc2.currTime;
  }
  return Motor2.RPM;
}

void ReadEncoder1(){
  float newData = myEnc1.read();
  if(newData != Motor1.oldData){
    Motor1.oldData = newData;
    Serial.println(newData/100);
  }
}

void ReadEncoder2(){
  float newData = myEnc2.read();
  if(newData != Motor2.oldData){
    Motor2.oldData = newData;
    Serial.println(newData/100);
  }
}

void ControlMotor(float Target1, float Target2){
  float Error1, Error2; 
  float lastError1, lastError2;
  static float sumError1, sumError2;

  Error1 = Target1 - toRPM1();
  Error2 = Target2 - toRPM2();

  Motor1.currTime = millis();
  if((Motor1.currTime - Motor1.lastTime) >= 100){
    Motor1.proportional = 0.95 * Error1;
    Motor1.integral = 1 * sumError1 * 0.1;
    Motor1.derivative = 0.00002 * (Error1 - lastError1) / 0.1;

    Motor2.proportional = 0.95 * Error2;
    Motor2.integral = 1 * sumError2 * 0.1;
    Motor2.derivative = 0.00002 * (Error2 - lastError2) / 0.1;
    
    Motor1.PID = Motor1.proportional + Motor1.integral + Motor1.derivative;
    Motor2.PID = Motor2.proportional + Motor2.integral + Motor2.derivative;

    sumError1 += Error1;
    sumError2 += Error2;

    if(sumError1 >= 4000) sumError1 = 4000;
    else if(sumError1 <= -4000) sumError1 = -4000;

    if(sumError2 >= 4000) sumError2 = 4000;
    else if(sumError2 <= -4000) sumError2 = -4000;

    lastError1 = Error1;
    lastError2 = Error2;

    // Serial.print(toRPM1());
    // Serial.print(" | ");
    Serial.println(toRPM2());

    Motor1.PID = constrain(Motor1.PID, 0, MAX_RPM);
    Motor2.PID = constrain(Motor2.PID, 0, MAX_RPM);

    MotorStatus(CCW, CCW, Motor1.PID, Motor2.PID);

    Motor1.lastTime = Motor1.currTime;
  }
}

void ControlMotion(float setpoint){
  Motion.lastTime = millis();
  if((Motion.currTime - Motion.lastTime) >= 100){

  }
}

void setup() {
  Serial.begin(9600);
  pinMode(RPWM1, OUTPUT);
  pinMode(LPWM1, OUTPUT);
  pinMode(RPWM2, OUTPUT);
  pinMode(LPWM2, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  DriverStatus(ENABLE_L);
}

void loop() {
  // ControlMotor(0,160);
  Test.currTime = millis();
  if((Test.currTime - Test.lastTime) <= 20000){
    MotorStatus(CCW,CCW);
    Serial.println(toRPM2());
  } 
  else DriverStatus(DISABLE);
}