#include "NT_rescue.h"
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050_light.h>
#include <HUSKYLENS.h>
#include "SoftwareSerial.h"
#include "HardwareSerial.h"

int xOrigin;
int yOrigin;
int xTarget;
int yTarget;

HUSKYLENS huskylens;
MPU6050 gyro(Wire);

LiquidCrystal_I2C lcd(0x27, 16, 2);
void printResult(HUSKYLENSResult result);

//Settings.|
//         |
NT_rescue::NT_rescue(int _m1a, int _m1b, int _m1e, int _m2a, int _m2b, int _m2e, int _m3a, int _m3b, int _m3e, int _m4a, int _m4b, int _m4e) {
  m1a = _m1a;
  m1b = _m1b;
  m1e = _m1e;
  m2a = _m2a;
  m2b = _m2b;
  m2e = _m2e;
  m3a = _m3a;
  m3b = _m3b;
  m3e = _m3e;
  m4a = _m4a;
  m4b = _m4b;
  m4e = _m4e;
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  pinMode(m1a, OUTPUT);
  pinMode(m1b, OUTPUT);
  pinMode(m1e, OUTPUT);
  pinMode(m2a, OUTPUT);
  pinMode(m2b, OUTPUT);
  pinMode(m2e, OUTPUT);
  pinMode(m3a, OUTPUT);
  pinMode(m3b, OUTPUT);
  pinMode(m3e, OUTPUT);
  pinMode(m4a, OUTPUT);
  pinMode(m4b, OUTPUT);
  pinMode(m4e, OUTPUT);
}

void NT_rescue::begin() {
  setLCD();
  Serial.begin(115200);
  calibrateMPU();
  beginHusky();
  Serial.println("Started!");
}

void NT_rescue::calibrateMPU() {
  // All kinds of things that need to be done.
  Serial.begin(115200);
  Wire.begin();
  // gyro.setGyroOffsets(-54, -36, 26);
  // gyro.setAccOffsets(-598,  -789, 2126);


  byte status = gyro.begin();
  Serial.print("MPU6050 status: ");
  Serial.println(status);
  while (status != 0) {}  // Stop everything if could not connect to MPU6050.

  Serial.println("Calculating offsets, do not move MPU6050");
  delay(1000);
  // mpu.upsideDownMounting = true; // Uncomment this line if the MPU6050 is mounted upside-down.
  gyro.calcOffsets();  // Gyro and accelerometer.
  Serial.println("Done!\n");
}

void NT_rescue::beginHusky() {
  Wire.begin();
  if (!huskylens.begin(Wire)) {
    Serial.println(F("Husky Begin failed!"));
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }
}

// int NT_rescue::FindLine(){
//   int deltax = xOrigin - xTarget; //delta x
//   Serial.print("delta X = ");
//   Serial.println(deltax);
//   // PIDcalc(tempdirection, 0);//direction 0 is ahead
//   .steer(calc(deltax, 0));//
//   if(output > 50 || output < -50){ //if a sharp turn, reduce speed
//     mycar.move(150); //set this value according to behaviour
//    }
//    else {mycar.move(_speed);} //run at the default speed
//   return(output); //the correction value  for steer()
// }

void printResult(HUSKYLENSResult result) {
  if (result.command == COMMAND_RETURN_BLOCK) {
    Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
  } else if (result.command == COMMAND_RETURN_ARROW) {
    xOrigin = result.xOrigin;
    xTarget = result.xTarget;
    yOrigin = result.yOrigin;
    yTarget = result.yTarget;
    //Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
  } else {
    Serial.println("Object unknown!");
  }
}
int NT_rescue::getYaw() {
  gyro.update();
  int yaw = -gyro.getAngleZ();

  while (yaw > 360) {
    yaw -= 360;
  }

  while (yaw < -360) {
    yaw += 360;
  }

  return yaw;
}


//         ^
//Settings.|


//Gyro. |
//      |


void NT_rescue::movein(int degree, int speed) {

  gyro.update();
  while (true) {
    moveinOut = calc(degree, getYaw(), 1.0, 0.01, 0.2);
    moveFSR = moveinOut + speed;
    moveFSL = moveinOut - (moveFOut);

    Tf(moveFSR);
    THf(moveFSL);
    Of(moveFSL);
    Ff(moveFSR);
    LcdYaw();
  }
}



void NT_rescue::setLCD() {
  lcd.begin();
  lcd.backlight();
}
void NT_rescue::printYaw() {
  Serial.print("Yaw: ");
  Serial.println(getYaw());
  lcd.print(getYaw());
}


void NT_rescue::LcdYaw() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Yaw: ");
  lcd.println(getYaw());
}


void NT_rescue::resetYaw() {
  calibrateMPU();
}


//Gyro. ^
//      |


double NT_rescue::calc(double sp, double pv, float kp, float ki, float kd) {
  Time = millis() / 1000.0;

  deltaTime = Time - prev_Time;
  if (deltaTime == 0) {
    deltaTime = 0.01;
  }

  error = sp - pv;
  P = kp * error;

  integral += error * deltaTime;
  I = ki * integral;

  derivative = (error - prev_Error) / deltaTime;
  D = kd * derivative;


  output = P + I + D;

  prev_Error = error;

  prev_Time = Time;
  Serial.println(derivative);
  return output;
}




void NT_rescue::check_Ku(int speed, int degree, int Ku) {
  prev_Error = 0;
  int count = 0;
  while (getYaw() != degree) {
    LcdYaw();
    turnOut = calc(degree, getYaw(), Ku, 0, 0);

    turnSpeed = speed + abs(turnOut);
    if (getYaw() == degree) {
      break;
    }
    if (turnOut > 0) {
      turnR(turnSpeed);
      LcdYaw();


    } else {
      turnL(turnSpeed);
      LcdYaw();
    }

    LcdYaw();
    printYaw();
    Serial.print("count: ");
    Serial.print(count);
    count++;
  }

  stopMotors();
}
void NT_rescue::FollowLine(int speed, int, int color, float kp, float ki, float kd) {
}

void NT_rescue::moveF(int speed) {
  // Tf(speed);
  // THf(speed);
  // Of(speed);
  // Ff(speed);
  integral = 0;
  prev_Error = 0;

  kp = Kp;
  ki = Ki;
  kd = Kd;

  while (true) {
    moveFOut = calc(0, getYaw(), 1, 0.01, 0.1);
    moveFSR = moveFOut + speed;
    moveFSL = speed - (moveFOut);


    Tf(moveFSR);
    THf(moveFSL);
    Of(moveFSL);
    Ff(moveFSR);
    LcdYaw();
  }
}


void NT_rescue::moveB(int speed) {
  Tb(speed);
  THb(speed);
  Ob(speed);
  Fb(speed);
}

void NT_rescue::turnR(int speed) {
  Tb(speed);
  THf(speed);
  Of(speed);
  Fb(speed);
}

void NT_rescue::turnL(int speed) {
  Tf(speed);
  THb(speed);
  Ob(speed);
  Ff(speed);
}

void NT_rescue::move(int degree, int speed) {
  integral = 0;
  prev_Error = 0;
  kp = Kp;
  ki = Ki;
  kd = Kd;

  moveOut = calc(degree, getYaw(), 1, 0.01, 0.1);
  moveSpeed = abs(moveOut) + speed;
}


void NT_rescue::turn(int degree, int speed) {
  integral = 0;
  prev_Error = 0;
  kp = Kp;
  ki = Ki;
  kd = Kd;
  while (getYaw() != degree) {
    LcdYaw();
    turnOut = calc(degree, getYaw(), 1, 0.01, 0.1);

    turnSpeed = speed + abs(turnOut);
    if (getYaw() == degree) {
      break;
    }
    if (turnOut > 0) {
      turnR(turnSpeed);
      LcdYaw();


    } else {
      turnL(turnSpeed);
      LcdYaw();
    }

    LcdYaw();
    printYaw();
  }
  stopMotors();
}



void NT_rescue::L(int speed) {
  Tf(speed);
  THf(speed);
  Ob(speed);
  Fb(speed);
}

void NT_rescue::R(int speed) {
  Tb(speed);
  THb(speed);
  Of(speed);
  Ff(speed);
}
void NT_rescue::stopMotors() {
  Of(0);
  Tf(0);
  THf(0);
  Ff(0);
}
void NT_rescue::DfL(int speed) {
  Of(speed);
  Ff(speed);
  Tb(speed);
  THb(speed);
}

void NT_rescue::DfR(int speed) {
  Ob(speed);
  Fb(speed);
  Tf(speed);
  THf(speed);
}
void NT_rescue::DbL(int speed) {
  Ob(speed);
  Fb(speed);
  Tb(speed);
  THb(speed);
}

void NT_rescue::DbR(int speed) {
  Of(speed);
  Ff(speed);
  Tf(speed);
  THf(speed);
}

//Basic wheels functions.
void NT_rescue::Of(int speed) {
  digitalWrite(m1a, HIGH);
  digitalWrite(m1b, LOW);
  analogWrite(m1e, speed);
}

void NT_rescue::Ob(int speed) {
  digitalWrite(m1b, HIGH);
  digitalWrite(m1a, LOW);
  analogWrite(m1e, speed);
}

void NT_rescue::Tf(int speed) {
  digitalWrite(m2b, HIGH);
  digitalWrite(m2a, LOW);
  analogWrite(m2e, speed);
}

void NT_rescue::Tb(int speed) {
  digitalWrite(m2b, LOW);
  digitalWrite(m2a, HIGH);
  analogWrite(m2e, speed);
}

void NT_rescue::THf(int speed) {
  digitalWrite(m3a, HIGH);
  digitalWrite(m3b, LOW);
  analogWrite(m3e, speed);
}

void NT_rescue::THb(int speed) {
  digitalWrite(m3a, LOW);
  digitalWrite(m3b, HIGH);
  analogWrite(m3e, speed);
}

void NT_rescue::Ff(int speed) {
  digitalWrite(m4a, LOW);
  digitalWrite(m4b, HIGH);
  analogWrite(m4e, speed);
}

void NT_rescue::Fb(int speed) {
  digitalWrite(m4a, HIGH);
  digitalWrite(m4b, LOW);
  analogWrite(m4e, speed);
}
