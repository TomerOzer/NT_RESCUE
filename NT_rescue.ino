#include "NT_rescue.h"
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050_light.h>

int M1A = 7;
int M1B = 6;
int M1E = 5;
int M2A = 12;
int M2B = 13;
int M2E = 11;
int M3A = 2;
int M3B = 3;
int M3E = 4;
int M4A = 9;
int M4B = 8;
int M4E = 10;

NT_rescue robot(M1A, M1B, M1E, M2A, M2B, M2E, M3A, M3B, M3E, M4A, M4B, M4E);

void setup() {
  robot.begin();


}

void loop() {
}
