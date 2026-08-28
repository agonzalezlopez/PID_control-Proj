#include <Wire.h>
#include "IMU.h"
#include "Motor.h"

// #include <PinChangeInterrupt.h> //this is for interrupting in any pins rather than designated ones


//USES ADAFRUIT LIBRARY for MPU6050


//motor constants
const int PWMA = 10;
const int AIN1 = 9;
const int AIN2 = 8;

const int encoder1A = 4;
const int encoder1B = 3;

// FOR OTHER MOTOR 
const int BIN1 = 7;
const int BIN2 = 6;
const int PWMB = 5;

volatile long encoderReading = 0;

// Creating objects
IMU imu;

Motor JGA370(AIN1,AIN2, encoder1A,encoder1B,PWMA);

// for the JGA370
void encoderSetupJGA0(){
  JGA370.setEncoderReading();
}
//add same encoder setup for 371

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial) delay(10);

  if(!imu.begin()){
    Serial.println("Error");
    while(1){ delay(10); }
  }  

  JGA370.begin();

  attachInterrupt(digitalPinToInterrupt(encoder1B), encoderSetupJGA0, CHANGE);
}

void loop() {

  JGA370.setSpeed(60);
  JGA370.setDirection(1);
  
  Serial.println(JGA370.getEncoderReading());
  delay(500);

  JGA370.setSpeed(0);
  JGA370.setDirection(1);
  delay(500);
}
