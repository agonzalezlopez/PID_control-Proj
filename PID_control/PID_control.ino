#include <Wire.h>
#include "IMU.h"
#include "Motor.h"
#include <PID_v1.h>
#include <PinChangeInterrupt.h> //this is for interrupting in any pins rather than designated ones


//USES ADAFRUIT LIBRARY for MPU6050, PID Library (1.2.0), PinChangeInterrupt (1.2.9) 


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

const int encoder2A = 13;
const int encoder2B = 12;

// Creating objects
IMU imu;
Motor JGA370(AIN1,AIN2, encoder1A,encoder1B,PWMA);
Motor JGA371(BIN1,BIN2, encoder2A,encoder2B,PWMB);

double Setpoint0, Input0, Output0;

double const kp0 = 12,ki0 = 0.0,kd0 = 0.0;
double const kp1 = 12,ki1 = 0.0,kd1 = 0.0;

PID PID0(&Input0, &Output0, &Setpoint0,kp0,ki0,kd0,DIRECT);// PID FOR 370 MOTOR
PID PID1(&Input0, &Output0, &Setpoint0,kp1,ki1,kd1,DIRECT);// PID FOR 371 MOTOR

// for the JGA370
void encoderSetupJGA0(){
  JGA370.setEncoderReading();
}
//add same encoder setup for 371
void encoderSetupJGA1(){
  JGA371.setEncoderReading();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial) delay(10);

  if(!imu.begin()){
    Serial.println("Error");
    while(1){ delay(10); }
  }  

  JGA370.begin();
  JGA371.begin();

  attachInterrupt(digitalPinToInterrupt(encoder1B), encoderSetupJGA0, CHANGE);
  attachPCINT(digitalPinToPCINT(encoder2A), encoderSetupJGA1, CHANGE);

  Input0 = imu.getRoll();
  Setpoint0 = 85; // degrees

  PID0.SetMode(AUTOMATIC);
  PID0.SetOutputLimits(-255, 255);

  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-255, 255);
}

void loop() {

  Input0 = imu.getRoll();
  PID0.Compute();
  
  Serial.println("Current Input: ");
  Serial.print(Input0);
  
  Serial.println("Current output: ");
  Serial.print(Output0);
  
  driveMotor(Output0, JGA370);
  driveMotor(Output0, JGA371);
  
  Serial.println("Current Speed: ");
  Serial.print(constrain(abs(Output0),0,255));
  delay(50);
}

void driveMotor(double output, Motor &type){
  int currDirection = (output >= 0)? 1 : -1;
  int currSpeed = constrain(abs(output),0, 255);

  int minimumSpeed = 55;

  if(currSpeed > 0 && currSpeed < minimumSpeed){
    currSpeed = minimumSpeed;
  }

  type.setDirection(currDirection);
  type.setSpeed(currSpeed);
}
