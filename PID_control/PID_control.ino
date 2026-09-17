#include <Wire.h>
#include "IMU.h"
#include "Motor.h"
#include <PID_v1.h>
#include <PinChangeInterrupt.h> //this is for interrupting in any pins rather than designated ones


//USES ADAFRUIT LIBRARY for MPU6050, PID Library (1.2.0), PinChangeInterrupt (1.2.9) 
const int buttonID = 5;
const int buzzerID = 6;

//motor constants 370
const int PWMA = 10;
const int AIN1 = 9;
const int AIN2 = 8;

const int encoder1A = 4;
const int encoder1B = 3;

// FOR OTHER MOTOR 371
const int BIN1 = A0;
const int BIN2 = A1;
const int PWMB = 11;

const int encoder2A = 13;
const int encoder2B = 12;

// Creating objects
IMU imu;
Motor JGA370(AIN1,AIN2, encoder1A,encoder1B,PWMA,true);
Motor JGA371(BIN1,BIN2, encoder2A,encoder2B,PWMB);

double Setpoint0, Input0, Output0, Output1;

double const kp0 = 4,ki0 = 0.0,kd0 = 6;
double const kp1 = 4,ki1 = 0.0,kd1 = 6;

const float FAILSAFE_ANGLE = 25.0; 

PID PID0(&Input0, &Output0, &Setpoint0,kp0,ki0,kd0,DIRECT);// PID FOR 370 MOTOR
PID PID1(&Input0, &Output1, &Setpoint0,kp1,ki1,kd1,DIRECT);// PID FOR 371 MOTOR

//Button values
bool systemRunning = false;
int lastButtonReading = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // ms

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

  pinMode(buttonID, INPUT_PULLUP);
  pinMode(buzzerID, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoder1B), encoderSetupJGA0, CHANGE);
  attachPCINT(digitalPinToPCINT(encoder2A), encoderSetupJGA1, CHANGE);

  Input0 = imu.getPitch();
  Setpoint0 = 0; // degrees

  PID0.SetMode(AUTOMATIC);
  PID0.SetOutputLimits(-255, 255);

  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-255, 255);
}

void loop() {
  unsigned long loopStart = millis();


  checkButton();
  int buttonState = digitalRead(buttonID);

  Input0 = imu.getPitch();
  PID0.Compute();
  PID1.Compute();
  
  Serial.print("Input:");
  Serial.print(Input0);
  Serial.print(" LoopMs:");
  Serial.println(millis() - loopStart);
  
  // Serial.println("Current output: ");
  // Serial.print(Output0);
  
  // Serial.println("Current Speed: ");
  // Serial.print(constrain(abs(Output0),0,255));


      double error = Setpoint0 - Input0;

  if (abs(error) > FAILSAFE_ANGLE) {
    JGA370.stop();
    JGA371.stop();
  } else if (systemRunning) {
    driveMotor(Output0, JGA370);
    driveMotor(Output1, JGA371);
  } else {
    JGA370.stop();
    JGA371.stop();
  }

  }


void checkButton() {
  int reading = digitalRead(buttonID);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis(); // reset debounce timer on any change
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // reading has been stable for debounceDelay — treat as real
    static int stableState = HIGH;
    if (reading != stableState) {
      stableState = reading;
      if (stableState == LOW) { // button pressed (active-low with INPUT_PULLUP)
        systemRunning = !systemRunning; // toggle
        delay(5000); // 5 second delay to prepare
        tone(buzzerID, systemRunning ? 1000 : 500, 150); // quick beep
      }
    }
  }

  lastButtonReading = reading;
}





void driveMotor(double output, Motor &type){
  int currDirection = (output >= 0)? 1 : -1;
  int currSpeed = constrain(abs(output),0, 255);

  int minimumSpeed = 35;

  if(currSpeed > 0 && currSpeed < minimumSpeed){
    currSpeed = minimumSpeed;
  }

  type.setDirection(currDirection);
  type.setSpeed(currSpeed);
}
