#include "Motor.h";
#include <Arduino.h>;
#include <Wire.h>;

volatile long encoderTick = 0;

Motor::Motor(int pin1, int pin2, int encoderPin1, int encoderPin2, int PWMPin){
  _pin1 = pin1;
  _pin2 = pin2;
  _encoderPin1 = encoderPin1;
  _encoderPin2 = encoderPin2;
  _PWMPin = PWMPin;
} 

void Motor::begin(){
  pinMode(_pin1, OUTPUT);
  pinMode(_pin2, OUTPUT);
  pinMode(_PWMPin, OUTPUT);

  pinMode(_encoderPin1, INPUT_PULLUP);
  pinMode(_encoderPin2, INPUT_PULLUP);
}

void Motor::setSpeed(double speed){
  analogWrite(_PWMPin, speed);
}

int Motor::getDirection(){
  int currentState = 0;
  int state = digitalRead(_encoderPin1);
  if(state == LOW){
    currentState = 1; // Forward
  }
  else{
    currentState = -1; // Reverse
  }
  return currentState;
}


//CAUTION: NEED TO INVERT FOR MIRRORING MOTORS
// 1 for forward, -1 for reverse
void Motor::setDirection(int direction){
  if(direction == 1){
    //forward
    digitalWrite(_pin1, HIGH);
    digitalWrite(_pin2, LOW);
  }
  else{
    //backward
    digitalWrite(_pin1, LOW);
    digitalWrite(_pin2, HIGH);
  }
}

void Motor::setEncoderReading(){
  bool encoderPinA = digitalRead(_encoderPin1);
  bool encoderPinB = digitalRead(_encoderPin2);

  if(encoderPinA == encoderPinB){
    encoderTick++; // forward    
  }
  else{
    encoderTick--; // reverse
  }
}

long Motor::getEncoderReading(){
  return encoderTick;
}

void Motor::stop(){
  setSpeed(0);
}