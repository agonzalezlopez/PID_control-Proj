#ifndef MOTOR_H
#define MOTOR_H

class Motor{
  public:
    Motor(int pin1, int pin2, int encoderPin1, int encoderPin2, int PWMPin, bool inverted = false);
    void begin();
    void setSpeed(double speed);
    int getDirection();
    void setDirection(int direction);
    void setEncoderReading();
    long getEncoderReading();
    void stop();

    private:
      int _pin1, _pin2, _encoderPin1, _encoderPin2, _PWMPin;
      bool _inverted;
};

#endif