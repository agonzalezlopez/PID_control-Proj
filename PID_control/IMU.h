#ifndef IMU_H
#define IMU_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

class IMU{
  public:
    IMU();  
    bool begin();
    void update();
    float getRoll();
    float getPitch();

    private:
      Adafruit_MPU6050 mpu;
      sensors_event_t accel, gy, temp;
      const float alpha = 0.98;
      float filterRoll, filterPitch;
      unsigned long prevTime;
};

#endif