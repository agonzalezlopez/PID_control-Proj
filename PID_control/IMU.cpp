#include "IMU.h";


IMU::IMU(){}

bool IMU::begin(){
  filterRoll = 0;
  filterPitch = 0;
  prevTime = millis();

  if(!mpu.begin()){
    return false;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  return true;
}

void IMU::update(){
  mpu.getEvent(&accel, &gy, &temp);
}

float IMU::getRoll(){
  update();
  float roll = atan2(accel.acceleration.y, accel.acceleration.z) * (180/PI); // Actual Roll using acceleration

  unsigned long now = millis();
  float dt = (now - prevTime) / 1000; // Delta time
  prevTime = now;

  float gyroRollRate = gy.gyro.x * (180/PI);

  filterRoll = (alpha * (filterRoll + gyroRollRate * dt) + (1 - alpha) * roll);

  // negative to invert the values shown
  return -filterRoll;
}

float IMU::getPitch(){
  update();
  //Pitch using acceleration measurements
  float pitch = atan2(-accel.acceleration.x, sqrt(accel.acceleration.y*accel.acceleration.y + accel.acceleration.z*accel.acceleration.z)) * (180.0/PI); 

  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0; // Delta time
  prevTime = now;

  float gyroPitchRate = gy.gyro.y * (180/PI);
  filterPitch = (alpha * (filterPitch + gyroPitchRate * dt) + (1- alpha) * pitch);

  return filterPitch;
}