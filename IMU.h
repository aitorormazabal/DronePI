#ifndef IMU_H
#define IMU_H
#include <stdint.h>
#include <stdio.h>
#include "MPU6050.h"
class IMU{
private:
  IMU(){}
  bool connexion;
  bool readSinceLastWrite;
  int devStatus;
  int packetSize;
  float gyroScale;
  MPU6050 mpu;
  float calAngles[3];
  float anglesDMP[3];
  float anglesCF[3];//Complementary filter angles
  float gyroData[3];
  float eulerRates[3];
  int16_t rawGyro[3];
  float accelScale;
  int16_t rawAcc[3];
  bool packetFlag;
  void CalculateEulerRates();
public:
  static IMU& instance();
  float angle(int axis);
  float rate(int axis);
  void SetLevel();
  void Init();
  void Calibrate();
  void Update();
  void PrintState();

};

#endif
