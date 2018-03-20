

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "Helper.h"
#include <unistd.h>
#include <math.h>
#include "I2Cdev.h"
#include "MS5611.h"
#include "altitude_kf.h"
#include  "MPU6050_6Axis_MotionApps20.h"
#include "IMU.h"
#include "FC.h"
#include <stdlib.h>

IMU& IMU::instance(){
    static IMU instance;
    return instance;
}
void IMU::Calibrate()
{
  double avgX=0;
  double avgY=0;
  double avgAcc=0;
  int numITS=2000;
  int fifoCount;
  uint8_t fifoBuffer[64];
  for (int i=0;i<numITS;i++)
  {
    long t=millis();
    mpu.getMotion6(&rawAcc[0], &rawAcc[1], &rawAcc[2], &rawGyro[0], &rawGyro[1], &rawGyro[2]);
   

    fifoCount = mpu.getFIFOCount();
   if (fifoCount>1000)
   {
    mpu.resetFIFO();
    printf("FIFO OVERFLOW\n");
   }
   if (fifoCount >= packetSize)
   {
      Quaternion q;
      VectorFloat gravity;  
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(calAngles, &q, &gravity);
      float tmp=calAngles[0];
      //switch from zyx to xyz order and from radian to degree
      calAngles[0]=calAngles[2]*180/M_PI;
      calAngles[2]=-tmp*180/M_PI;
      calAngles[1]=-calAngles[1]*180/M_PI;
   }
   if (i>numITS-200)
        avgAcc+=rawAcc[2];
    long elapsed=millis()-t;
    bool waited=false;
    if (elapsed<FC::LOOP_TIME_MS)
    {
      waited=true;
      struct timespec time,rem;
      time.tv_sec=0;
      time.tv_nsec=(FC::LOOP_TIME_MS-elapsed)*1000000;
      int res=nanosleep(&time,&rem);
    }
    if (!waited)
    {
      printf("Calibration lagging! ");
      printf("%ld\n", elapsed);
    }
  }
  avgAcc=avgAcc/199;
  accelScale=avgAcc/9.8f;
}


void IMU::Init(){
    printf("Initializing I2C devices...\n");
    mpu.initialize();

    // verify connection
    printf("Testing device connections...\n");
    printf(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed\n");
    connexion=mpu.testConnection();

    if (!connexion)
      ;

    
    printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();

    if (devStatus==0)
    {
      printf("Enabling DMP\n");
      mpu.setDMPEnabled(true);
      packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
      printf("DMP Initialization failed %d \n",devStatus);
    }
    gyroScale=16.4F;
    anglesDMP[0]=0;
    anglesDMP[2]=0;
    anglesDMP[1]=0;
    packetFlag=false;
}


void IMU::Update()
{
   int fifoCount;
   uint8_t fifoBuffer[64];
   mpu.getMotion6(&rawAcc[0], &rawAcc[1], &rawAcc[2], &rawGyro[0], &rawGyro[1], &rawGyro[2]);
   gyroData[0]=rawGyro[0]/gyroScale;
   gyroData[1]=rawGyro[1]/gyroScale;
   gyroData[2]=rawGyro[2]/gyroScale;

   fifoCount = mpu.getFIFOCount();
   if (fifoCount>=3*packetSize)
   {
    mpu.resetFIFO();
    printf("\n\n\n\n\n\n\n------------------------------------------------------------\n\n\n\n\n\n\nFIFO OVERFLOW\n\n\n\n\n\n\n-----------------------------------------------\n\n\n\n\n\n\n\n\n\n\n\n");
   }
   if (fifoCount >= packetSize)
   {
      Quaternion q;
      VectorFloat gravity;  
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      
      VectorFloat g2=gravity;
      g2.normalize();
      mpu.dmpGetYawPitchRoll(anglesDMP, &q, &gravity);
      float tmp=anglesDMP[0];
      //switch from zyx to xyz order and from radian to degree
      anglesDMP[0]=anglesDMP[2]*180/M_PI;
      anglesDMP[2]=-tmp*180/M_PI;
      anglesDMP[1]=-anglesDMP[1]*180/M_PI;
   }
   else{
      ;
   }
}
float IMU::rate(int axis)
{
  return gyroData[axis];
}
float IMU::angle(int axis)
{
  return anglesDMP[axis]-calAngles[axis];
}
void IMU::PrintState(){
  printf("X:%f Y:%f Z:%f GX:%f GY:%f GZ:%f\n",angle(0),angle(1),angle(2),rate(0),rate(1),rate(2));
}