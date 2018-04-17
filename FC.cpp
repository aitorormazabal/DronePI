#include "FC.h"
#include "Input.h"
#include "Output.h"
#include "IMU.h"
#include "Servo.h"
#include <unistd.h>
#include "Helper.h"
#include <sys/time.h>
#include <time.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
FC& FC::instance(){
	static FC instance;
	return instance;
}
int FC::state(){
	return _state;
}
void FC::LoadSettings(std::string fileName){
	std::ifstream file(fileName.c_str());
	if (file.is_open())
	{
		file>>PWM_RES;
		file>>PWM_MIN;
		file>>PWM_MAX;
		file>>MS_TO_UNARM;
		for (int i=0;i<3;i++)
		{
			float P,I,D;
			file>>P>>I>>D;
			setRatePIDs(P,I,D,i);
			file>>P>>I>>D;
			setAnglePIDs(P,I,D,i);
		}
		file.close();
	}
}
void FC::Init(){
	for (int i=0;i<3;i++)
	{
		ratePIDS[i]= new PID(0,0,0,500,150);
		anglePIDS[i]= new PID(0,0,0,500,150);
	}
	msWithoutInput=0;
	Input::instance().Init();
	printf("Input Initialized\n");
	IMU::instance().Init();
	printf("IMU Initialized\n");
	usleep(20000);
	LoadSettings("Settings.txt");
	Output::instance().Init(0,2,1,3, PWM_RES);
	printf("Output Initialized\n");
	Unarm();
	lastTimePID=-1;
}

void FC::setRatePIDs(float P, float I, float D, int axis){
	printf("Setting rate PIDs: %f %f %f %d\n", P,I,D,axis);
	ratePIDS[axis]->P=P;
	ratePIDS[axis]->I=I;
	ratePIDS[axis]->D=D;
}
void FC::setAnglePIDs(float P, float I, float D, int axis){
	printf("Setting angle PIDs: %f %f %f %d\n", P,I,D,axis);
	anglePIDS[axis]->P=P;
	anglePIDS[axis]->I=I;
	anglePIDS[axis]->D=D;
}
void FC::setAltitudePID(float P, float I, float D){
	printf("Setting Altitude PIDs: %f %f %f \n", P,I,D);
}
void FC::ChangeState(int state)
{
	for (int i=0;i<3;i++) //Reset PID errors on state change
	{
		ratePIDS[i]->Reset();
		anglePIDS[i]->Reset();
	}
	if (state==STATE_ARMED_RATE){
		printf("Entering Rate mode\n");
		_state=STATE_ARMED_RATE;
	}
	else if (state==STATE_ARMED_ANGLE){
		printf("Entering Angle mode\n");
		_state=STATE_ARMED_ANGLE;
		IMU::instance().SetLevel();
	}
	else if (state==STATE_UNARMED){
		printf("Unarming\n");
		Unarm();
	}
}
void FC::ResetInputTimer(){
	msWithoutInput=0;
}
void FC::Update(){
	startTime=millis();
	long time1,time2,time3;
	Input::instance().Update();
	IMU& imu=IMU::instance();
	imu.Update();
	imu.PrintState();
	RunPIDs();
	WriteOutput();

	msWithoutInput+=LOOP_TIME_MS;
	if ( msWithoutInput> MS_TO_UNARM)
	{
		printf("Too long without input, unarming\n");
		Unarm();
		msWithoutInput=0;
	}

	bool waited=false;
	long elapsed=millis()-startTime;
	if (elapsed<LOOP_TIME_MS)
	{
		waited=true;
		struct timespec time,rem;
		time.tv_sec=0;
		time.tv_nsec=(LOOP_TIME_MS-elapsed-1)*1000000;
		int res=nanosleep(&time,&rem);
	}
	if (!waited)
	{
		printf("Loop lagging!");
		printf("%ld\n", elapsed);
	}
}
void FC::Unarm(){
	Output::instance().Unarm();
	_state=STATE_UNARMED;
	for (int i=0;i<3;i++)
	{
		ratePIDS[i]->Reset();
		anglePIDS[i]->Reset();
	}
}
void FC::RunPIDs(){
	long deltaus=us()-lastTimePID;
	deltaus=deltaus%1000000;
	float delta=((float) deltaus)/1000000.0f;
	if (lastTimePID==-1)
		delta=0.01f;
	if (_state==STATE_ARMED_RATE)
	{
		for (int i=0;i<3;i++)
		{
			ratePIDS[i]->target=Input::instance().targetRate[i];
			ratePIDS[i]->reading=IMU::instance().rate(i);
			PIDResults[i]=ratePIDS[i]->Run(delta);
		}
	}
	if (_state==STATE_ARMED_ANGLE)
	{
		for (int i=0;i<3;i++)
		{
			anglePIDS[i]->target=Input::instance().targetAngle[i];
			anglePIDS[i]->reading=IMU::instance().angle(i);
			ratePIDS[i]->target=anglePIDS[i]->Run(delta);
			ratePIDS[i]->reading=IMU::instance().rate(i);
			PIDResults[i]=ratePIDS[i]->Run(delta);
		}
	}
	lastTimePID=us();
}
void FC::WriteOutput(){
	if (_state==STATE_UNARMED)
		return;
	int FR,FL,BR,BL;
	int adjThrottle=PWM_MIN+Input::instance().throttle*(PWM_MAX-PWM_MIN)/100;

	FR = adjThrottle -PIDResults[0] - PIDResults[1]  + PIDResults[2];
	FL= adjThrottle + PIDResults[0] - PIDResults[1]  - PIDResults[2];
	BR = adjThrottle - PIDResults[0] + PIDResults[1] - PIDResults[2];
	BL = adjThrottle + PIDResults[0] + PIDResults[1] + PIDResults[2];
	int motorFixUpper=0;
	int motorFixLower=0;
	int outputs[4];
	outputs[0]=FR;
	outputs[1]=FL;
	outputs[2]=BR;
	outputs[3]=BL;
	for (int i=0;i<4;i++)
	{
		if (outputs[i]-PWM_MAX>motorFixUpper)
		motorFixUpper=outputs[i]-PWM_MAX;
		if (PWM_MIN-outputs[i]>motorFixLower)
			motorFixLower=PWM_MIN-outputs[i];
	}
	for (int i=0;i<4;i++)
	{
		outputs[i]=outputs[i]-motorFixUpper+motorFixLower;
	}
	Output::instance().WriteMotors(outputs[0],outputs[1],outputs[2],outputs[3]);
	printf("Output: %d %d %d %d\n", outputs[0], outputs[1], outputs[2], outputs[3]);
}