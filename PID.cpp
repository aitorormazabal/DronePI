#include "PID.h"
#include <stdio.h>
#include "Helper.h"
PID::PID(float P,float I, float D, float errorBound, float outputBound, float delta)
{
	this->P=P;
	this->I=I;
	this->D=D;
	reading=0;
	lastReading=0;
	target=0;
    error=0;
	this->errorBound=errorBound;
	for (int i=0;i<5;i++)
	{
		pastDerivatives[i]=0;
	}
	this->delta=delta;
	this->outputBound=outputBound;
}
void PID::Reset(){
    error=0;
}
float PID::Run()
{
    float currError=target-reading;
    error+=(currError*delta);
    error=constrain(error,-errorBound,errorBound);
    float PTerm=P*currError;
    float currentD=-(reading-lastReading);
    float filteredD=currentD+pastDerivatives[0]+pastDerivatives[1]+pastDerivatives[2]+pastDerivatives[3];
    filteredD=filteredD/5;
    float DTerm=D*filteredD;
    float ITerm=I*error;
    float res=PTerm+DTerm+ITerm;
    res=constrain(res,-outputBound,outputBound);
    lastReading=reading;
    for (int i=0;i<3;i++)
    	pastDerivatives[i]=pastDerivatives[i+1];
    pastDerivatives[3]=currentD;
    output=res;
    return res;
}