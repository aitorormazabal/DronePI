#include "Servo.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>


Servo::Servo(int PWM_RES){
	this->PWM_RES=PWM_RES;
}

void Servo::writeMicroseconds(long m)
{
	//gpioServo(pin,m);
	m=m/PWM_RES;
	int fifo;
	fifo=open("/dev/servoblaster",O_WRONLY);
	if (fifo<0){
		fprintf(stderr,"Error opening PWM fifo\n");
		return;
	}
	char command[15];
	command[0]='0'+pin;//Convert pin number to char
	command[1]='=';
	char buf[10];
	int length=sprintf(buf,"%ld",m);
	for (int i=0;i<length;i++)
	{
		command[i+2]=buf[i];
	}
	command[length+2]='\n';
	command[length+3]='\0';
	int res=write(fifo,command,length+3);
	if (res<0)
		fprintf(stderr,"Error writing to PWM fifo\n");
	close(fifo);
}
void Servo::attach(int p)
{
	pin=p;
}
