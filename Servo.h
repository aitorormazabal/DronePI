#ifndef SERVO_H
#define SERVO_H
class Servo{
private:
	int pin;
	int PWM_RES;
public:
	void writeMicroseconds(long m);
	void attach(int pin);
	Servo(int PWM_RES);
};
#endif