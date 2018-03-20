#ifndef OUTPUT_H
#define OUTPUT_H
#include "Servo.h"
class Output{
private:
	Output();
	Servo *m_FR,*m_FL,*m_BR,*m_BL;
public:
	static Output& instance();
	void Init(int FR, int FL, int BR, int BL,int PWM_RES);
	void Unarm();
	void WriteMotors(int FR,int FL, int BR, int BL);
};
#endif
