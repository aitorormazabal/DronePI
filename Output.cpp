#include "Output.h"
#include "Input.h"
#include <stdio.h>
Output::Output(){}
Output& Output::instance(){
    static Output instance;
    return instance;
}
void Output::Init(int FR, int FL, int BR, int BL, int PWM_RES){
  printf("Output init\n");
  m_FR=new Servo(PWM_RES);
  m_FL=new Servo(PWM_RES);
  m_BR=new Servo(PWM_RES);
  m_BL=new Servo(PWM_RES);
  m_BL->attach(BL);
  m_BR->attach(BR);
  m_FL->attach(FL);
  m_FR->attach(FR);
  Unarm();
}
void Output::Unarm(){
  m_FR->writeMicroseconds(1000);
  m_FL->writeMicroseconds(1000);
  m_BR->writeMicroseconds(1000);
  m_BL->writeMicroseconds(1000);
}
void Output::WriteMotors(int FR,int FL, int BR, int BL)
{
  if (m_FR==0||m_FL==0||m_BR==0|m_BL==0){
    printf("Servos null!\n");
    return;
  }
  m_FR->writeMicroseconds(FR);
  m_FL->writeMicroseconds(FL);
  m_BR->writeMicroseconds(BR);
  m_BL->writeMicroseconds(BL);
}
