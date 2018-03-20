
#include "PID.h"
#include <string>
class FC{
public:
	static FC& instance();
	int state();
	void Init();
	void Update();
	void ResetInputTimer();
	void Unarm();
	void setAnglePIDs(float P, float I, float D, int axis);
	void setRatePIDs(float P, float I, float D, int axis);
	void setAltitudePID(float P, float I, float D);
	static const int STATE_UNARMED=0;
	static const int STATE_ARMED_RATE=1;
	static const int STATE_ARMED_ANGLE=2;
	static constexpr float LOOP_TIME_S=0.01F;
	static const long LOOP_TIME_MS=10;
	int MS_TO_UNARM=500;
	int PWM_RES=5;
	int PWM_MIN=1200;
	int PWM_MAX=1900;
private:
	FC(){};
	void LoadSettings(std::string file);
	void RunPIDs();
	void WriteOutput();
	int _state;
	int msWithoutInput;
	PID *ratePIDS[3];
	PID *anglePIDS[3];
	int PIDResults[3];
	long startTime;
};