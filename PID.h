
class PID{
private:
	float delta;
	float error;
	float lastReading;
	float outputBound;
	float pastDerivatives[4];
public:
	float P,I,D;
	float errorBound;
	float target;
	float reading;
	float output;
	PID(float P,float I,float D, float errorBound,float outputBound);
	float Run(float delta);//returns output
	void Reset();
};