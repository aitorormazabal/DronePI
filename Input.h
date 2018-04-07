#ifndef INPUT_H
#define INPUT_H
class Input{
private:
	Input(){}
	int commsFD;
	bool readSinceLastWrite();
	void setAnglePID();
	void setRatePID();
	void setAltitudePID();
public:
	static Input& instance();
	float targetRate[3];
	float targetAngle[3];
	float cameraPoseT[3];//Camera pose translation
	int throttle;

	void PrintState();
	void Init();
	void Update();
	void WriteInputs(int pitch, int roll, int yaw, int throttle);
};
#endif
