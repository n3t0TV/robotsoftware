
struct MusclesState
{
    int Brain;
    //~ int Go;
    int Speed;
    int Angle;
    int Illumination;
    int Turning;
    int Coupling;
    int Power;
    int Reset;
    int Offset;
    int Brake;
    int Mode;
    int prevMode; 
    double TimeStamp;
    

    int speedMode;
    int lowSpeedLimit;
    int mediumSpeedLimit;
    int highSpeedLimit;
    
};

struct MusclesStateActon
{
    int Teleop;
    int Speed;
    int Angle;
    int Illumination;
    int Turning;
    int AlleyLights;
    int Coupling;
    int Reset;
    int Brake;
    double TimeStamp;
};

struct MusclesStateWagonIndia
{
    int Teleop;
    int Speed;
    int Angle;
    int Illumination;
    int Turning;
    int AlleyLights;
    int Coupling;
    int Reset;
    int Brake;
    int RightWheel;
    int LeftWheel;
    double TimeStamp;
    int DrivingMode;
    float AngularRate;
};

enum ControlMode
{
	CONTROL_GOOD_QUALITY,
	CONTROL_LOW_QUALITY,
	CONTROL_DISCONNECTED
};


struct ControlQuality
{
    int latencyVehicle;
    int latencyTeleop;
    int controlMode;
};
