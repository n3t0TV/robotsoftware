using namespace std;
#include <vector>

struct PICStatus
{
    int desiredSpeed;
    int currentSpeed;
    int desiredSpeedRight;
    int currentSpeedRight;
    int desiredSpeedLeft;
    int currentSpeedLeft;
    float desiredAngularRate;
    float currentAngularRate;
    unsigned int batteryLevel;
    float batteryVoltage;
    int yaw;
    int roll;
    int pitch;
    string version;
    bool brake;
    int turningLights;
    int illumination;
    bool picOK;
    int errorVector;
    uint64_t udpTimeStamp;
    uint64_t qrTimeStamp;
    uint lastControlMsg;
};

struct heartbeat{
	string imei;
	double latitud;
	double longitud;
	double altitud;
	int rssi;
	int ber;
	double bandwidth;
	int batteryLevel;
	int latency;
	int status=0;
};
 
struct GPSData{
	double latitud;
	double longitud;
	double altitud;
	int rssi;
	int sim;
	bool validGPS;
    int satellites;
};
 
struct PerformanceData{
	double latency;
	double throughput;
	double latency_teleops;
};

struct ADC_currents
{
    int battery_current;
    int current_drive;
    int current_steer1;
    int current_steer2;
    int current_brake;
    int current_coupling;
    int current_p24;
    int current_p12;
    int current_p5;
    int current_p3;
};

struct VideoProcessing {
    // whereiam
    int whereiam_cls_id;
    float whereiam_cls_conf;
    float lpt, rpt;
    bool auto_pilot, inferring;
};

