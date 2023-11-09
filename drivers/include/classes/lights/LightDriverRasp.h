#include <iostream>
#include <string>
#include <ros/console.h>
#include <drivers/sensor_light_msg.h>
#include <drivers/control_light_msg.h>
#include "classes/SPI/SPIDriver.h"

#define POLE_PERIOD_MS 1000

//TODO: quitar cuando este nuevo hardware
#define HEADLIGHTS 	0b00000001
#define	POLE 		0b00000010
#define	TAIL		0b00000100
#define STOP		0b00001000
#define REVERSE 	0b00010000

class LightDriver:virtual public SPIDriver
{
	public:
		LightDriver(ros::NodeHandle nh_priv);
		virtual ~LightDriver();
		int GetSensors();
		void SetControl(const drivers::control_light_msg& msg);

	private:
		drivers::sensor_light_msg sensor_msg;
		ros::Publisher pubLightSensors;
		int pole_count;
};

LightDriver::LightDriver(ros::NodeHandle nh_priv):SPIDriver(nh_priv)
{
	commandsDir = "/braintemp/wagonIndiaLight.json";
	pubLightSensors = nh.advertise <drivers::sensor_light_msg>("sensor_topic/light", 10);
	pole_count = 0;
}

LightDriver::~LightDriver()
{
	ROS_DEBUG_STREAM("destructor LightDriver");
}

int LightDriver::GetSensors()
{
	sensor_msg.IlluminationFPWMDuty = GetCommand<int>("ILLUMINATION_F_PWM_DUTY", false);
	sensor_msg.IlluminationRPWMDuty = GetCommand<int>("ILLUMINATION_R_PWM_DUTY", false);
	sensor_msg.BlinkerState = GetCommand<int>("BLINKER_STATE", false);
	sensor_msg.PoleState = GetCommand<int>("LIGHT_POLE_STATE", false);
	sensor_msg.ReverseState = GetCommand<int>("LIGHT_REVERSE_STATE", false);
	sensor_msg.StopState = GetCommand<int>("LIGHT_STOP_STATE", false);
	sensor_msg.HeadState = GetCommand<int>("LIGHT_HEAD_STATE", false);
	sensor_msg.HeadPWMDuty = GetCommand<int>("HEAD_LIGHT_PWM_DUTY", false);  //Falta var de ui
	sensor_msg.TailState = GetCommand<int>("LIGHT_TAIL_STATE", false);

	pubLightSensors.publish(sensor_msg);

	return 0;
}

void LightDriver::SetControl(const drivers::control_light_msg& msg)
{
	SetCommand("ILLUMINATION_F_PWM_DUTY", msg.IlluminationFPWMDuty, false);
	SetCommand("ILLUMINATION_R_PWM_DUTY", msg.IlluminationRPWMDuty, false);
	SetCommand("BLINKER_STATE", msg.BlinkerState, false);
	SetCommand("LIGHT_POLE_STATE", msg.PoleState, false);
	SetCommand("LIGHT_POLE_PERIOD_MS", POLE_PERIOD_MS, false);
	SetCommand("LIGHT_REVERSE_STATE", msg.ReverseState, false);
	SetCommand("LIGHT_STOP_STATE", msg.StopState, false);
	SetCommand("LIGHT_HEAD_STATE", msg.HeadState, false);
	SetCommand("HEAD_LIGHT_PWM_DUTY", msg.HeadPWMDuty, false);//falta la var del ui
	SetCommand("LIGHT_TAIL_STATE", msg.TailState, false);

	//TODO: quitar cuando este nuevo hardware
	int lightState = 0;
	if (msg.StopState == 1) lightState += STOP;
	if (msg.ReverseState == 1) lightState += REVERSE;
	if (msg.HeadState == 1) lightState += HEADLIGHTS;
	if (msg.TailState == 1) lightState += TAIL;
	if (pole_count%10<5 && msg.PoleState == 2) lightState += POLE;
	pole_count ++;

	SetCommand("LIGHT_STATE", lightState, false);
	ROS_WARN_STREAM("LIGHT_STATE: " << lightState);
}
