#include <ctime>
#include <string.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Byte.h>

#include "libraries/json.hpp"
#include <modules/state_msg.h>
#include <modules/joystick_msg.h>

using json = nlohmann::json;
using namespace std;

class ControlSamples
{
    public:
	ControlSamples(ros::NodeHandle);
	~ControlSamples(){};
	
	void go(int angle);
	void stop();
	int time, angle;

    private:
	ros::NodeHandle nh;
	ros::Publisher pubControl;
	modules::joystick_msg comandos;
	std_msgs::Byte err_msg;
	string node_name;
	int log_level,speed;
	bool speed_in_meters_per_second{false};
	int speed_conversion_const {1};		
};
    
ControlSamples::ControlSamples(ros::NodeHandle nh_priv)
{
    node_name = ros::this_node::getName	();    
    nh_priv.param("log_level", log_level,0);
    ros::console::levels::Level console_level;
    console_level = (ros::console::levels::Level)log_level;
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    nh.param("modular", speed_in_meters_per_second,true);
    nh_priv.param("time",time,5);
    nh_priv.param("speed",speed,10);
    nh_priv.param("angle",angle,0);
    
    pubControl = nh.advertise<modules::joystick_msg>("control_topic", 10);

    if (speed_in_meters_per_second) speed_conversion_const = 0.01;
    comandos.Speed = speed;
    comandos.Angle = 0;
    comandos.AngularRate = 0;
    comandos.TurningLights = 0;
    comandos.Illumination = 0;
    comandos.Brake = 0;
    comandos.Reset = 0;
    comandos.EmergencyStop = false;
    comandos.EnabledEmergencyStop = true;
    comandos.TimeThreshold_ms = 2000;
    comandos.LeftWheel = 0;
    comandos.RightWheel = 0;
    comandos.DrivingMode = 2;
    comandos.OrientationSetpoint = 0;
    comandos.MovCamera = 0;
    comandos.cameraExp = 20;
    comandos.videoMode = 1;
}

void ControlSamples::go(int angle)
{	
    comandos.AngularRate = angle;
    pubControl.publish(comandos);
}

void ControlSamples::stop()
{	
    comandos.Speed = 0;
    comandos.AngularRate = 0;
    comandos.Brake = 1;
    pubControl.publish(comandos);
}
