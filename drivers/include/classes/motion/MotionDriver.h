#include <iostream>
#include <string>
#include <ros/console.h>
#include <drivers/sensor_motion_msg.h>
#include <drivers/control_motion_msg.h>
#include "classes/SPI/SPIDriver.h"
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#define EXCEPTION_TIME 0.5

class MotionDriver:virtual public SPIDriver
{
	public:
		MotionDriver(ros::NodeHandle nh_priv);
		virtual ~MotionDriver();
		int GetSensors();
		void SetMotionControl(const drivers::control_motion_msg& msg);
		void SetBrakeControl(const std_msgs::Bool& msg);

	private:
		void PublishException();
		drivers::sensor_motion_msg sensor_msg;
		ros::Publisher pubMotionSensors, pubBrainException;
		ros::Time lastControlMessage;
		ros::Duration lastMessageElapsedTime;
		ros::Timer exceptionTimer;
		ros::ServiceServer pid_params_srv;
		std_msgs::Byte err_msg;
		struct pid_param {
			float kp{0};
			float ki{0};
			float kd{0};
		} pid_right, pid_left;
		bool SetControlParams(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
		void SetParams(const pid_param& params, const string& sufix);
};

MotionDriver::MotionDriver(ros::NodeHandle nh_priv):SPIDriver(nh_priv)
{
	commandsDir = "/config/wagonIndiaMotion.json";
	pubMotionSensors = nh.advertise <drivers::sensor_motion_msg>("sensor_topic/motion", 10);
	pubBrainException = nh.advertise <std_msgs::Byte>("exception_topic/brain", 10);
	exceptionTimer = nh.createTimer(ros::Duration(EXCEPTION_TIME), std::bind(&MotionDriver::PublishException, this));
	pid_params_srv = nh.advertiseService("motion/pid_params",&MotionDriver::SetControlParams, this);
	lastControlMessage = ros::Time::now();
	err_msg.data = FLAG_BRAIN_INTRN;
}

MotionDriver::~MotionDriver()
{
	ROS_DEBUG_STREAM("destructor MotionDriver");
}

bool MotionDriver::SetControlParams(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    /** Parametros de control **/
    nh.param(node_name + "/right/kp", pid_right.kp, pid_right.kp);
    nh.param(node_name + "/right/ki", pid_right.ki, pid_right.ki);
    nh.param(node_name + "/right/kd", pid_right.kd, pid_right.kd);
    nh.param(node_name + "/left/kp", pid_left.kp, pid_left.kp);
    nh.param(node_name + "/left/ki", pid_left.ki, pid_left.ki);
    nh.param(node_name + "/left/kd", pid_left.kd, pid_left.kd);

	SetParams(pid_right, "RIGHT");
	SetParams(pid_left, "LEFT");

	res.success = true;
	res.message = "Params updated! Right: kp=" + to_string(pid_right.kp) + " ki=" + to_string(pid_right.ki) + " kd=" + to_string(pid_right.kd) +
					" Left: kp=" + to_string(pid_left.kp) + " ki=" + to_string(pid_left.ki) + " kd=" + to_string(pid_left.kd);
	return true;
}

void MotionDriver::SetParams(const pid_param& params, const string& sufix)
{
	/* kp */
	int16_t param_integ = (int) params.kp;
	int16_t param_frac = (params.kp - param_integ) * 32768;
	ROS_INFO("%s -- KP_%s param int: %d, frac: %d", node_name.c_str(), sufix.c_str(), param_integ, param_frac);
	SetCommand("VEL_CONTROL_KP_FRAC_" + sufix, param_frac);
	SetCommand("VEL_CONTROL_KP_INTEG_" + sufix, param_integ);
	/* ki */
	param_integ = (int) params.ki;
	param_frac = (params.ki - param_integ) * 32768;
	ROS_INFO("%s -- KI_%s param int: %d, frac: %d", node_name.c_str(), sufix.c_str(), param_integ, param_frac);
	SetCommand("VEL_CONTROL_KI_FRAC_" + sufix, param_frac);
	SetCommand("VEL_CONTROL_KI_INTEG_" + sufix, param_integ);
	/* kd */
	param_integ = (int) params.kd;
	param_frac = (params.kd - param_integ) * 32768;
	ROS_INFO("%s -- KD_%s param int: %d, frac: %d", node_name.c_str(), sufix.c_str(), param_integ, param_frac);
	SetCommand("VEL_CONTROL_KD_FRAC_" + sufix, param_frac);
	SetCommand("VEL_CONTROL_KD_INTEG_" + sufix, param_integ);
}

int MotionDriver::GetSensors()
{
	sensor_msg.DriveSpeed = GetCommand<int>("DRIVE_SPEED_SP");
	sensor_msg.DriveSpeedCurr = GetCommand<int>("DRIVE_SPEED_CURR");
	sensor_msg.DriveAngularRate = GetCommand<float>("DRIVE_ANGULAR_RATE_SP");
	sensor_msg.DriveAngularRateCurr = GetCommand<float>("DRIVE_ANGULAR_RATE_CURR");
	sensor_msg.DriveSpeedRightCurr = GetCommand<int>("VEL_CONTROL_VELOCITY_RIGHT");
	sensor_msg.DriveSpeedLeftCurr = GetCommand<int>("VEL_CONTROL_VELOCITY_LEFT");

	pubMotionSensors.publish(sensor_msg);

	return 0;
}

void MotionDriver::SetBrakeControl(const std_msgs::Bool& msg)
{
	SetCommand("DRIVE_BRAKES", msg.data);
}

void MotionDriver::SetMotionControl(const drivers::control_motion_msg& msg)
{
	switch (msg.DrivingMode)
	{
		case 0:
			SetCommand("DRIVE_SPEED_SP_RIGHT", 0);
			SetCommand("DRIVE_SPEED_SP_LEFT", 0);
		break;
		case 1:
			SetCommand("DRIVE_SPEED_SP_RIGHT", msg.DriveSpeedRight);
			SetCommand("DRIVE_SPEED_SP_LEFT", msg.DriveSpeedLeft);
		break;	
		case 2:
			SetCommand("DRIVE_SPEED_SP", msg.DriveSpeed);
			SetCommand("DRIVE_ANGULAR_RATE_SP", msg.DriveAngularRate);
		break;
	}

	lastControlMessage = ros::Time::now();

}

void MotionDriver::PublishException()
{
	lastMessageElapsedTime = ros::Time::now() - lastControlMessage;
	if (lastMessageElapsedTime.toSec() > EXCEPTION_TIME) {
		err_msg.data = FLAG_BRAIN_INTRN;

		/** Fatal Error - Emergency Stop*/
		SetCommand("DRIVE_SPEED_SP", 0);
		SetCommand("DRIVE_ANGULAR_RATE_SP", 0);
		SetCommand("DRIVE_BRAKES", 1);
	} else {
		ROS_INFO_STREAM("PublishException - OK");
		err_msg.data = STATUS_OK;
	}

	pubBrainException.publish(err_msg);
}
