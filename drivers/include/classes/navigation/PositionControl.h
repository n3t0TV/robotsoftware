/* *
 * */

#include <ros/ros.h>
#include <string.h>
#include <ros/console.h>
#include <brainmodules/joystick_msg.h>
#include <brainmodules/testing_msg.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <cmath>
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
  
class PositionControl
{
    public:
	PositionControl(std::string name, ros::NodeHandle);

    private:
	std::string node_name;
	ros::NodeHandle nh;
	ros::Subscriber subImu, subOrientation, subVelocity, subActivate;
	ros::Publisher pubNavigation, pubStatus, pubCurrYaw;
	void OrientationGoalCallback(const std_msgs::Float32 &orientation);
	void VelocityGoalCallback(const std_msgs::Int32 &vel);
	void ActivateCallback(const std_msgs::Bool &activate);
	void ImuCallback(const sensor_msgs::Imu &msg);
	void goalCB();
	void executeCB(){}
	brainmodules::joystick_msg nav_msg;
	brainmodules::testing_msg test_msg;
	double roll, pitch, yaw;
	double goal;
	double kp, kd, ki, error_i, error_d, prev_error,error_threshold;
	ros::Time current_time, last_time;
};
  
PositionControl::PositionControl(std::string name, ros::NodeHandle nh_priv)
{
    node_name = ros::this_node::getName	();
    int log_level;
    nh_priv.param("log_level", log_level,0);
    nh_priv.param("kp", kp,0.01);
    nh_priv.param("kd", kd,0.0);
    nh_priv.param("ki", ki,0.0);
    nh_priv.param("error_threshold", error_threshold,0.1);
    ros::console::levels::Level console_level;
    console_level = (ros::console::levels::Level)log_level;
    //~ ROS_INFO_STREAM(node_name << " --- Level: " << console_level);
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
	subActivate = nh.subscribe("/position_control",10,&PositionControl::ActivateCallback, this);
	subOrientation = nh.subscribe("/orientation_goal",10,&PositionControl::OrientationGoalCallback, this);
	subVelocity = nh.subscribe("/velocity_goal",10,&PositionControl::VelocityGoalCallback, this);
	subImu = nh.subscribe("/imu",10,&PositionControl::ImuCallback, this);
	pubNavigation = nh.advertise<brainmodules::joystick_msg>("navigation_topic", 1);
	pubStatus = nh.advertise<brainmodules::testing_msg>("testing_topic", 1);
	pubCurrYaw = nh.advertise<std_msgs::Float32>("yaw_topic", 1);
	
}

void PositionControl::ImuCallback(const sensor_msgs::Imu &msg)
{
	tf::Quaternion quat(msg.orientation.x,msg.orientation.y, msg.orientation.z, msg.orientation.w);
	tf::Matrix3x3 m(quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	if (!test_msg.testing)
		return;
	current_time = ros::Time::now();
	ros::Duration dt = current_time - last_time;
	double error = goal - yaw;
	if (abs(error) > M_PI && error < 0)
		error += 2 * M_PI;
	else if (abs(error) > M_PI && error > 0)
		error -= 2 * M_PI;
	if (error < error_threshold && error > -error_threshold)
	{
		ROS_INFO_STREAM("Goal succeeded: " << yaw);
		nav_msg.AngularRate = 0;
		nav_msg.DrivingMode = 2;
		pubNavigation.publish(nav_msg);
		//~ error_i = prev_error = 0;
		//~ test_msg.testing = false;
		//~ test_msg.mode = brainmodules::testing_msg::TEST_NAVIGATION;
		//~ pubStatus.publish(test_msg);
		return;
	}
	error_d = (error - prev_error) / dt.toSec();
	error_i += error * dt.toSec(); 
	nav_msg.AngularRate = error * kp + error_i * ki + error_d *kd;
	nav_msg.DrivingMode = 2;
	pubNavigation.publish(nav_msg);
	std_msgs::Float32 yaw_msg;
	yaw_msg.data = yaw;
	pubCurrYaw.publish(yaw_msg);
	ROS_INFO_STREAM("Current error: " << error);
	last_time = current_time;
		
}
void PositionControl::OrientationGoalCallback(const std_msgs::Float32 &orientation)
{
	if(orientation.data==goal)
		return;
	error_i = prev_error = 0;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	goal = fmod(orientation.data, 2 * M_PI);
	//~ goal = goal % (2.0 * M_PI);
	test_msg.testing = true;
	test_msg.mode = brainmodules::testing_msg::TEST_NAVIGATION;
	ROS_INFO_STREAM("Goal accepted: " << goal);
}

void PositionControl::VelocityGoalCallback(const std_msgs::Int32 &vel)
{
    if (!test_msg.testing) return;
    nav_msg.Speed = vel.data;
}

void PositionControl::ActivateCallback(const std_msgs::Bool &activate)
{
	test_msg.testing = activate.data;
	test_msg.mode = brainmodules::testing_msg::TEST_NAVIGATION;
	if (test_msg.testing)
		ROS_INFO_STREAM("Navigation Module Active");
	else 
	{
		//~ nav_msg.DrivingMode = 2;
		//~ pubNavigation.publish(nav_msg);
		ROS_INFO_STREAM("Navigation Module Inactive");
	}
	pubStatus.publish(test_msg);
	goal = yaw;
}
