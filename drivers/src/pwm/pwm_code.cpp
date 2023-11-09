#include <ros/ros.h>
#include "classes/I2C/PCA9685Jetson.h"
/*

*/
int main(int argc, char **argv)
{
	ros::init(argc,argv,"pwm_node");
	ROS_INFO_STREAM("pwm_node Jetson");
	ros::NodeHandle nh("~");
	ros::Rate rate(10);
	PCA9685 pwmDriver(nh);
	pwmDriver.Initialize();
	
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
	
}


