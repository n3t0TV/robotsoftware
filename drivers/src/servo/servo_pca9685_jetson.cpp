#include <ros/ros.h>
#include "classes/servo/PCA9685/ServoPCA9685Jetson.h"
/*

*/
int main(int argc, char **argv)
{
	ros::init(argc,argv,"servo_node");
	ROS_INFO_STREAM("servo_node Jetson");
	ros::NodeHandle nh("~");
	ros::Rate rate(10);
	Servo servo(nh);
	servo.Servo_Initialize();
	
	while(ros::ok())
	{
		ros::spinOnce();		
		rate.sleep();
	}
	return 0;
	
}

