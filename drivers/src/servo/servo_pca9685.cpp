#include <ros/ros.h>
#include "classes/servo/ServoPCA9685_V2.h"
/*

*/
int main(int argc, char **argv)
{
	ros::init(argc,argv,"servo_node");
	ROS_INFO_STREAM("servo_node");
	ros::NodeHandle nh("~");
	ros::Rate rate(10);
	Servo servo(nh);
	servo.Initialize();
	
	while(ros::ok())
	{
		ros::spinOnce();		
		rate.sleep();
	}
	return 0;
	
}

