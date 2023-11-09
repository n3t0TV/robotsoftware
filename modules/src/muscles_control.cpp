#include <ros/ros.h>
#include <ros/console.h>
#include "classes/Control/MusclesControl.h"
/*

*/
int main(int argc, char **argv)
{
	ros::init(argc,argv,"muscles_control_node");
	ROS_INFO_STREAM("muscles_control_node");
	ros::NodeHandle nh("~");
	MusclesControl musclesControl(nh);
	ros::Rate rate(20);
	while(ros::ok())
	{
		rate.sleep();
		ros::spinOnce();		
	}
	return 0;
	
}

