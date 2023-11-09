#include <ros/ros.h>
#include "classes/navigation/OrientationControl.h"
/*

*/
int main(int argc, char **argv)
{
	ros::init(argc,argv,"orientation_control_node");
	ROS_INFO_STREAM("orientation_control_node");
	ros::NodeHandle nh("~");
	OrientationControl oc(nh);
	ros::Rate rate(10);
	while(ros::ok())
	{
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
	
}
