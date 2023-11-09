#include <ros/ros.h>
#include <ros/console.h>
#include "classes/navigation/DiffOdometry.h"
/*

*/
int main(int argc, char **argv)
{
	ros::init(argc,argv,"diff_odom_node");
	ROS_INFO_STREAM("diff_odom_node");
	ros::NodeHandle nh("~");
	DiffOdometry diff_odom(nh);
	ros::Rate rate(100);
	while(ros::ok())
	{
		rate.sleep();
		ros::spinOnce();		
	}
	return 0;
	
}
