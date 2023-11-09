#include <ros/ros.h>
#include <ros/console.h>
#include "classes/Test/VideoMetrics.h"
/*

*/
int main(int argc, char **argv)
{
	ros::init(argc,argv,"video_metrics_node");
	ROS_INFO_STREAM("video_metrics_node");
	ros::NodeHandle nh("~");
	VideoMetrics video_metrics(nh);
	ros::Rate rate(15);
	while(ros::ok())
	{
		rate.sleep();
		ros::spinOnce();		
	}
	return 0;
	
}
