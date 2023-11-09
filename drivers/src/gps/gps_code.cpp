#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include "classes/gps/GPS_GSM_v3.h"	

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc,argv,"gps_node");
	ros::NodeHandle nh("~");
	
	GPS_GSM gps(nh);
	ROS_INFO_STREAM(gps.node_name);
	gps.initModule();
	
	ros::Rate rate(0.33);	
	while(ros::ok())
	{		
		gps.publishGpsData();
		rate.sleep();
		ros::spinOnce();
	}
    
	return 0;
}
