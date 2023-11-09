//~ #include "classes/OTAManager.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "netmonitor_node");
	ros::NodeHandle nh("~");
	//~ OTAManager ota_manager(nh);
	ros::Rate rate(5);
	
	while (ros::ok())
	{		
		rate.sleep();
		ros::spinOnce();
	}
	ros::spinOnce();
	return 0;
}
