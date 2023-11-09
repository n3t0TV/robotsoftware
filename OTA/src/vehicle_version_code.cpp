#include <ros/ros.h>

#include "classes/VehicleVersion.h"

using namespace std;

int main(int argc, char **argv)
{
	//Initialize and start the node
	ros::init(argc,argv,"vehicle_version_node");	
	ros::NodeHandle nh("~");	
	VersionManager manager(nh);
	ros::Rate rate(10);

	while(ros::ok())
	{
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
