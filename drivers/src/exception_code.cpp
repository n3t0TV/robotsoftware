#include <ros/ros.h>

#include "classes/VehicleException.h"

using namespace std;

int main(int argc, char **argv)
{
	//Initialize and start the node
	ros::init(argc,argv,"exception_node");	
	ros::NodeHandle nh("~");	
	VehicleException exceptHandler(nh);
	ros::Rate rate(5);
	while(ros::ok())
	{
		exceptHandler.Publish();
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
