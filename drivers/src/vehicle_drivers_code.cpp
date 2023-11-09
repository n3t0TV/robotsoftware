#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <iostream>
#include <string>
#include <modules/joystick_msg.h>
#include "classes/VehicleCore.h"

using namespace std;

int main(int argc, char **argv)
{
	//Initialize and start the node
	ros::init(argc,argv,"vehicle_drivers_node");
	ROS_INFO_STREAM("vehicle_drivers_node");
	ros::NodeHandle nh("~");
	ros::Rate rate(20);
	VehicleCore vehicle(nh);
		
	while(ros::ok())
	{
		vehicle.PublishSensors();
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
