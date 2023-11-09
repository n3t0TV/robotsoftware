#include <ros/ros.h>
#include "libraries/json.hpp"
#include "classes/imu/ImuSpi.h"

int main(int argc, char **argv)
{
	//Initialize and start the node
	ros::init(argc,argv,"imu_spi_node");
	ros::NodeHandle nh("~");
	ROS_INFO_STREAM("Starting imu_spi_node");
	
	ImuSpi imu(nh);
	ros::Rate rate(10);
	imu.Initialize();
	while(ros::ok())
	{
		imu.GetSensors();
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
