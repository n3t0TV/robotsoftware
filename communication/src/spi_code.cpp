#include <ros/ros.h>

#ifdef JETSON
#include "classes/Jetson/SPI_jetson.h"
#endif

#ifdef RASP
#include "classes/Raspberry/SPI_rasp.h"
#endif

int main (int argc, char ** argv)
{
	ros::init(argc, argv, "spi_Node");
	ros::NodeHandle n("~");
	SPI spi(n);
	ros::Rate rate(300);
	while (ros::ok())
	{		
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
