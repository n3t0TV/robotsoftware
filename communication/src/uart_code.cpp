#include <ros/ros.h>

#ifdef JETSON
#include "classes/Jetson/UART_jetson.h"
#endif

#ifdef RASP
#include "classes/Raspberry/UART_rasp.h"
#endif

int main (int argc, char ** argv)
{
	ros::init(argc, argv, "Program_PIC_Node");
	ros::NodeHandle n("~");
	UART uart(n);
	ros::Rate rate(1);
	while (ros::ok())
	{
		
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}

