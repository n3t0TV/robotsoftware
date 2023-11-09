#include <ros/ros.h>
#include <ros/console.h>
#include "classes/UDP/TeleopsControl.h"
/*

*/
int main(int argc, char **argv)
{
	ros::init(argc,argv,"controlsocketNode");
	ROS_INFO_STREAM("control_socket_node");
	ros::NodeHandle nh("~");
	ROS_INFO_STREAM("Starting control node!!!");
	TeleopsControl teleopsControl(nh);
	ros::Rate rate(30);
	teleopsControl.Initialize();
	while(ros::ok())
	{
		teleopsControl.stateMachine();
		rate.sleep();
		ros::spinOnce();		
	}
	return 0;
	
}

