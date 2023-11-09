#include <ros/ros.h>
#include <classes/Error/Error.h>

int main (int argc, char ** argv)
{
	ros::init(argc, argv, "Error_Node");
	ros::NodeHandle nh("~");
	ErrorLog error_log(nh);
	ros::spin();
	return 0;
}
