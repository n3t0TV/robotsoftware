#include "classes/MQTT/MQTTError.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "mqtt_error_node");
	ros::NodeHandle nh("~");
	MQTTError error_mqtt_pub(nh);
	ros::Rate rate(10);
	ros::spin();
	return 0;
}
