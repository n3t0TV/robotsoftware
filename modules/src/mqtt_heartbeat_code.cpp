#include <ros/ros.h>
#include "classes/MQTT/MQTTHeartbeat.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "mqtt_heartbeat_node");
  ROS_INFO_STREAM("heartbeatMqttNode");
  ros::NodeHandle nh("~");
  MQTTHeartbeat heartbeat_mqtt_pub(nh);

  ros::Rate rate(1);

  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
