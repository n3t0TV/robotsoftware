#include "classes/Control/NavigationControl.h"
/*
 */

int main(int argc, char **argv) {
  ros::init(argc, argv, "navigation_control_node");
  ROS_INFO_STREAM("navigation_control_node");
  ros::NodeHandle nh("~");
  NavigationControl navControl(nh);
  ros::Rate rate(20);
  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
