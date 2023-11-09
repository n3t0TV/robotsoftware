#include <stdio.h>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <drivers/control_motion_msg.h>
#include "classes/motion/MotionDriver.h"
#include <std_msgs/Bool.h>
using namespace std;

MotionDriver *driver;
ros::Subscriber subMotionControl;
ros::Subscriber subBrakeControl;

void MotionControlCallback(const drivers::control_motion_msg& msg)
{
    driver->SetMotionControl(msg);
    ROS_DEBUG_STREAM("Mensaje de control: " << msg);
}

void BrakeControlCallback(const std_msgs::Bool& msg)
{
    driver->SetBrakeControl(msg);
    ROS_DEBUG_STREAM("Mensaje de control: " << msg);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"motion_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(10);
    
    driver = new MotionDriver(nh);
    driver->Initialize();
    subMotionControl = nh.subscribe("/control_topic/motion",10,MotionControlCallback);
    subBrakeControl = nh.subscribe("/control_topic/brake",10,BrakeControlCallback);
    
    while(ros::ok())
    {
        driver->GetSensors();
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
