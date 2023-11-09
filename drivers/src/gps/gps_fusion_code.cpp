#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include "classes/gps/FusionGPS.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"gps_fusion_node");
    ROS_INFO_STREAM("gps_fusion_node");
    ros::NodeHandle nh("~");

    FusionGPS fusion(nh);

    ros::Rate rate(0.33);

    while(ros::ok())
    {
        fusion.publishGps();
        rate.sleep();
        ros::spinOnce();
    }
}