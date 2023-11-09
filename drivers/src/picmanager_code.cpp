#include <stdio.h>
#include <iostream>
#include <string>
#include <ros/ros.h>

#ifdef JETSON
#include "classes/pic/PicManagerDriverJetson.h"
#endif

#ifdef RASP
#include "classes/pic/PicManagerDriverRasp.h"
#endif

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"picmanager_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(10);

    PicManagerDriver driver(nh);
    driver.Initialize();
    driver.InitUPS();   
    
    while(ros::ok())
    {
        driver.GetSensors();
        rate.sleep();
        ros::spinOnce();
    }
    
    driver.DeinitUPS();

    return 0;
}
