#include <stdio.h>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <drivers/control_light_msg.h>

#ifdef JETSON
#include "classes/lights/LightDriverJetson.h"
#endif

#ifdef RASP
#include "classes/lights/LightDriverRasp.h"
#endif

using namespace std;

LightDriver *driver;
ros::Subscriber subLightControl;

void LightControlCallback(const drivers::control_light_msg& msg)
{
    driver->SetControl(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"light_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(1);
    
    driver = new LightDriver(nh);
    driver->Initialize();
    subLightControl = nh.subscribe("/control_topic/light",1,LightControlCallback);
    
    while(ros::ok())
    {
        driver->GetSensors();
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
