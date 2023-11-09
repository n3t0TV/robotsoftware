#include <stdio.h>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <drivers/control_expcamera_msg.h>

#define DEFAULT_EXP 20
#define AUTO_EXPOSURE -1

using namespace std;

ros::Subscriber subCameraExposure;
int currentExp;

void Initialize()
{
    //setting auto exposure (1 for manual)
    system("v4l2-ctl -d /dev/camera_front --set-ctrl exposure_auto=3");
    system("v4l2-ctl -d /dev/camera_back --set-ctrl exposure_auto=3");

    //recommended value
    currentExp = DEFAULT_EXP;

    ROS_DEBUG_STREAM("Camera exposure initialized");
}

void ChangeExposureCallback(const drivers::control_expcamera_msg& msg)
{
    if (msg.cameraExp != currentExp) {
        int nRet = 0;
        currentExp = msg.cameraExp;
        if (currentExp != AUTO_EXPOSURE) {
            ROS_DEBUG_STREAM("Changing camera exposure to " << currentExp << "...");

            nRet = system("v4l2-ctl -d /dev/camera_front --set-ctrl exposure_auto=1");
            nRet = system(("v4l2-ctl -d /dev/camera_front --set-ctrl exposure_absolute=" + to_string(currentExp)).c_str());
            if (nRet != 0)
                ROS_ERROR_STREAM("Error setting exposure to camera_front");
            else
                ROS_DEBUG_STREAM("Exposure camera_front done");

            nRet = system("v4l2-ctl -d /dev/camera_back --set-ctrl exposure_auto=1");
            nRet = system(("v4l2-ctl -d /dev/camera_back --set-ctrl exposure_absolute=" + to_string(currentExp)).c_str());
            if (nRet != 0)
                ROS_ERROR_STREAM("Error setting exposure to camera_back");
            else
                ROS_DEBUG_STREAM("Exposure camera_back done");
        } else {
            ROS_DEBUG_STREAM("Changing camera auto-exposure");

            nRet = system("v4l2-ctl -d /dev/camera_front --set-ctrl exposure_auto=3");
            if (nRet != 0)
                ROS_ERROR_STREAM("Error setting auto-exposure to camera_front");
            else
                ROS_DEBUG_STREAM("Auto-exposure camera_front done");

            nRet = system("v4l2-ctl -d /dev/camera_back --set-ctrl exposure_auto=3");
            if (nRet != 0)
                ROS_ERROR_STREAM("Error setting auto-exposure to camera_back");
            else
                ROS_DEBUG_STREAM("Auto-exposure camera_back done");
        }
    }

}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"camera_exposure_node");
    ros::NodeHandle nh("~");
    
    int log_level;
    nh.param("log_level", log_level,0);
    ros::console::levels::Level console_level;
    console_level = (ros::console::levels::Level)log_level;
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    Initialize();
    subCameraExposure = nh.subscribe("/control_topic/cam_exp",10,ChangeExposureCallback);
    
    ros::spin();

}
