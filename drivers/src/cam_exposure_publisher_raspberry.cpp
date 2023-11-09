#include "classes/cam_exposure_publisher.h"
#include "drivers/control_expcamera_msg.h"


const char* kCamExpTopicName = "control_topic/cam_exp";

using CamExpMsg = drivers::control_expcamera_msg;


CamExposurePublisher::CamExposurePublisher(ros::NodeHandle node_handle)
{
    ros::Publisher pub = node_handle.advertise<CamExpMsg>(kCamExpTopicName,
                                                          kQueueSizeExpMsg);
    ros_cam_exp_pubs_.push_back(pub);
}


void CamExposurePublisher::Publish(int32_t exposure, int32_t video_mode)
{
    CamExpMsg msg;

    msg.cameraExp = exposure;
    msg.videoMode = video_mode;

    for(ros::Publisher pub : ros_cam_exp_pubs_)
    {
        pub.publish(msg);
    }
}
