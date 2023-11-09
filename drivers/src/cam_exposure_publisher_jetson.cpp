#include "classes/cam_exposure_publisher.h"
#include "brainvision/CamExposure.h"


using CamExpMsg = brainvision::CamExposure;


static uint16_t UiToBrainVisionExpConversion(int32_t ui_exp)
{
    const int32_t conversion_coefficient = 1023;
    const int32_t conversion_divisor = 400;

    return (uint16_t) ((ui_exp * conversion_coefficient) / conversion_divisor);
}


CamExposurePublisher::CamExposurePublisher(ros::NodeHandle node_handle)
{
    ros::Publisher pub;
    const char* kCamExpZoomTopicName = "/brain_vision_zoom/cam_exp";
    const char* kCamExpFrontTopicName = "/brain_vision_front/cam_exp";
    const char* kCamExpSpinTopicName = "/brain_vision_spin/cam_exp";
    const bool kEnableLatch = true;

    pub = node_handle.advertise<CamExpMsg>(kCamExpZoomTopicName,
                                           kQueueSizeExpMsg, kEnableLatch);
    ros_cam_exp_pubs_.push_back(pub);

    pub = node_handle.advertise<CamExpMsg>(kCamExpFrontTopicName,
                                           kQueueSizeExpMsg, kEnableLatch);
    ros_cam_exp_pubs_.push_back(pub);

    pub = node_handle.advertise<CamExpMsg>(kCamExpSpinTopicName,
                                           kQueueSizeExpMsg, kEnableLatch);
    ros_cam_exp_pubs_.push_back(pub);
}


void CamExposurePublisher::Publish(int32_t exposure, int32_t video_mode)
{
    if(prev_exposure_ != exposure)
    {
        CamExpMsg msg;
        bool valid_exp = false;

        if(exposure == kAutomaticExpValue)
        {
            msg.auto_exp = true;
            msg.value = 0U;
            valid_exp = true;
        }
        else
        {
            msg.auto_exp = false;
            msg.value = UiToBrainVisionExpConversion(exposure);

            if((msg.MIN_EXP <= msg.value) && (msg.value <= msg.MAX_EXP))
            {
                valid_exp = true;
            }
            else
            {
                ROS_ERROR("The exposure value for brain vision is invalid"
                          " (%d), the provided UI exposure value is %d",
                          msg.value, exposure);
            }
        }

        if(valid_exp)
        {
            for(ros::Publisher pub : ros_cam_exp_pubs_)
            {
                pub.publish(msg);
            }
        }

        prev_exposure_ = exposure;
    }
}
