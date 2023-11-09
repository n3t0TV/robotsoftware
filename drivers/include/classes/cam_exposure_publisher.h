#include <stdint.h>
#include <ros/ros.h>
#include <vector>

class CamExposurePublisher
{
    public:
        CamExposurePublisher(ros::NodeHandle ros_node_handle);

        void Publish(int32_t exposure, int32_t video_mode);

    private:
        const uint32_t kQueueSizeExpMsg = 10U;

        const int32_t kMinUiExp = 2;

        const int32_t kMaxUiExp = 400;

        const int32_t kInvalidExpValue = -10;

        const int32_t kAutomaticExpValue = -1;

        std::vector<ros::Publisher> ros_cam_exp_pubs_;

        int32_t prev_exposure_ = kInvalidExpValue;
};
