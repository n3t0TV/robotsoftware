#include <ros/ros.h>

#ifdef JETSON
#include "classes/audio/audio_player.h"
#endif

#ifdef RASP
#include "classes/audio/SpeakerRasp.h"
#endif

using namespace std;

int main( int argc, char**argv)
{
    ros::init(argc, argv, "audio_player_node");
    ros::NodeHandle nh("~");
    AudioPlayer player(nh);
    ros::Rate rate(2);

    ROS_DEBUG_STREAM("audio_player_node");
    while(ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}
