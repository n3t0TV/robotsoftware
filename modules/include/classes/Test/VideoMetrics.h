/* *
 * 
 * */

#include <ros/ros.h>
#include <string.h>
#include <ros/console.h>
#include <modules/processed_video_metrics.h>
#include <modules/metrics_msg.h>
#include <std_msgs/Duration.h>

  
class VideoMetrics
{
    public:
	VideoMetrics(ros::NodeHandle);

	void MsgCallback(const modules::metrics_msg msg);
	int StateMachine();

	int status;
	//int lastResetValue;
    private:
	std::string node_name;
	ros::NodeHandle nh;
	ros::Subscriber subVideoMetrics;
	ros::Publisher pubVideoMetricsProcessed;
	modules::processed_video_metrics proc_metrics; 
	int prev_frames_dropped{0};
	int prev_diff_bytes{0};
	int prev_diff_packets{0};
};

VideoMetrics::VideoMetrics(ros::NodeHandle nh_priv)
{
    node_name = ros::this_node::getName	();
    int log_level;
    nh_priv.param("log_level", log_level,0);
    ros::console::levels::Level console_level;
    console_level = (ros::console::levels::Level)log_level;
    //~ ROS_INFO_STREAM(node_name << " --- Level: " << console_level);
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    subVideoMetrics = nh.subscribe("/metrics_topic",10,&VideoMetrics::MsgCallback, this);
    pubVideoMetricsProcessed = nh.advertise<modules::processed_video_metrics>("proc_metrics_topic", 1);
}


void VideoMetrics::MsgCallback(const modules::metrics_msg msg)
{
    proc_metrics.deltaFramesDropped = msg.framesDropped - prev_frames_dropped;
    proc_metrics.diffBytes = msg.bytesSent - msg.bytesReceived;
    proc_metrics.diffPackets = msg.packetsSent - msg.packetsReceived;
    proc_metrics.deltaBytes = proc_metrics.diffBytes - prev_diff_bytes;
    proc_metrics.deltaBytes = proc_metrics.diffPackets - prev_diff_packets;
    pubVideoMetricsProcessed.publish(proc_metrics);
    prev_frames_dropped = msg.framesDropped;
    prev_diff_bytes = proc_metrics.diffBytes;
    prev_diff_packets = proc_metrics.diffPackets ;
}




