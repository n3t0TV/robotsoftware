#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include "libraries/json.hpp"
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <math.h>
using namespace std;
using json = nlohmann::json;
class ErrorLog
{
	public:
		ErrorLog(ros::NodeHandle);
		~ErrorLog(){};
	private:
		ros::NodeHandle nh;
		ros::Subscriber error_sub, log_enable_sub;
		ros::Publisher error_pub;
		string node_name;
		json error_json;
		bool log_enable;
		void ROSErrorsCallback(const rosgraph_msgs::Log &);
		void LogEnableCallback(const std_msgs::Bool &);
};
ErrorLog::ErrorLog(ros::NodeHandle nh_priv)
{
	node_name = ros::this_node::getName	();
	int log_level;
	nh_priv.param("log_level", log_level,0);
	ros::console::levels::Level console_level;
	console_level = (ros::console::levels::Level)log_level;
	ROS_INFO_STREAM(node_name << " --- Level: " << console_level);
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level) ) {
		ros::console::notifyLoggerLevelsChanged();
	}
	error_sub = nh.subscribe("rosout_agg", 100, &ErrorLog::ROSErrorsCallback, this);
	log_enable_sub = nh.subscribe("log_enable_topic", 1, &ErrorLog::LogEnableCallback, this);
	error_pub = nh.advertise<std_msgs::String>("scooter_error", 100);
	log_enable = false;
}

void ErrorLog::ROSErrorsCallback(const rosgraph_msgs::Log & log_msg)
{
	if (!log_enable)
		return;
	std_msgs::String error_msg;
	if (log_msg.level>0)
	{
		error_json["LEVEL"] = log2(log_msg.level);
		error_json["NODE_NAME"] = log_msg.name;
		error_json["MESSAGE"] = log_msg.msg;
		error_json["FILE"] = log_msg.file;
		error_json["LINE"] = log_msg.line;
		ROS_INFO_STREAM("ERROR DE CASTEO");
		error_msg.data = error_json.dump();
		error_pub.publish(error_msg);
	}
}
void ErrorLog::LogEnableCallback(const std_msgs::Bool &log_enable_msg)
{
		log_enable = log_enable_msg.data;
}
