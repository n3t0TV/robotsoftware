#include <ros/ros.h>
#include <modules/imei_service.h>
#include <modules/mqtt_subscribe_srv.h>
#include <modules/mqtt_publishers_msg.h>
#include <modules/mqtt_subscribers_msg.h>
#include <std_msgs/String.h>
#include "libraries/json.hpp"
#include <string.h>

using namespace std;
using json = nlohmann::json;

class MQTTError 
{
	public:
		MQTTError(ros::NodeHandle nh_priv);
		~MQTTError(){};
		void PublishError();
	private:
		ros::NodeHandle nh;
		ros::Publisher mqtt_publishers_pub;
		ros::Subscriber mqtt_subscribers_sub;
		ros::Subscriber scooter_error_sub;
		ros::Publisher server_error_pub;
		ros::ServiceClient imei_service_client, mqtt_subscribe_client;
		string error_base_topic;
		string node_name;
		string server_topic, vehicle_topic;
		modules::mqtt_publishers_msg error_msg;
		void MQTTSubscribersCallback(const modules::mqtt_subscribers_msg&);
		void MQTTScooterErrorCallback(const std_msgs::String&);
				
};

MQTTError::MQTTError(ros::NodeHandle nh_priv)
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
	nh_priv.param<string>("error_base_topic", error_base_topic,"error");
	mqtt_publishers_pub = nh.advertise<modules::mqtt_publishers_msg>("mqtt_publishers", 1);
	mqtt_subscribers_sub = nh.subscribe("mqtt_subscribers", 1, &MQTTError::MQTTSubscribersCallback, this);
	//server_error_pub = nh.advertise<modules::server_error_msg>("server_error",1);
	scooter_error_sub = nh.subscribe("scooter_error", 1, &MQTTError::MQTTScooterErrorCallback, this);
	imei_service_client = nh.serviceClient<modules::imei_service>("imei_service");
	mqtt_subscribe_client = nh.serviceClient<modules::mqtt_subscribe_srv>("mqtt_subscribe");
	modules::imei_service imei_srv;
	ros::service::waitForService ("imei_service");
	ROS_INFO_STREAM(node_name << " --- Waiting for *** imei_service *** service");
	if (!imei_service_client.call(imei_srv))
	{
		ROS_ERROR_STREAM(node_name << " --- Cannot call IMEI service ");
	}
	string imei = imei_srv.response.imei;
	vehicle_topic = error_base_topic + "/" + "vehicle/" + imei;
	server_topic = error_base_topic + "/" + "server/" + imei;
	modules::mqtt_subscribe_srv subscribe_srv;
	ros::service::waitForService ("mqtt_subscribe");
	ROS_INFO_STREAM(node_name << " --- Waiting for *** mqtt_subscribe *** service");
	subscribe_srv.request.mqtt_topic = server_topic;
	subscribe_srv.request.node_id = node_name;
	if(!mqtt_subscribe_client.call(subscribe_srv))
	{
		ROS_ERROR_STREAM(node_name << " --- Cannot call MQTT subscribe service ");
	}
	
}

void MQTTError::MQTTSubscribersCallback(const modules::mqtt_subscribers_msg& subscribers_msg)
{
	if (subscribers_msg.mqtt_topic == error_base_topic)
	{
		ROS_DEBUG_STREAM (node_name << " --- mqtt status msg: " << subscribers_msg.raw_msg);
		json jsonResponse = json::parse(subscribers_msg.raw_msg); 
		//~ if(jsonResponse.contains("tipoError")){
			//~ status_msg.status = jsonResponse["ID_ESTATUS"];
			//~ if(jsonResponse.contains("VEHICLE_ALARM")){
				//~ status_msg.geo_alarm = jsonResponse["VEHICLE_ALARM"];
			//~ } 
		//~ }
	}	
}

void MQTTError::MQTTScooterErrorCallback(const std_msgs::String& msg)
{
	error_msg.raw_msg = msg.data;
	error_msg.mqtt_topic = vehicle_topic;
	error_msg.qos = 0;
	mqtt_publishers_pub.publish(error_msg);
}
