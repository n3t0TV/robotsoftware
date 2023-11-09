#include <mosquittopp.h>
#include <ros/ros.h>
#include <string.h>
#include "libraries/json.hpp"

#include <modules/bt_device_state_msg.h>
#include <modules/git_service.h>
#include <modules/imei_service.h>
#include <modules/mqtt_publishers_msg.h>
#include <modules/provider_service.h>
#include <modules/sensor_msg.h>
#include <modules/state_msg.h>
#include <std_srvs/Trigger.h>

using namespace std;
using json = nlohmann::json;

class MQTTHeartbeat {
 public:
  MQTTHeartbeat(ros::NodeHandle nh_priv);
  ~MQTTHeartbeat(){};
  void PublishHeartbeat();
  int vehicle_status;

 private:
  string heartbeat_base_topic, node_name, commit, branch, sw_version;
  bool running_source{false};

  ros::NodeHandle nh;
  ros::Publisher mqtt_heartbeat_pub;
  ros::Subscriber subSensors, subStatus, sub_joystick_status;
  ros::ServiceClient clientImei, clientProvider, clientGit, version_client;
  ros::ServiceServer heartbeat_srv;
  ros::Timer heartbeatTimer;

  modules::sensor_msg sensorMessage;
  modules::bt_device_state_msg joystick_state;
  modules::mqtt_publishers_msg mqtt_heartbeat_msg;

  void SensorsCallback(const modules::sensor_msg &msg);
  void JoystickCallback(const modules::bt_device_state_msg &msg);
  void MQTTStatusCallback(const modules::state_msg &status_msg);
  bool HeartbeatCallback(std_srvs::Trigger::Request &request,
                         std_srvs::Trigger::Response &response);
};

MQTTHeartbeat::MQTTHeartbeat(ros::NodeHandle nh_priv) {
  node_name = ros::this_node::getName();
  int log_level;
  nh_priv.param("log_level", log_level, 0);
  ros::console::levels::Level console_level;
  console_level = (ros::console::levels::Level)log_level;
  ROS_INFO_STREAM(node_name << " --- Level: " << console_level);
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  nh.param("running_source", running_source, false);
  nh_priv.param<string>("heartbeat_base_topic", heartbeat_base_topic,
                        "heartbeat/");

  subSensors =
      nh.subscribe("sensor_topic", 1, &MQTTHeartbeat::SensorsCallback, this);
  subStatus =
      nh.subscribe("status_topic", 2, &MQTTHeartbeat::MQTTStatusCallback, this);
  sub_joystick_status = nh.subscribe("joystick_status_topic", 2,
                                     &MQTTHeartbeat::JoystickCallback, this);

  mqtt_heartbeat_pub =
      nh.advertise<modules::mqtt_publishers_msg>("mqtt_publishers", 1);
  heartbeatTimer = nh.createTimer(
      ros::Duration(30), std::bind(&MQTTHeartbeat::PublishHeartbeat, this));

  heartbeat_srv =
      nh.advertiseService("heartbeat", &MQTTHeartbeat::HeartbeatCallback, this);
  ros::service::waitForService("imei_service");
  clientImei = nh.serviceClient<modules::imei_service>("imei_service");
  modules::imei_service imei_srv;
  if (!clientImei.call(imei_srv)) {
    ROS_ERROR_STREAM(node_name << " --- IMEI service not working.");
    sensorMessage.Imei = "";
  } else {
    sensorMessage.Imei = imei_srv.response.imei;
  }

  clientGit = nh.serviceClient<modules::git_service>("git_service");
  modules::git_service git_srv;
  if (!clientGit.call(git_srv)) {
    ROS_ERROR_STREAM(node_name << " --- git service not working.");
    branch = "UNKNOWN";
    commit = "UNKNOWN";
  } else {
    branch = git_srv.response.branch;
    commit = git_srv.response.commit;
  }

  clientProvider =
      nh.serviceClient<modules::provider_service>("provider_service");

  ros::service::waitForService("version_service");
  version_client = nh.serviceClient<std_srvs::Trigger>("version_service");
  std_srvs::Trigger version_msg;
  if (!version_client.call(version_msg)) {
    ROS_WARN_STREAM(node_name << " --- SW version service not working.");
    sw_version = "xxx";
  } else {
    sw_version = version_msg.response.message;
  }
}

void MQTTHeartbeat::MQTTStatusCallback(const modules::state_msg &status_msg) {
  vehicle_status = status_msg.status;

  /* Cambia el rate de publicacion de heatbeat segun el estado*/
  if (vehicle_status == 5 || vehicle_status == 6) {
    heartbeatTimer.stop();
    heartbeatTimer.setPeriod(ros::Duration(5));
    heartbeatTimer.start();
  } else {
    heartbeatTimer.stop();
    heartbeatTimer.setPeriod(ros::Duration(30));
    heartbeatTimer.start();
  }
}

void MQTTHeartbeat::SensorsCallback(const modules::sensor_msg &msg) {
  sensorMessage = msg;
}

void MQTTHeartbeat::JoystickCallback(const modules::bt_device_state_msg &msg) {
  joystick_state = msg;
}

bool MQTTHeartbeat::HeartbeatCallback(std_srvs::Trigger::Request &request,
                                      std_srvs::Trigger::Response &response) {
  ROS_DEBUG_STREAM("Heartbeat service triggered");
  PublishHeartbeat();

  response.success = true;
  return response.success;
}

void MQTTHeartbeat::PublishHeartbeat() {
  json j;

  j["VEHICLE_STATUS"] = vehicle_status;
  j["BRAIN_BRANCH"] = branch;
  j["BRAIN_COMMIT"] = commit;
  j["BATTERY"] = sensorMessage.batteryLevel;
  j["VERSION_FIRMWARE"] = sensorMessage.version;
  j["VERSION_BRAIN"] = sw_version;
  j["RUNNING_SOURCE"] = running_source;

  j["UID_PHYSICAL"] = sensorMessage.Imei;
  j["GPS_LAT"] = sensorMessage.Latitude;
  j["GPS_LON"] = sensorMessage.Longitude;
  j["ALTITUD"] = sensorMessage.Altitude;
  j["ACCURACY"] = sensorMessage.Accuracy;
  j["SATELLITES"] = sensorMessage.Satellites;
  j["SIM"] = sensorMessage.Sim;
  j["RSSI"] = sensorMessage.rssi;
  j["GPS_FIXED"] = sensorMessage.validGPS;
  j["TECHNOLOGY"] = sensorMessage.accessTechnology;
  j["BAND"] = sensorMessage.band;
  j["MODULE_CONNECTED"] = sensorMessage.ModuleConnected;
  j["AUDIO_VOLUME"] = sensorMessage.speaker_volume;
  j["JOYSTICK"] = json::object({{"MAC", joystick_state.mac},
                                {"CONNECTED", (bool)joystick_state.connected}});

  modules::provider_service provider_srv;
  if (!clientProvider.call(provider_srv)) {
    ROS_ERROR_STREAM(node_name << " ---  provider service not working.");
    j["PROVIDER"] = "ERROR";
  } else {
    j["PROVIDER"] = provider_srv.response.provider;
  }

  try {
    ROS_DEBUG_STREAM(j.dump());
    mqtt_heartbeat_msg.raw_msg = j.dump();
    mqtt_heartbeat_msg.mqtt_topic = heartbeat_base_topic;
    mqtt_heartbeat_msg.qos = 0;
    mqtt_heartbeat_pub.publish(mqtt_heartbeat_msg);
  } catch (const exception &e) {
    ROS_ERROR_STREAM(e.what());
  }
}
