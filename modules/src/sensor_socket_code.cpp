#include <modules/imei_service.h>
#include <modules/sensor_msg.h>
#include <modules/state_msg.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <vector>

#include "libraries/json.hpp"
//#include <modules/feedback_msg.h>

#include <modules/imei_service.h>
#include <modules/sensor_msg.h>
#include <modules/sensorsocket_status_msg.h>
#include <modules/socket_state_msg.h>
#include <modules/state_msg.h>
#include <modules/vp_whereiam_msg.h>
#include <std_msgs/Int32.h>

#include "classes/UDP/TeleopsSensor.h"

using namespace std;
using json = nlohmann::json;

TeleopsSensor teleopsSensor;
ros::Publisher pubSocketStatus;

void requestImei(ros::NodeHandle nh) {
  // ros::service::waitForService("imei_service");
  ros::ServiceClient imeiClient =
      nh.serviceClient<modules::imei_service>("imei_service");
  modules::imei_service::Request req;
  modules::imei_service::Response res;
  if (imeiClient.call(req, res))
    teleopsSensor.imei = res.imei;
  else
    teleopsSensor.imei = "";
}

void StatusCallback(const modules::state_msg msg) {
  /*ROS_INFO("Teleop sensor state message");
  ROS_INFO_STREAM(msg.status);
  ROS_INFO_STREAM(msg.servidor);
  ROS_INFO_STREAM(msg.control);
  ROS_INFO_STREAM(msg.video);
  ROS_INFO_STREAM(msg.sensores);*/

  // Update server and sensor port
  teleopsSensor.teleops.estatus = msg.status;
  teleopsSensor.teleops.servidor = msg.servidor;
  teleopsSensor.teleops.sensores = msg.sensores;

  if (msg.status == EstadoScooter::TELEOP) {
    teleopsSensor.desiredSocketStatus = 1;
  } else {
    teleopsSensor.desiredSocketStatus = 0;
  }
}

void SensorCallback(const modules::sensor_msg msg)
{
	teleopsSensor.picStatus.batteryLevel=msg.batteryLevel;
	teleopsSensor.picStatus.batteryVoltage=msg.batteryVoltage;
	teleopsSensor.picStatus.currentSpeed=msg.currentSpeed;
	teleopsSensor.picStatus.currentAngularRate=msg.angular_rate_curr;
	teleopsSensor.picStatus.yaw=msg.yaw;
	teleopsSensor.picStatus.pitch=msg.pitch;
	teleopsSensor.picStatus.roll=msg.roll;
	teleopsSensor.picStatus.desiredSpeed=msg.desiredSpeed;
	teleopsSensor.picStatus.desiredAngularRate=msg.angular_rate_sp;
	teleopsSensor.picStatus.currentSpeedRight=msg.currentSpeedRight;
	teleopsSensor.picStatus.desiredSpeedRight=msg.desiredSpeedRight;
	teleopsSensor.picStatus.currentSpeedLeft=msg.currentSpeedLeft;
	teleopsSensor.picStatus.desiredSpeedLeft=msg.desiredSpeedLeft;
	teleopsSensor.picStatus.udpTimeStamp=msg.UDPTimeStamp;
	teleopsSensor.picStatus.qrTimeStamp=msg.QRTimeStamp;
	teleopsSensor.picStatus.lastControlMsg=msg.LastControlMsg;
	
	teleopsSensor.gpsData.latitud=msg.Latitude;
	teleopsSensor.gpsData.longitud=msg.Longitude;
	teleopsSensor.gpsData.altitud=msg.Altitude;
	teleopsSensor.gpsData.rssi=msg.rssi;
	teleopsSensor.gpsData.sim=msg.Sim;
	teleopsSensor.gpsData.validGPS = msg.validGPS;
	teleopsSensor.gpsData.satellites = msg.Satellites;

	teleopsSensor.vpData.auto_pilot = msg.AutoPilot;
	teleopsSensor.vpData.inferring = msg.Inferring;
}

void ExceptionCallback(const std_msgs::Int32 msg) {
  teleopsSensor.picStatus.errorVector = msg.data;
}

void WhereIAmCallback(const modules::vp_whereiam_msg msg) {
  teleopsSensor.vpData.whereiam_cls_id = msg.cls_id;
  teleopsSensor.vpData.whereiam_cls_conf = msg.cls_confidence;
  teleopsSensor.vpData.lpt = msg.lpt;
  teleopsSensor.vpData.rpt = msg.rpt;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sensorsocketNode");
  ROS_INFO_STREAM("sensor_socket_node");
  ros::NodeHandle nh;
  string node_name = ros::this_node::getName();
  int log_level;
  nh.param("log_level", log_level, 0);
  ros::console::levels::Level console_level;
  console_level = (ros::console::levels::Level)log_level;
  ROS_INFO_STREAM(node_name << " --- Level: " << console_level);
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::Rate rate(20);

  requestImei(nh);

  ros::Subscriber subSensor = nh.subscribe("sensor_topic", 1, SensorCallback);
  ros::Subscriber subStatus = nh.subscribe("status_topic", 10, StatusCallback);
  ros::Subscriber subExceptions =
      nh.subscribe("exception_topic", 5, ExceptionCallback);
  ros::Subscriber subWhereIam =
      nh.subscribe("video_processing/whereiam", 1, WhereIAmCallback);

  // teleopsSensor.Initialize();

  while (ros::ok()) {
    if (teleopsSensor.imei.empty()) {
      requestImei(nh);
    }
    teleopsSensor.stateMachine();
    ros::spinOnce();
    // publishSocketStatus(pubSocketStatus);
    rate.sleep();
  }
  return 0;
}
