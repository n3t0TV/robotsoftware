#include <ros/console.h>
#include <ros/ros.h>

#include <iostream>

#include "classes/structs/Container.h"

#include "libraries/exceptions/Exceptions.h"
#include "libraries/json.hpp"

#include <drivers/sensor_picmanager_msg.h>
#include <modules/mqtt_publishers_msg.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Trigger.h>

using namespace std;
using json = nlohmann::json;

class VehicleException {
 public:
  VehicleException(ros::NodeHandle);
  ~VehicleException();
  void Publish();

 private:
  string node_name;
  int error_vector;
  std_msgs::Int32 error_msg;

  FeedtainerStatus fdbck1;
  FeedtainerStatus fdbck2;

  ros::Time lastSPINodeError;
  ros::Timer timerFeedtainer;
  ros::Publisher pubError;
  ros::NodeHandle nh;
  ros::Subscriber subPicErrors, subSPIErrors, subBrainError, subTeleopError,
      subContainerError;
  ros::ServiceServer containers_reset_srv;

  void PicErrorCallback(const drivers::sensor_picmanager_msg& msg);
  void SPIErrorCallback(const std_msgs::Byte& msg);
  void BrainErrorCallback(const std_msgs::Byte& msg);
  void TeleopErrorCallback(const std_msgs::Byte& msg);
  void ContainerFeedbackWatcher(const ros::TimerEvent& event);
  void ContainerFeedbackCallback(const modules::mqtt_publishers_msg& msg);
  bool ContainerResetService(std_srvs::Trigger::Request& req,
                             std_srvs::Trigger::Response& res);

  void ResetFeedtainer();
  void ResetContainers();
  void PicUnmaskErrorMsg(const int32_t& err, const char* prefix);
};

VehicleException::VehicleException(ros::NodeHandle nh_priv) {
  node_name = ros::this_node::getName();
  ROS_DEBUG_STREAM(node_name);
  int log_level;
  nh_priv.param("log_level", log_level, 0);
  ros::console::levels::Level console_level;
  console_level = (ros::console::levels::Level)log_level;
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  error_vector = STATUS_OK;
  lastSPINodeError = ros::Time::now();
  timerFeedtainer = nh.createTimer(
      ros::Duration(2.0), &VehicleException::ContainerFeedbackWatcher, this);

  pubError = nh.advertise<std_msgs::Int32>("exception_topic", 10);
  subPicErrors = nh.subscribe("sensor_topic/picmanager", 1,
                              &VehicleException::PicErrorCallback, this);
  subSPIErrors = nh.subscribe("exception_topic/spi", 1,
                              &VehicleException::SPIErrorCallback, this);
  subBrainError = nh.subscribe("exception_topic/brain", 1,
                               &VehicleException::BrainErrorCallback, this);
  subTeleopError = nh.subscribe("exception_topic/teleop", 1,
                                &VehicleException::TeleopErrorCallback, this);
  subContainerError = nh.subscribe(
      "mqtt_publishers", 1, &VehicleException::ContainerFeedbackCallback, this);
  containers_reset_srv = nh.advertiseService(
      "containers_reset_srv", &VehicleException::ContainerResetService, this);

  ResetFeedtainer();
}

void VehicleException::PicErrorCallback(
    const drivers::sensor_picmanager_msg& msg) {
  /*Error de comunicaion SPI con pic*/
  if (msg.Version.size() <= 0) {
    ROS_WARN("SPI PIC Died!");
    error_vector |= FLAG_SPI_COM;
  } else {
    error_vector &= ~FLAG_SPI_COM;
  }

  // limpia banderas de error Kangaroo/less
  error_vector &= ~FLAG_UART_KANGAROO;
  error_vector &= ~FLAG_DRIVES;

  /*Solo esta implementado error 1 y drive error*/
  if (msg.WagonError1 != STATUS_OK) {
    /*Drive right*/
    if (msg.WagonError1 & PIC_ERR_DRV_R) {
      /* Kangaroo handler */
      if (msg.WagonDriveRightError <= PIC_ERR_DRV_S_LOST) {
        /* Just one because left & right are identicals on Kangaroo*/
        if (msg.WagonDriveRightError & PIC_ERR_DRV_CNTRL) {
          ROS_ERROR_THROTTLE(1, "Kangaroo control error! -- %d",
                             msg.WagonDriveRightError);
          error_vector |= FLAG_UART_KANGAROO;
        } else {
          ROS_WARN_THROTTLE(2, "Kangaroo error, no fatal! -- %d",
                            msg.WagonDriveRightError);
        }
      }
      /* Kangarooless handler (tortoise control) */
      if (msg.WagonDriveRightError > PIC_ERR_DRV_S_LOST) {
        PicUnmaskErrorMsg(msg.WagonDriveRightError, "Drive right");
        error_vector |= FLAG_DRIVES;
      }
    }

    /*Drive let*/
    if (msg.WagonError1 & PIC_ERR_DRV_L) {
      /* Kangarooless handler (tortoise control)*/
      if (msg.WagonDriveLeftError > PIC_ERR_DRV_S_LOST) {
        PicUnmaskErrorMsg(msg.WagonDriveLeftError, "Drive left");
        error_vector |= FLAG_DRIVES;
      }
    }
  }

  msg.WagonFailSafeMode ? error_vector |= FLAG_SAFE_MODE
                        : error_vector &= ~FLAG_SAFE_MODE;
  msg.WagonFailSafeRelease ? error_vector |= FLAG_SAFE_RELEASE
                           : error_vector &= ~FLAG_SAFE_RELEASE;
}

void VehicleException::PicUnmaskErrorMsg(const int32_t& err,
                                         const char* prefix) {
  int32_t _err = 0;
  for (size_t i = 0; i < sizeof(err) * 8; i++) {
    _err = err & (1 << i);
    switch (_err) {
      case MC_ERR_COMM:
        ROS_ERROR("%s -- Communication lost with the motion driver (%d)",
                  prefix, _err);
        break;

      case MC_ERR_CURR_DIFF:
        ROS_ERROR(
            "%s -- Motor current derivative surpassed the secure limit (%d)",
            prefix, _err);
        break;

      case MC_ERR_CURR_LIMIT:
        ROS_ERROR("%s -- Motor current surpassed the secure limit (%d)", prefix,
                  _err);
        break;

      case MC_ERR_QEB:
        ROS_ERROR("%s -- Quadrature encoder B signal lost (%d)", prefix, _err);
        break;

      case MC_ERR_QEA:
        ROS_ERROR("%s -- Quadrature encoder A signal lost (%d)", prefix, _err);
        break;
    }
  }
}

void VehicleException::SPIErrorCallback(const std_msgs::Byte& msg) {
  ros::Duration spiPeriod(
      0.5);  // da tiempo que no sea un caso aislado (node died)

  if (msg.data & SPI_ERR_NODE) {
    if (ros::Time::now() - lastSPINodeError < spiPeriod) {
      ROS_ERROR_STREAM("SPI Node Died!");
      error_vector |= FLAG_SPI_COM;
    } else {
      error_vector &= ~FLAG_SPI_COM;
    }
    lastSPINodeError = ros::Time::now();
  }
}

void VehicleException::BrainErrorCallback(const std_msgs::Byte& msg) {
  if (msg.data != STATUS_OK) {
    ROS_ERROR_STREAM("Brain Internal Error!");
    error_vector |= FLAG_BRAIN_INTRN;
  } else {
    error_vector &= ~FLAG_BRAIN_INTRN;
  }
}

void VehicleException::TeleopErrorCallback(const std_msgs::Byte& msg) {
  if (msg.data != STATUS_OK) {
    ROS_ERROR_STREAM("Teleop Communication Error!");
    error_vector |= FLAG_TELEOP_COM;
  } else {
    error_vector &= ~FLAG_TELEOP_COM;
  }
}

void VehicleException::ResetFeedtainer() {
  ROS_DEBUG_STREAM("Reseting feedback data structure...");
  fdbck1.id = "1";
  fdbck1.param_path = "/ble_node_" + fdbck1.id + "/pid";
  fdbck1.previous = time(0);
  fdbck1.status = false;
  fdbck1.connection = false;

  fdbck2.id = "2";
  fdbck2.param_path = "/ble_node_" + fdbck2.id + "/pid";
  fdbck2.previous = time(0);
  fdbck2.status = false;
  fdbck2.connection = false;
}

void VehicleException::ResetContainers() {
  string killCommand;
  int nRet{0};
  nh.param(fdbck1.param_path, fdbck1.pid, 0);
  nh.param(fdbck2.param_path, fdbck2.pid, 0);
  if (fdbck1.pid) {
    killCommand = "kill -9 " + to_string(fdbck1.pid);
    nRet = system(killCommand.c_str());
    if (nRet) {
      ROS_ERROR("Couldn't kill the ble_node_%s proccess with PID %d",
                fdbck1.id.c_str(), fdbck1.pid);
    } else {
      nh.setParam(fdbck1.param_path, 0);
    }
    ros::Duration(0.5).sleep();
  }
  if (fdbck2.pid) {
    killCommand = "kill -9 " + to_string(fdbck2.pid);
    nRet = system(killCommand.c_str());
    if (nRet) {
      ROS_ERROR("Couldn't kill the ble_node_%s proccess with PID %d",
                fdbck2.id.c_str(), fdbck2.pid);
    } else {
      nh.setParam(fdbck2.param_path, 0);
    }
    ros::Duration(0.5).sleep();
  }
  ResetFeedtainer();
}

bool VehicleException::ContainerResetService(std_srvs::Trigger::Request& req,
                                             std_srvs::Trigger::Response& res) {
  ROS_DEBUG("Containers Reset service");
  ResetContainers();
  res.success = true;
  return res.success;
}

void VehicleException::ContainerFeedbackWatcher(const ros::TimerEvent& event) {
  if (fdbck1.status or fdbck2.status) {
    time_t now = time(0);
    time_t lapse1 = now - fdbck1.previous;
    time_t lapse2 = now - fdbck2.previous;
    // ROS_DEBUG_STREAM("1 | connection: " << fdbck1.connection
    //                                     << " | lapse: " << lapse1);
    // ROS_DEBUG_STREAM("2 | connection: " << fdbck2.connection
    //                                     << " | lapse: " << lapse2);
    if ((fdbck1.connection && ((lapse1) > 10)) ||
        (fdbck2.connection && (lapse2 > 10))) {
      ROS_WARN_STREAM(node_name << " --- Kill them all >:( ");
      ResetContainers();
    }
  }
}

void VehicleException::ContainerFeedbackCallback(
    const modules::mqtt_publishers_msg& msg) {
  json message = json::parse(msg.raw_msg);
  // ROS_DEBUG_STREAM(message);
  if (message.contains("mac") && message.contains("id") &&
      message.contains("version")) {
    string id_string = message["id"];
    int id = stoi(id_string);
    switch (id) {
      case 1:
        fdbck1.previous = time(0);
        fdbck1.status = true;
        fdbck1.connection = message["connection"];
        break;

      case 2:
        fdbck2.previous = time(0);
        fdbck2.status = true;
        fdbck2.connection = message["connection"];
        break;
    }
  }
}

void VehicleException::Publish() {
  // ROS_DEBUG_STREAM("Error vector: " << error_vector);
  error_msg.data = error_vector;
  pubError.publish(error_msg);
}

VehicleException::~VehicleException() {}
