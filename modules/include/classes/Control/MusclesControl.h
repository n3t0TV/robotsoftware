#include <ros/ros.h>
#include <string.h>
#include <ros/console.h>
#include "libraries/exceptions/Exceptions.h"
#include "libraries/json.hpp"
#include "../Structs/EstadoTeleops.h"
#include <modules/joystick_msg.h>
#include <modules/motion_ctrl_msg.h>
#include <modules/misc_ctrl_msg.h>
#include <modules/state_msg.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Bool.h>

#define MIN_CTRL_RATE 2

using json = nlohmann::json;

class MusclesControl
{
public:
  MusclesControl(ros::NodeHandle);
  ~MusclesControl(){};

  void StateMachine();

  int status;

private:
  ros::NodeHandle nh;
  ros::Subscriber subStatus;
  ros::Subscriber subTeleopsUDP, subTeleopsQR, subTeleopsMisc, subJoystick, subVending;
  ros::Publisher pubControl, pubTeleopException, pubTeleopRunning;
  ros::Duration lastMessageElapsedTime;
  ros::Time lastMessageTimeStamp;
  ros::Time emergencyStopStartTime;
  ros::Timer exceptionTimer, rateControlTimer, stateMachineTimer;
  modules::joystick_msg musculos;
  modules::motion_ctrl_msg motion_udp, motion_qr;
  std_msgs::Byte err_msg;
  std_msgs::Bool teleop_msg;
  string node_name;
  bool lastEmergencyStop;
  bool brainEmergencyStopStarted;
  bool udp_rate_ok{true}, qr_rate_ok{true}, rate_ok{true};
  uint udp_count{0}, qr_count{0}, rate_count{0};
  uint64_t lastControlTimestamp{0};
  uint lastControlMsg{0};

  void UDPTeleopsCallback(const modules::motion_ctrl_msg msg);
  void QRTeleopsCallback(const modules::motion_ctrl_msg msg);
  void MiscTeleopsCallback(const modules::misc_ctrl_msg msg);
  void JoystickCallback(const ros::MessageEvent<modules::joystick_msg const>& event);
  void VendingCallback(const modules::misc_ctrl_msg msg);
  void StatusCallback(const modules::state_msg msg);
  void ToJoystickMsg(const modules::motion_ctrl_msg msg, string debug);
  void PublishException();
  void RateControl();
};

MusclesControl::MusclesControl(ros::NodeHandle nh_priv)
{
  node_name = ros::this_node::getName();
  int log_level;
  nh_priv.param("log_level", log_level, 0);
  ros::console::levels::Level console_level;
  console_level = (ros::console::levels::Level)log_level;
  //~ ROS_INFO_STREAM(node_name << " --- Level: " << console_level);
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  subStatus = nh.subscribe("status_topic", 10, &MusclesControl::StatusCallback, this);
  subJoystick = nh.subscribe("joystick_topic", 2, &MusclesControl::JoystickCallback, this);
  subVending = nh.subscribe("vending_topic", 2, &MusclesControl::VendingCallback, this);
  subTeleopsUDP = nh.subscribe("teleops_topic/udp", 1, &MusclesControl::UDPTeleopsCallback, this);
  subTeleopsQR = nh.subscribe("teleops_topic/qr", 1, &MusclesControl::QRTeleopsCallback, this);
  subTeleopsMisc = nh.subscribe("teleops_topic/misc", 1, &MusclesControl::MiscTeleopsCallback, this);
  pubControl = nh.advertise<modules::joystick_msg>("teleops_topic", 5);
  pubTeleopException = nh.advertise<std_msgs::Byte>("exception_topic/teleop", 2);
  pubTeleopRunning = nh.advertise<std_msgs::Bool>("teleop_running", 2, true);
  exceptionTimer = nh.createTimer(ros::Duration(1), std::bind(&MusclesControl::PublishException, this));
  rateControlTimer = nh.createTimer(ros::Duration(1), std::bind(&MusclesControl::RateControl, this));
  stateMachineTimer = nh.createTimer(ros::Duration(0.1), std::bind(&MusclesControl::StateMachine, this));

  status = EstadoScooter::PARKED;
  musculos.Speed = 0;
  musculos.Angle = 0;
  musculos.TurningLights = 0;
  musculos.Illumination = 0;
  musculos.Brake = 1;
  musculos.Reset = 0;
  musculos.EmergencyStop = false;
  musculos.EnabledEmergencyStop = true;
  musculos.TimeThreshold_ms = 2000;
  musculos.LeftWheel = 0;
  musculos.RightWheel = 0;
  musculos.DrivingMode = 2;
  musculos.OrientationSetpoint = 0;
  musculos.MovCamera = 0;
  musculos.cameraExp = 20;
  musculos.videoMode = 1;
  musculos.UDPTimeStamp = 0;
  musculos.QRTimeStamp = 0;
  musculos.AutoPilot = 0;
  lastEmergencyStop = false;
  brainEmergencyStopStarted = false;
}

void MusclesControl::UDPTeleopsCallback(const modules::motion_ctrl_msg msg)
{
  if (status != EstadoScooter::TELEOP)
    return;

  motion_udp = msg;
  //motion_udp.TimeStamp = ros::Time::now().toNSec();
  udp_count++;
}

void MusclesControl::QRTeleopsCallback(const modules::motion_ctrl_msg msg)
{
  if (status != EstadoScooter::TELEOP)
    return;

  motion_qr = msg;
  //motion_qr.TimeStamp = ros::Time::now().toNSec();
  qr_count++;
}

void MusclesControl::RateControl()
{
  if (status != EstadoScooter::TELEOP)
    return;

  ROS_DEBUG_STREAM(node_name << " --- control_rate: " << rate_count << ", udp_rate: " << udp_count << ", qr_rate: " << qr_count);
  udp_rate_ok = udp_count >= MIN_CTRL_RATE;
  qr_rate_ok = qr_count >= MIN_CTRL_RATE;
  rate_ok = rate_count >= MIN_CTRL_RATE;

  udp_count = 0;
  qr_count = 0;
  rate_count = 0;
}

void MusclesControl::MiscTeleopsCallback(const modules::misc_ctrl_msg msg)
{
  if (status != EstadoScooter::TELEOP)
    return;

  musculos.Illumination = msg.Illumination;
  musculos.TurningLights = msg.TurningLights;
  musculos.Reset = msg.Reset;
  musculos.DrivingMode = msg.DrivingMode;
  musculos.MovCamera = msg.MovCamera;
  musculos.cameraExp = msg.cameraExp;
  musculos.AutoPilot = msg.AutoPilot;
}

void MusclesControl::JoystickCallback(const ros::MessageEvent<modules::joystick_msg const>& event)
{
  if (status != EstadoScooter::JOYSTICK)
    return;

  const std::string& publisher_name = event.getPublisherName();
  const modules::joystick_msgConstPtr& msg = event.getMessage();
  if (publisher_name.compare("/joystick_node") == 0) {
    musculos.DrivingMode = msg->DrivingMode;
    musculos.Speed = msg->Speed;
    musculos.AngularRate = msg->AngularRate;
    musculos.Brake = msg->Brake;
    musculos.AutoPilot = msg->AutoPilot;
  } else {
    musculos.cameraExp = msg->cameraExp;
    musculos.MovCamera = msg->MovCamera;
  }
}

void MusclesControl::VendingCallback(const modules::misc_ctrl_msg msg)
{
  if (status != EstadoScooter::VENDING)
    return;

  musculos.MovCamera = msg.MovCamera;
}

void MusclesControl::StatusCallback(const modules::state_msg msg)
{
  ROS_DEBUG_STREAM(node_name << " --- Status: " << msg.status);
  status = msg.status;

  if (status == EstadoScooter::TELEOP || status == EstadoScooter::JOYSTICK)
    teleop_msg.data = true;
  else
    teleop_msg.data = false;

  pubTeleopRunning.publish(teleop_msg);
}

void MusclesControl::StateMachine()
{
  switch (status)
  {
    case EstadoScooter::COMMAND:
      //~ ROS_DEBUG_STREAM(node_name << " --- COMMAND TEST");
      break;
    case EstadoScooter::JOYSTICK:
      ROS_DEBUG_STREAM(node_name << " --- JOYSTICK");
      //~ return;
      break;
    case EstadoScooter::VENDING:
      musculos.Speed = 0;
      musculos.TurningLights = 0;
      musculos.Brake = 1;
      musculos.DrivingMode = 2;
      break;

    case EstadoScooter::PARKED:
      ROS_DEBUG_STREAM(node_name << " --- IDLE");
      musculos.Speed = 0;
      musculos.Angle = 0;
      musculos.TurningLights = 0;
      musculos.Illumination = 0;
      musculos.Brake = 1;
      musculos.Reset = 0;
      musculos.LeftWheel = 0;
      musculos.RightWheel = 0;
      musculos.DrivingMode = 2;
      musculos.AngularRate = 0;
      musculos.OrientationSetpoint = 0;
      musculos.MovCamera = 0;
      break;

    case EstadoScooter::TELEOP:
      if (motion_udp.TimeStamp > lastControlTimestamp || motion_qr.TimeStamp > lastControlTimestamp)
      {
        if (motion_udp.TimeStamp > motion_qr.TimeStamp)
        {
          ToJoystickMsg(motion_udp, "UDP > timestamp");
          rate_count++;
          lastControlMsg = UDP_CHANNEL;
        }
        else if (motion_udp.TimeStamp < motion_qr.TimeStamp)
        {
          ToJoystickMsg(motion_qr, "QR > timestamp");
          rate_count++;
          lastControlMsg = QR_CHANNEL;
        }
        else
        {
          ToJoystickMsg(motion_qr, "QR == UDP timestamp");
          rate_count++;
          lastControlMsg = UDP_QR_CHANNEL;
        }
      }

      if (!rate_ok)
      {
        //Emergency stop
        ROS_WARN_STREAM("EMERGENCY STOP!");
        musculos.Speed = 0;
        musculos.TurningLights = 3;
        musculos.Illumination = 0;
        musculos.AngularRate = 0;
      }

      //metrics
      musculos.UDPTimeStamp = motion_udp.TimeStamp;
      musculos.QRTimeStamp = motion_qr.TimeStamp;
      musculos.LastControlMsg = lastControlMsg;

      break;

    default:
      ROS_DEBUG_STREAM(node_name << " --- Estado Inicial");
      musculos.Speed = 0;
      musculos.Angle = 0;
      musculos.TurningLights = 0;
      musculos.Illumination = 0;
      musculos.Brake = 1;
      musculos.Reset = 0;
      musculos.LeftWheel = 0;
      musculos.RightWheel = 0;
      musculos.AngularRate = 0;
      musculos.OrientationSetpoint = 0;
      musculos.MovCamera = 0;
  }
  pubControl.publish(musculos);
}

void MusclesControl::ToJoystickMsg(const modules::motion_ctrl_msg msg, string debug)
{
  ROS_DEBUG_STREAM("Mensaje de control: " + debug);
  musculos.Speed = msg.Speed;
  musculos.Brake = msg.Brake;
  musculos.AngularRate = msg.AngularRate;
  if (musculos.Brake) musculos.TurningLights = 4;
  lastControlTimestamp = msg.TimeStamp;
}

void MusclesControl::PublishException()
{
  if (status == EstadoScooter::TELEOP)
  {
    //if (lastMessageElapsedTime.toSec() > 2.0)
    if (!rate_ok)
      err_msg.data = FLAG_TELEOP_COM;
    else
      err_msg.data = STATUS_OK;
  }
  else
  {
    err_msg.data = STATUS_OK;
  }
  pubTeleopException.publish(err_msg);
}
