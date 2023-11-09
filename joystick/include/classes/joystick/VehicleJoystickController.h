#include <ros/package.h>
#include <ros/ros.h>
#include "libraries/json.hpp"

#include <pwd.h>

#include <modules/autopilot_state_msg.h>
#include <modules/bt_device_state_msg.h>
#include <modules/containers_msg.h>
#include <modules/enable_autopilot.h>
#include <modules/joystick_msg.h>
#include <modules/mac_service.h>
#include <modules/mqtt_publishers_msg.h>
#include <modules/speaker_msg.h>
#include <modules/state_msg.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include "classes/EstadoTeleops.h"
#include "classes/joystick/JoystickWireless.h"

using namespace std;
using json = nlohmann::json;

class JoystickController {
 public:
  JoystickController(ros::NodeHandle nh_priv);
  ~JoystickController(){};

  int status;
  bool mac_update{false}, pair_req{false}, reload_device{false};
  void readData();
  void initialize();
  void publishData();
  void initPairing();
  void requestHeartbeat();

 private:
  Joystick joy;

  string node_name, joystick_mac{""}, joystick_mac_path{""}, bt_cmd{""},
      joystick_pkg_path{""}, cmd_audio{""}, pairing_script_path{""};

  int linear_speed{0}, read_err_counter{0};
  const char* homedir;
  const char* mqtt_container_topic = "container_instruction";
  bool connected{false}, prev_connected{false}, pump_e_stop{false},
      bt_audio{false}, safe_lock{true};

  ros::Publisher joystickPub, speakerPub, containerPub, autopilotPub,
      joy_status_pub;
  ros::Publisher mqtt_container_pub;
  ros::ServiceServer joystick_mac_srv, pairing_srv, enable_autopilot_srv;
  ros::ServiceClient heartbeat_srv_client;
  ros::NodeHandle nh;
  ros::Subscriber statusSub;
  ros::Timer joystick_conn;

  modules::state_msg statusMsg;
  modules::autopilot_state_msg autopilotStateMsg;
  modules::speaker_msg speakerMsg;
  modules::joystick_msg musculos{};
  modules::containers_msg containerMsg;
  modules::bt_device_state_msg joystick_state;
  modules::mqtt_publishers_msg mqtt_container_msg;
  json mqtt_container_data;

  void StatusCallback(const modules::state_msg msg);
  void ConnectionWatcher();
  bool MacService(modules::mac_service::Request& request,
                  modules::mac_service::Response& response);
  bool PairingService(std_srvs::SetBool::Request& request,
                      std_srvs::SetBool::Response& response);
  bool EnableAutoPilotCallback(modules::enable_autopilot::Request& request,
                               modules::enable_autopilot::Response& response);
  void SetEnableAutoPilot(bool enable);

  string exec(const char* cmd);
};

JoystickController::JoystickController(ros::NodeHandle nh_priv) {
  node_name = ros::this_node::getName();
  int log_level;
  nh_priv.param("log_level", log_level, 0);
  nh_priv.param("pump_e_stop", pump_e_stop, false);
  nh_priv.param<string>("joystick_mac_path", joystick_mac_path,
                        "/braintemp/joy_mac.txt");
  ros::console::levels::Level console_level;
  console_level = (ros::console::levels::Level)log_level;
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  statusSub = nh.subscribe("status_topic", 10,
                           &JoystickController::StatusCallback, this);
  speakerPub = nh.advertise<modules::speaker_msg>("speaker_topic", 10);
  joystickPub = nh.advertise<modules::joystick_msg>("joystick_topic", 1);
  containerPub = nh.advertise<modules::containers_msg>("containers_topic", 2);
  autopilotPub =
      nh.advertise<modules::autopilot_state_msg>("autopilot_status_topic", 1);
  joy_status_pub = nh.advertise<modules::bt_device_state_msg>(
      "joystick_status_topic", 1, true);
  mqtt_container_pub =
      nh.advertise<modules::mqtt_publishers_msg>("mqtt_publishers", 1);

  joystick_mac_srv = nh.advertiseService("joystick_mac_service",
                                         &JoystickController::MacService, this);
  pairing_srv = nh.advertiseService("joystick_pairing_service",
                                    &JoystickController::PairingService, this);
  enable_autopilot_srv = nh.advertiseService(
      "followme_autopilot", &JoystickController::EnableAutoPilotCallback, this);
  heartbeat_srv_client = nh.serviceClient<std_srvs::Trigger>("heartbeat");
  joystick_conn =
      nh.createTimer(ros::Duration(1.0),
                     std::bind(&JoystickController::ConnectionWatcher, this));

  joystick_pkg_path = ros::package::getPath("joystick");
  if ((homedir = getenv("HOME")) == NULL) {
    homedir = getpwuid(getuid())->pw_dir;
  }

  joystick_mac_path.insert(0, homedir);
}

void JoystickController::initialize() {
  // read the joystick mac
  ifstream infile(joystick_mac_path.c_str());
  if (infile.good()) {
    getline(infile, joystick_mac);
    ROS_INFO("Joystick mac: %s", joystick_mac.c_str());
  }
  infile.close();

  connected = prev_connected = false;
  joystick_state.mac = joystick_mac;
  joystick_state.connected = connected;
  joy_status_pub.publish(joystick_state);
  // returns until finds the mac file
  if (joystick_mac.empty()) {
    ROS_ERROR_STREAM(node_name << " -- Error reading joystick mac");
    return;
  }

  // cmd to request the joystich connection status
  bt_cmd = "hcitool con | grep " + joystick_mac;
  ros::Duration(1).sleep();

  joy.set_device();
  if (joy.fd > 0) joy.close_fd();

  while (!connected || joy.open_fd() < 0) {
    ROS_INFO_STREAM_THROTTLE(30, "Waiting for joystick connection...");
    ros::Duration(3).sleep();
    ros::spinOnce();

    if (mac_update || pair_req) return;
  }

  musculos.DrivingMode = 2;
  ros::Duration(1).sleep();
}

void JoystickController::initPairing() {
  int nRet = system(pairing_script_path.c_str());
  if (nRet) {
    ROS_ERROR("%s -- Error in pairing script", node_name.c_str());
  } else {
    ROS_INFO("%s -- Pairing script succesfully executed", node_name.c_str());
  }
  pair_req = false;
  mac_update = false;
}

void JoystickController::requestHeartbeat() {
  std_srvs::Trigger trigger_srv;
  heartbeat_srv_client.call(trigger_srv);
}

void JoystickController::StatusCallback(const modules::state_msg msg) {
  ROS_DEBUG_STREAM(node_name << " --- Status received: " << msg.status);
  status = msg.status;
  if (status == EstadoScooter::JOYSTICK) {
    linear_speed = 0;
    musculos.Speed = 0;
    musculos.AngularRate = 0;
  }
}

bool JoystickController::PairingService(std_srvs::SetBool::Request& request,
                                        std_srvs::SetBool::Response& response) {
  ROS_DEBUG_STREAM(node_name << " --- Pairing service");
  response.success = true;
  if (request.data) {
    pairing_script_path = joystick_pkg_path + "/scripts/bt_con.py -r";
  } else {
    /* Just remove previous paired devices*/
    pairing_script_path = joystick_pkg_path + "/scripts/bt_con.py -r -e";
  }
  pair_req = true;

  return response.success;
}

bool JoystickController::MacService(modules::mac_service::Request& request,
                                    modules::mac_service::Response& response) {
  ROS_DEBUG_STREAM("Joystick mac service");

  response.success = false;
  if (!request.mac_to_save.empty()) {
    ROS_DEBUG("Joystick mac update: %s", request.mac_to_save.c_str());

    try {  // write the joystick mac
      ofstream outfile(joystick_mac_path.c_str());
      if (outfile) {
        outfile << request.mac_to_save;
        joystick_mac = request.mac_to_save;
        response.success = true;
      }
      outfile.close();
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM(node_name << "-- Error saving mac" << e.what());
    }
    if (response.success) {
      pairing_script_path =
          joystick_pkg_path + "/scripts/bt_con.py -r -m " + joystick_mac;
      mac_update = true;
    }
  }

  return response.success;
}

bool JoystickController::EnableAutoPilotCallback(
    modules::enable_autopilot::Request& request,
    modules::enable_autopilot::Response& response) {
  SetEnableAutoPilot(request.autopilot_enable);
  response.success = true;
  return true;
}

void JoystickController::SetEnableAutoPilot(bool enable) {
  if (musculos.AutoPilot == enable) return;

  musculos.AutoPilot = enable;
  speakerMsg.mp3Id = musculos.AutoPilot ? "autopiloton" : "autopilotoff";
  speakerPub.publish(speakerMsg);
  linear_speed = 0;
  musculos.Speed = 0;
  musculos.AngularRate = 0.0;

  autopilotStateMsg.enable = musculos.AutoPilot;
  autopilotPub.publish(autopilotStateMsg);
}

void JoystickController::ConnectionWatcher() {
  // returns until finds the mac file
  if (joystick_mac.empty()) {
    return;
  }

  prev_connected = connected;
  try {
    connected = !exec(bt_cmd.c_str()).empty();
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM(node_name << "-- Error executing command" << e.what());
  }

  if (prev_connected ^ connected) {
    joystick_state.connected = connected;
    joy_status_pub.publish(joystick_state);
    if (connected) {
      ROS_INFO_STREAM(node_name << "--- joystick connected");
      // TODO function to change status (audio)
      speakerMsg.mp3Id = "105";
      speakerPub.publish(speakerMsg);
    } else {
      ROS_WARN_STREAM(node_name << "--- joystick disconnected");
      linear_speed = 0;
      musculos.Speed = 0;
      musculos.AngularRate = 0;
    }
  }
}

void JoystickController::readData() {
  while (true) {
    if (joystick_mac.empty() || !connected || reload_device) {
      ros::Duration(3).sleep();
      continue;
    }

    if (joy.read_event(&joy.event) == 0) {
      joy.update_event();
      switch (joy.type) {
        case JS_EVENT_BUTTON:
          // ROS_DEBUG_STREAM("Button id: " << joy.number);
          if (joy.value) {
            switch (joy.number) {
              case A:
                // ROS_DEBUG_STREAM(" A");
                // ROS_DEBUG_STREAM("Button event: A - Stop");
                linear_speed = 0;
                musculos.AngularRate = 0;
                SetEnableAutoPilot(false);
                break;

              case X:
                // ROS_DEBUG_STREAM(" X");
                // ROS_DEBUG_STREAM("Button event: X - I'm a smart store. You
                // can buy ...");
                speakerMsg.mp3Id = "137";
                speakerPub.publish(speakerMsg);
                break;

              case Y:
                // ROS_DEBUG_STREAM(" Y");
                // ROS_DEBUG_STREAM("Button event: Y - I'm sorry, this box is
                // out of stock ...");
                speakerMsg.mp3Id = "134";
                speakerPub.publish(speakerMsg);
                break;

              case B:
                // ROS_DEBUG_STREAM(" B");
                // ROS_DEBUG_STREAM("Button event: B - You can use Apple Pay,
                // Google Pay ...");
                speakerMsg.mp3Id = "138";
                speakerPub.publish(speakerMsg);
                break;

              case LINES:
                // ROS_DEBUG_STREAM("LINES 1");
                SetEnableAutoPilot(!(musculos.AutoPilot));
                break;

              case SQUARES:
                // ROS_DEBUG_STREAM("Cambiando salida de audio");
                // if (bt_audio) {
                //   cmd_audio =
                //       "pacmd set-default-sink "
                //       "alsa_output.usb-Burr-Brown_from_TI_USB_audio_CODEC-00."
                //       "analog-stereo";
                //   bt_audio = false;
                // } else {
                //   cmd_audio =
                //       "pacmd set-default-sink "
                //       "bluez_sink.28_FA_19_86_BF_0C.a2dp_sink";
                //   bt_audio = true;
                // }
                // system(cmd_audio.c_str());
                break;

              default:
                // ROS_DEBUG_STREAM("NONE   " << joy.number);
                break;
            }
          }
          break;

        case JS_EVENT_AXIS:
          // ROS_DEBUG_STREAM("Axis id: " << joy.axis << " | Value: " <<
          // joy.value);
          switch (joy.axis) {
            case CROSS_X:
              if (joy.value != 0) {
                if (joy.value > 0) {
                  containerMsg.id = "2";
                  ROS_DEBUG_STREAM("Axis event: Cruz X - Contenedor 2");
                }
                if (joy.value < 0) {
                  containerMsg.id = "1";
                  ROS_DEBUG_STREAM("Axis event: Cruz X - Contenedor 1");
                }
                containerMsg.open = true;
                containerPub.publish(containerMsg);
                speakerMsg.mp3Id = "144";
                speakerPub.publish(speakerMsg);

                /* Containers V2 */
                try {
                  mqtt_container_data["command"] = "open";
                  mqtt_container_data["id"] = containerMsg.id;
                  mqtt_container_msg.raw_msg = mqtt_container_data.dump();
                  mqtt_container_msg.mqtt_topic = mqtt_container_topic;
                  mqtt_container_msg.qos = 0;
                  mqtt_container_pub.publish(mqtt_container_msg);
                } catch (const exception &e) {
                  ROS_ERROR_STREAM(e.what());
                }
              }
              break;

            case CROSS_Y:
              if (joy.value != 0) {
                if (joy.value < 0 and linear_speed < 100) {
                  linear_speed += 20;
                  // ROS_DEBUG_STREAM("Axis event: Cruz Y - Incrementar
                  // velocidad");
                }
                if (joy.value > 0 and linear_speed > -100) {
                  linear_speed -= 20;
                  // ROS_DEBUG_STREAM("Axis event: Cruz Y - Decrementar
                  // velocidad");
                }
              }
              break;

            case JS_LEFT_X:
              musculos.AngularRate = (joy.value / 32767.0) * (14);
              // ROS_DEBUG_STREAM("Axis event: Joystick izq - Giro tanque
              // izq.");
              break;

            case JS_RIGHT_X:
              musculos.AngularRate = (joy.value / 32767.0) * (40);
              // ROS_DEBUG_STREAM("Axis event: Joystick der. - Giro tanque
              // der.");
              break;
            case RT:
              if (joy.value > 0)
                safe_lock = false;
              else
                safe_lock = true;
              break;
          }
          break;

        default:
          break;
      }
      musculos.Speed = linear_speed;
    } else {
      ROS_WARN_STREAM_THROTTLE(
          1,
          node_name << "--- No event available. Joystick input disconnected.");
      musculos.Speed = 0;
      musculos.AngularRate = 0;
      read_err_counter++;
      if (read_err_counter > 50) {
        ROS_WARN_STREAM(node_name << " -- Reloading input device");
        reload_device = true;
      }
    }
    ros::Duration(0.01).sleep();  // 100 hz read
  }
}

void JoystickController::publishData() {
  if (pump_e_stop) {
    if (safe_lock) {
      ROS_DEBUG_STREAM_THROTTLE(
          1, node_name << "--- Press & hold RT to enable motion");
      linear_speed = 0;
      musculos.Speed = 0;
      musculos.AngularRate = 0;
    }
  }
  joystickPub.publish(musculos);
  ROS_DEBUG_STREAM_THROTTLE(1, "Speed: " << musculos.Speed << " AngulaRate: "
                                         << musculos.AngularRate);
}

string JoystickController::exec(const char* cmd) {
  char buffer[128];
  std::string result = "";
  FILE* pipe = popen(cmd, "r");
  if (!pipe) throw std::runtime_error("popen() failed!");
  try {
    while (fgets(buffer, sizeof buffer, pipe) != NULL) {
      result += buffer;
    }
  } catch (...) {
    pclose(pipe);
    throw;
  }
  pclose(pipe);
  // ROS_DEBUG("Salida de cmd: %s", result.c_str());
  return result;
}
