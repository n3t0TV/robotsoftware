#include <ros/master.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <mosquittopp.h>
#include <pwd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <algorithm>
#include <fstream>
#include <vector>
#include "libraries/cipher.h"
#include "libraries/json.hpp"

#include <modules/bt_macs_msg.h>
#include <modules/console_service.h>
#include <modules/containers_msg.h>
#include <modules/imei_service.h>
#include <modules/joystick_msg.h>
#include <modules/mac_service.h>
#include <modules/manager_status_msg.h>
#include <modules/misc_ctrl_msg.h>
#include <modules/mqtt_publishers_msg.h>
#include <modules/speaker_msg.h>
#include <modules/state_msg.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#define RYML_SINGLE_HDR_DEFINE_NOW
#include "libraries/ryml_all.hpp"

#define BRIDGE_PARAM                                                   \
  "\"[{factory: 'mqtt_bridge.bridge:RosToMqttBridge', msg_type: "      \
  "'{dataPackage}.msg:{dataType}',topic_from: {topicFrom}, topic_to: " \
  "~/out}]\""

#define SELL 1
#define REFILL 2
#define DELIVERY 3

using namespace std;
using json = nlohmann::json;
using namespace ros;

vector<string> split(string data, char spliter);
string replaceFirstOccurrence(string& s, const string& toReplace,
                              const string& replaceWith);

class MQTTManager : public mosqpp::mosquittopp {
 public:
  MQTTManager(ros::NodeHandle);
  ~MQTTManager();
  void publishManagerStatus();
  bool publish_status, connection_status;
  int publication_id{0}, disconnection_error{0};
  void noConnectionAudio();

 private:
  const char* kSpeakerSetVolTopicName = "speaker_set_volume";
  const char* kPaymentCardTapTopicName = "payment/card_tap_event";
  const char* kPaymentTransRespTopicName = "payment/transaction_response";
  const std::uint32_t kSpeakerSetVolTopicQSize = 1;
  const string kProdMqttHost{"vehicle.tortops.com"},
      kDevMqttHost{"dev3.vehicle.tortops.com"};

  ros::NodeHandle nh;
  ros::ServiceClient imei_service_client, mac_service_client,
      init_program_pic_client, reload_broadcaster_clnt,
      console_broadcaster_clnt, container_reset_clnt, pair_joystick_clnt,
      pair_speaker_clnt;
  ros::ServiceServer subscribe_srv;
  ros::Subscriber mqtt_publisher_sub;
  ros::Publisher manager_status_pub, status_pub, audio_pub, containers_pub,
      log_enable_pub, btmacs_pub, vending_pub, joystick_pub, mqtt_msg_pub;
  ros::Publisher speaker_set_vol_ros_pub_;
  ros::Publisher card_tap_event_pub_;
  ros::Publisher trans_resp_event_pub_;
  ros::Timer conn_stat_timer_;
  modules::state_msg status_msg;
  modules::speaker_msg audio_msg;
  modules::joystick_msg joystick_msg{};
  std_msgs::Bool log_enable_msg, btc_msg;
  string mqtt_host, mqtt_password, mqtt_id, node_name, passwd, user_password;
  bool securemqtt;
  json subscribers;
  int mqtt_port, mqtt_keepalive, macqtt;
  void connect();
  void Reconnect();
  void on_connect(int rc);
  void on_disconnect(int rc);
  void on_publish(int mid);
  void on_message(const struct mosquitto_message* message);
  void on_subscribe(int mid, int qos_count, const int* granted_qos);
  void Publishers_cb(const modules::mqtt_publishers_msg& publisher_msg);
  void ProcessInstruction(json jsonMessage);
  void ProcessOTACommand(json jsonMessage);
  void ProcessCommunication(json jsonMessage);
  void GetMonitorFeedback();
  void ReloadBroadcaster();
  const char* homedir;
  bool brainStarted{false}, bridgeStarted{false}, initBrain;
  pid_t pid, bridgePid;
  ros::master::V_TopicInfo topics;
};

MQTTManager::MQTTManager(ros::NodeHandle nh_priv) : mosquittopp() {
  connection_status = false;
  joystick_msg.MovCamera = -167;
  joystick_msg.cameraExp = 20;

  if ((homedir = getenv("HOME")) == NULL) {
    homedir = getpwuid(getuid())->pw_dir;
  }

  mosqpp::lib_init();
  node_name = ros::this_node::getName();
  int log_level;
  nh_priv.param("log_level", log_level, 0);
  nh_priv.param("initBrain", initBrain, false);
  ros::console::levels::Level console_level;
  console_level = (ros::console::levels::Level)log_level;
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  audio_pub = nh.advertise<modules::speaker_msg>("speaker_topic", 10);
  status_pub = nh.advertise<modules::state_msg>("status_topic", 10, true);
  vending_pub = nh.advertise<modules::misc_ctrl_msg>("vending_topic", 2);
  joystick_pub = nh.advertise<modules::joystick_msg>("joystick_topic", 2);
  log_enable_pub = nh.advertise<std_msgs::Bool>("log_enable_topic", 1);
  containers_pub = nh.advertise<modules::containers_msg>("containers_topic", 2);
  btmacs_pub = nh.advertise<modules::bt_macs_msg>("bt_macs_topic", 2, true);
  manager_status_pub =
      nh.advertise<modules::manager_status_msg>("manager_status", 1);
  mqtt_msg_pub = nh.advertise<std_msgs::String>("mqtt_on_msg", 2);
  speaker_set_vol_ros_pub_ = nh.advertise<std_msgs::Int32>(
                                                      kSpeakerSetVolTopicName,
                                                      kSpeakerSetVolTopicQSize);
  card_tap_event_pub_ =
      nh.advertise<std_msgs::Empty>(kPaymentCardTapTopicName, 2);
  trans_resp_event_pub_ =
      nh.advertise<std_msgs::Bool>(kPaymentTransRespTopicName, 2);

  mqtt_publisher_sub =
      nh.subscribe("mqtt_publishers", 10, &MQTTManager::Publishers_cb, this);

  nh_priv.param("macqtt", macqtt, 0);
  if (macqtt) {
    mac_service_client = nh.serviceClient<modules::mac_service>("mac_service");
    modules::mac_service mac_srv;
    ros::service::waitForService("mac_service");
    if (!mac_service_client.call(mac_srv))
      ROS_ERROR_STREAM(node_name << " --- mac_service couldn't be called");
    passwd = mac_srv.response.mac;
  } else {
    passwd = "VxVtfM3SUJmliXtn9vPa";
  }
  ROS_DEBUG_STREAM(node_name << "--- " << passwd);

  nh_priv.param<string>("mqtt_host", mqtt_host, kProdMqttHost);
  nh_priv.param("mqtt_port", mqtt_port, 8883);
  nh_priv.param<string>("mqtt_password", mqtt_password, passwd);
  nh_priv.param("mqtt_secure", securemqtt, true);
  nh_priv.param("mqtt_keepalive", mqtt_keepalive, 30);

  string keydir =
      ros::package::getPath("ota") + string{"/config/key.txt"};
  ifstream infile(keydir.c_str());
  string chiperStr = "";
  string key = "";

  if (infile.good()) {
    getline(infile, chiperStr);
    getline(infile, key);

    user_password = originalText(chiperStr, key);
  }
  infile.close();

  conn_stat_timer_ =
      nh.createTimer(ros::Duration(15.0),
                     bind(&MQTTManager::publishManagerStatus, this));

  init_program_pic_client =
      nh.serviceClient<std_srvs::Trigger>("init_program_pic");
  reload_broadcaster_clnt =
      nh.serviceClient<std_srvs::Trigger>("puppeteer/page_reload");
  console_broadcaster_clnt =
      nh.serviceClient<modules::console_service>("puppeteer/console");
  container_reset_clnt =
      nh.serviceClient<std_srvs::Trigger>("containers_reset_srv");
  pair_joystick_clnt =
      nh.serviceClient<std_srvs::SetBool>("joystick_pairing_service");
  pair_speaker_clnt =
      nh.serviceClient<std_srvs::SetBool>("speaker_pairing_service");
  imei_service_client = nh.serviceClient<modules::imei_service>("imei_service");
  modules::imei_service imei_srv;
  ros::service::waitForService("imei_service");
  if (!imei_service_client.call(imei_srv))
    ROS_ERROR_STREAM(node_name << " --- imei_service couldn't be called");
  mqtt_id = imei_srv.response.imei;

  status_msg.status = 6;
  status_pub.publish(status_msg);
  if (!mqtt_id.empty()) {
    connect();
  }
}

void MQTTManager::connect() {
  int nRet;

  reinitialise(mqtt_id.c_str(), false);

  std::string encrypt_file =
      ros::package::getPath("ota") + string{"/config/letsencrypt.pem"};

  if (securemqtt) {
    nRet = tls_set(encrypt_file.c_str());
    nRet = tls_insecure_set(false);
    nRet = tls_opts_set(0, "tlsv1.2", "ECDHE-RSA-AES256-GCM-SHA384");
    nRet = username_pw_set(mqtt_id.c_str(), mqtt_password.c_str());
  }

  nRet = connect_async(mqtt_host.c_str(), mqtt_port, mqtt_keepalive);
  nRet = loop_start();
}

void MQTTManager::Reconnect() {
  ROS_DEBUG("Connecting to %s, port: %d and keepalive: %d...",
            mqtt_host.c_str(), mqtt_port, mqtt_keepalive);
  int nRet = connect_async(mqtt_host.c_str(), mqtt_port, mqtt_keepalive);
  if (nRet == MOSQ_ERR_SUCCESS) {
    ROS_DEBUG("Connected");
  } else {
    ROS_ERROR("MQTT error: connect_async (%d)", nRet);
  }
}

void MQTTManager::Publishers_cb(
    const modules::mqtt_publishers_msg& mqtt_publisher) {
  //~ ROS_DEBUG_STREAM("Topic: " << mqtt_publisher.mqtt_topic.c_str());
  int x = publish(NULL, mqtt_publisher.mqtt_topic.c_str(),
                  strlen(mqtt_publisher.raw_msg.c_str()),
                  mqtt_publisher.raw_msg.c_str(), mqtt_publisher.qos, false);
  (x == 0) ? publish_status = true : publish_status = false;
}

void MQTTManager::on_disconnect(int rc) {
  ROS_ERROR_STREAM(node_name << " ---  disconnection(" << rc
                             << "), error: " << string{strerror(rc)});
  disconnection_error = rc;
  connection_status = false;
  // noConnectionAudio();
}

void MQTTManager::on_connect(int rc) {
  if (rc == 0) {
    ROS_DEBUG_STREAM(node_name << " --- connected with server");
    connection_status = true;
  } else {
    ROS_ERROR_STREAM(node_name << " --- Impossible to connect with server("
                               << rc << ")");
    connection_status = false;
  }
}

void MQTTManager::on_publish(int mid) {
  // ROS_DEBUG_STREAM (node_name << " --- Message (" << mid << ") succeed to be
  // published " );
  publication_id = mid;
}

void MQTTManager::on_subscribe(int mid, int qos_count, const int* granted_qos) {
  ROS_DEBUG_STREAM(node_name << " --- subscription succeeded (" << mid << ") ");
}

void MQTTManager::on_message(const struct mosquitto_message* message) {
  string topicFull = message->topic;

  ROS_DEBUG_STREAM(
      "==================== MQTT message received ====================");
  ROS_DEBUG_STREAM("Subscriber: " << mqtt_id);
  ROS_DEBUG_STREAM("Destiny topic: " << topicFull);
  ROS_DEBUG_STREAM("Data: " << reinterpret_cast<char*>(message->payload));

  vector<string> topic = split(topicFull, '/');
  string raw_msg = reinterpret_cast<char*>(message->payload);
  // Debug topic
  std_msgs::String str;
  str.data = "Topico: /" + topic.at(0) + ", msg: " + raw_msg;
  mqtt_msg_pub.publish(str);

  try {
    json jsonResponse = json::parse(raw_msg);
    if (topic.at(0) == "status") {
      ROS_DEBUG_STREAM("Cambio de estado.");
      if (jsonResponse.contains("ID_ESTATUS")) {
        status_msg.id = jsonResponse["UID_VEHICULO"];
        status_msg.status = jsonResponse["ID_ESTATUS"];
        if (status_msg.status == 5) {
          status_msg.control = jsonResponse["PUERTO_CONTROL"];
          status_msg.video = jsonResponse["PUERTO_VIDEO"];
          status_msg.sensores = jsonResponse["PUERTO_SENSOR"];
          status_msg.servidor = jsonResponse["IP_SERVIDOR"];
          status_msg.id_sesion_teleop = jsonResponse["UID_SESION_TELEOPS"];
          status_msg.url_servidor = jsonResponse["URL_SERVIDOR"];
          if (jsonResponse.contains("PUERTO_LATENCY"))
            status_msg.latency = jsonResponse["PUERTO_LATENCY"];
          else
            status_msg.latency = 0;
          if (jsonResponse.contains("SOURCE"))
            status_msg.source = jsonResponse["SOURCE"];
        } else if (status_msg.status == 4) {
          /* Se asigna vacio hasta que llegue el url del servidor de vending
           * (instruction)*/
          status_msg.url_servidor = "";
        } else {
          status_msg.control = 0;
          status_msg.video = 0;
          status_msg.sensores = 0;
          status_msg.latency = 0;
          status_msg.servidor = " ";
          status_msg.id_sesion_teleop = 0;
          status_msg.url_servidor = "";
        }
      }
      status_pub.publish(status_msg);
      if (status_msg.status == 6) joystick_pub.publish(joystick_msg);
    } else if (topic.at(0) == "instruction") {
      ROS_DEBUG_STREAM("Instruccion");
      ProcessInstruction(jsonResponse);
    } else if (topic.at(0) == "ota") {
      ROS_DEBUG_STREAM("Comando de OTA");
      ProcessOTACommand(jsonResponse);
    } else if (topic.at(0) == "communication") {
      ProcessCommunication(jsonResponse);
    }
  } catch (const exception& ex) {
    ROS_ERROR_STREAM(node_name << " --- Error parsing mqtt json: " << raw_msg);
    ROS_ERROR_STREAM(ex.what());
  }
  ROS_DEBUG_STREAM(
      "======================================================================="
      "=");
}

vector<string> split(string data, char spliter) {
  vector<string> tokens;
  string temp;
  stringstream check1(data);
  while (getline(check1, temp, spliter)) {
    tokens.push_back(temp);
  }
  /* for(int i = 0; i < tokens.size(); i++)
  cout << i << "(" << tokens[i].length() <<"):" << tokens[i] << endl; */
  return tokens;
}

string replaceFirstOccurrence(string& s, const string& toReplace,
                              const string& replaceWith) {
  size_t pos = s.find(toReplace);
  if (pos == std::string::npos) return s;
  return s.replace(pos, toReplace.length(), replaceWith);
}

void MQTTManager::ProcessOTACommand(json jsonMessage) {
  if (jsonMessage.contains("command")) {
    try {
      int nRet = 0;
      string command = jsonMessage["command"];

      if (command.compare("RB") == 0) {
        ROS_INFO_STREAM(node_name << " --- Stopping brain & rebooting");
        nRet = system(
            "shutdown -r +1 'The vehicle is about to restart (UI request)'");
        if (nRet == 0) nRet = system("systemctl --user stop roscore.service");
      } else if (command.compare("BU") == 0) {
        ROS_DEBUG_STREAM(node_name << " --- brain update");
        if (status_msg.status == 3 && !user_password.empty() &&
            jsonMessage.contains("data")) {

          string update_command =
              string{"echo " + user_password + " | sudo -S apt update"};
          nRet = system(update_command.c_str());

          string yml_data = jsonMessage["data"].get<string>();
          ryml::Tree tree_data =
              ryml::parse(ryml::to_csubstr(yml_data.c_str()));
          size_t root_id = tree_data.root_id(); /* root node id */

          if (tree_data.find_child(root_id, "dependencies") != ryml::NONE) {
            string install_command =
                "echo " + user_password + " | sudo -S apt install -y ";
            size_t dep_id = tree_data["dependencies"].id(); /* dep node id */

            if (tree_data.find_child(dep_id, "apt") != ryml::NONE) {
              ROS_DEBUG_STREAM(node_name << " --- apt dependencies");
              for (ryml::NodeRef const& child :
                   tree_data["dependencies"]["apt"].children()) {
                string temp = "";
                child >> temp;
                install_command += temp + ' ';
              }
            }

            nRet += system(install_command.c_str());
          }

          if (nRet == 0) {
            ofstream outfile(
                (string{homedir} + string{"/braintemp/sw_version.yaml"})
                    .c_str());
            outfile << yml_data;
            outfile.close();

            if (tree_data.find_child(root_id, "fw_update") != ryml::NONE &&
                tree_data["fw_update"].val() == "true") {
              ROS_DEBUG_STREAM(node_name << " --- updating fw...");
              std_srvs::Trigger program_pic_msg;
              bool fw_success = false;
              if (init_program_pic_client.call(program_pic_msg))
                fw_success = program_pic_msg.response.success;

              ROS_DEBUG_STREAM(node_name << " --- FW updated: "
                                         << (fw_success ? "true" : "false"));
            }
          }
        } else {
          ROS_ERROR_STREAM(
              node_name
              << " --- Error: vehicle not in idle status or bad update data");
          nRet = -1;
        }
      } else if (command.compare("LB") == 0) {
        ROS_DEBUG_STREAM(node_name << " --- launch brain");
        if (!brainStarted) {
          string scriptDir =
              string{homedir} + string{"/braintemp/launchBrain.sh"};
          string rosPackage = jsonMessage["rosPackage"];
          string rosLaunch = jsonMessage["rosLaunch"];
          string rosCommand = scriptDir + " " + rosPackage + " " + rosLaunch;

          ROS_DEBUG_STREAM(node_name << " --- launch ros command: "
                                     << rosCommand);

          pid = fork();
          if (pid > 0) { /** Parent process */
            brainStarted = true;
            nRet = 0;
            ROS_DEBUG_STREAM(node_name << " --- launch brain pid: " +
                                              to_string((int)pid));
          } else if (pid == 0) { /** Child process */
            setpgid(0, getpid());
            system(rosCommand.c_str());
            exit(0);
          } else {
            brainStarted = false;
            nRet = -1;
          }
        } else {
          ROS_WARN_STREAM(node_name << " --- brain already launched!");
        }
      } else if (command.compare("KB") == 0) {
        ROS_DEBUG_STREAM(node_name << " --- kill brain");
        if (brainStarted &&
            pid > 0) { /** Kill child and grandchild (all group process) */
          string rosCommand = "pkill -TERM -g " + to_string((int)pid);
          system(rosCommand.c_str());
          wait(NULL); /** Wait for status to avoid zombie process */
          brainStarted = false;
        } else {
          ROS_WARN_STREAM(node_name << " --- brain already killed!");
        }
      } else if (command.compare("ET") == 0) {
        ROS_DEBUG_STREAM(node_name << " --- launch bridge echo topic");
        if (!bridgeStarted) {
          string rosTopic = jsonMessage["rosTopic"];
          vector<string> rosType;

          topics.clear();
          ros::master::getTopics(topics);
          for (ros::master::V_TopicInfo::iterator it = topics.begin();
               it != topics.end(); it++) {
            const ros::master::TopicInfo& info = *it;
            if (info.name.compare(rosTopic) == 0)
              rosType = split(info.datatype, '/');
          }

          if (!rosType.empty()) {
            ROS_DEBUG_STREAM("topic: " << rosTopic
                                       << ", datatype: " << rosType.back());

            string bridgeParam = BRIDGE_PARAM;
            replaceFirstOccurrence(bridgeParam, "{topicFrom}", rosTopic);
            replaceFirstOccurrence(bridgeParam, "{dataPackage}",
                                   rosType.front());
            replaceFirstOccurrence(bridgeParam, "{dataType}", rosType.back());

            string scriptDir =
                string{homedir} + string{"/braintemp/launchBrain.sh"};
            string rosPackage = "modules";
            string rosLaunch = "mqtt_bridge.launch";
            string rosCommand = scriptDir + " " + rosPackage + " " + rosLaunch +
                                " " + mqtt_id + " " + bridgeParam;

            bridgePid = fork();
            if (bridgePid > 0) { /** Parent process */
              bridgeStarted = true;
              nRet = 0;
              ROS_DEBUG_STREAM(node_name << " --- launch bridge pid: " +
                                                to_string((int)bridgePid));
            } else if (bridgePid == 0) { /** Child process */
              setpgid(0, getpid());
              system(rosCommand.c_str());
              exit(0);
            } else {
              bridgeStarted = false;
              nRet = -1;
            }
          } else {
            ROS_WARN_STREAM(node_name << " --- Couldn't find topic!");
          }
        } else {
          ROS_WARN_STREAM(node_name << " --- bridge already launched!");
        }
      } else if (command.compare("ST") == 0) {
        ROS_DEBUG_STREAM(node_name << " --- kill bridge");
        if (bridgeStarted &&
            bridgePid >
                0) { /** Kill child and grandchild (all group process) */
          string rosCommand = "pkill -TERM -g " + to_string((int)bridgePid);
          system(rosCommand.c_str());
          wait(NULL); /** Wait for status to avoid zombie process */
          bridgeStarted = false;
        } else {
          ROS_WARN_STREAM(node_name << " --- bridge already killed!");
        }
      } else if (command.compare("SD") == 0) {
        ROS_INFO_STREAM(node_name << " --- Stopping brain & poweroff");
        nRet = system(
            "shutdown -P +1 'The vehicle is about to shutdown (UI request)'");
        if (nRet == 0) nRet = system("systemctl --user stop roscore.service");
      } else {
        ROS_WARN_STREAM(node_name << " --- command unknown");
      }

      if (nRet) ROS_ERROR_STREAM(node_name << " --- command system error");

    } catch (const exception& ex) {
      ROS_ERROR_STREAM(node_name << " --- Error parsing control json: "
                                 << jsonMessage);
      ROS_ERROR_STREAM(ex.what());
    } catch (...) {
      ROS_ERROR_STREAM(node_name << " --- Error defaul exception");
    }
  } else {
    ROS_WARN_STREAM(node_name << " --- Mqtt message has no command field");
    ROS_WARN_STREAM(jsonMessage);
  }
}

void MQTTManager::ProcessInstruction(json jsonMessage) {
  if (jsonMessage.contains("command")) {
    string command = jsonMessage["command"];

    if (command.compare("LR") == 0) {
      log_enable_msg.data = true;
      log_enable_pub.publish(log_enable_msg);
    } else if (command.compare("LH") == 0) {
      log_enable_msg.data = false;
      log_enable_pub.publish(log_enable_msg);
    } else if (command.compare("ST") == 0) {
      modules::containers_msg containers;
      modules::speaker_msg speaker;
      if (jsonMessage.contains("open") && jsonMessage["open"]) {
        containers.open = jsonMessage["open"];
        containers.id = jsonMessage["id"];
        if (jsonMessage.contains("mode")) {
          int mode = jsonMessage["mode"];
          switch (mode) {
            case SELL:
              speaker.mp3Id = "144";
              break;
            case REFILL:
              speaker.mp3Id = "142";
              break;
            case DELIVERY:
              speaker.mp3Id = "130";
              break;
            default:
              break;
          }
        }
        audio_pub.publish(speaker);
        containers_pub.publish(containers);
      } else if (jsonMessage.contains("status") && !jsonMessage["status"]) {
        if (jsonMessage.contains("mode") && jsonMessage["mode"] == 1) {
          speaker.mp3Id = "132";
          audio_pub.publish(speaker);
        } else {
          ROS_WARN_STREAM("Error en json de instruccion");
          ROS_DEBUG_STREAM(jsonMessage);
        }
      } else {
        ROS_WARN_STREAM("Error en json de instruccion");
        ROS_DEBUG_STREAM(jsonMessage);
      }
    } else if (command.compare("MU") == 0) {
      if (jsonMessage.contains("update") && jsonMessage["update"]) {
        modules::bt_macs_msg macs_msg;
        ROS_DEBUG_STREAM("macs are: " << jsonMessage["macs"]);
        macs_msg.macs = jsonMessage["macs"].dump();

        btmacs_pub.publish(macs_msg);
      } else {
        ROS_WARN_STREAM("Error en json de instruccion");
        ROS_DEBUG_STREAM(jsonMessage);
      }
    } else if (command.compare("SK") == 0) {
      audio_msg.mp3Id = "";
      audio_msg.text = "";
      audio_msg.lang = "";
      audio_msg.period = 0;
      if (jsonMessage.contains("id")) {
        audio_msg.mp3Id = jsonMessage["id"];
      }
      if (jsonMessage.contains("text")) {
        audio_msg.text = jsonMessage["text"];
      }
      if (jsonMessage.contains("lang")) {
        /* en (English) / es (Español)*/
        audio_msg.lang = jsonMessage["lang"];
      }
      if (jsonMessage.contains("period")) {
        audio_msg.period = jsonMessage["period"];
      }
      audio_pub.publish(audio_msg);
    } else if (command.compare("NT") == 0) {
      if (!user_password.empty()) {
        string switch_network_command =
            "echo " + user_password + " | sudo -S /usr/bin/switch-interface.sh";

        Reconnect(); /* Neccessary to catch the disconnection quickly */
        ROS_DEBUG("Switching network ...");
        int nRet = system(switch_network_command.c_str());
        if (nRet == 0) {
          ROS_INFO("Switch network script executed successfully");
          Reconnect(); /* Neccessary to reconnect with the new interface */
          ReloadBroadcaster();
        }
      }
    } else if (command.compare("CBT") == 0) {
      std_srvs::Trigger msg;
      if (!container_reset_clnt.call(msg)) {
        ROS_WARN("Error calling container reset service");
      }
    } else if (command.compare("VC") == 0) {
      if (status_msg.status == 4 && jsonMessage.contains("url")) {
        status_msg.url_servidor = jsonMessage["url"];
        status_pub.publish(status_msg);
      }

      if (status_msg.status == 4 && jsonMessage.contains("spin")) {
        modules::misc_ctrl_msg vending_ctrl;
        vending_ctrl.MovCamera = jsonMessage["spin"];
        vending_pub.publish(vending_ctrl);
      }
    } else if (command.compare("MN") == 0) {
      if (jsonMessage.contains("exposure")) {
        joystick_msg.cameraExp = jsonMessage["exposure"];
        joystick_pub.publish(joystick_msg);
      }

      if (jsonMessage.contains("spin")) {
        joystick_msg.MovCamera = jsonMessage["spin"];
        joystick_pub.publish(joystick_msg);
      }

      if (jsonMessage.contains("camera")) {
        int camera_id = jsonMessage["camera"];
        modules::console_service broadcaster_msg;
        broadcaster_msg.request.command =
            "window.setCamera(" + to_string(camera_id) + ")";
        if (console_broadcaster_clnt.call(broadcaster_msg)) {
          ROS_INFO("Broadcaster camera set: success = %s, msg = %s",
                   to_string(broadcaster_msg.response.success).c_str(),
                   broadcaster_msg.response.message.data());
        } else {
          ROS_WARN("Error calling broadcaster console service");
        }
      }

      /*Updating UI... */
      GetMonitorFeedback();
    } else if (command.compare("SV") == 0) {
      if (jsonMessage.contains("vol")) {
        std_msgs::Int32 msg;
        int vol_desired = jsonMessage["vol"];

        msg.data = (std::int32_t) vol_desired;
        speaker_set_vol_ros_pub_.publish(msg);
      }
    } else if (command.compare("SA") == 0) {
      if (status_msg.id != 0) {
        string sync_audio = ros::package::getPath("ota") +
                            "/scripts/sync_audios.sh --uid " +
                            to_string(status_msg.id);
        if (system(sync_audio.c_str())) {
          ROS_ERROR("%s -- Error calling sync_audios script",
                    node_name.c_str());
        }
      } else {
        ROS_ERROR("%s -- Error vehicle UID is 0", node_name.c_str());
      }
    } else if (command.compare("RP") == 0) {
      if (status_msg.status != 3) {
        ReloadBroadcaster();
      }
    } else if (command.compare("FB") == 0 && status_msg.status == 6) {
      /*On demand monitor feedback callback to update UI*/
      GetMonitorFeedback();
    } else if (command.compare("PD") == 0) {
      /* Pairing/removing device [joystick/speaker] */
      std_srvs::SetBool pair_msg;
      if (jsonMessage.contains("joystick")) {
        pair_msg.request.data = (jsonMessage["joystick"] == 1);
        /* Restart bluetooth.service when joystick is removed to prevent bugs in
         * the pairing procedure */
        if (!pair_msg.request.data && !user_password.empty()) {
          string restart_bt_srv =
              "echo " + user_password +
              " | sudo -S systemctl restart bluetooth.service";
          int nRet = system(restart_bt_srv.c_str());
          if (nRet == 0) {
            ROS_INFO("bluetooth.service restarted successfully");
          }
        }
        if (!pair_joystick_clnt.call(pair_msg)) {
          ROS_WARN("Error calling joystick pairing service");
        }
      }
      if (jsonMessage.contains("speaker")) {
        pair_msg.request.data = (jsonMessage["speaker"] == 1);
        if (!pair_speaker_clnt.call(pair_msg)) {
          ROS_WARN("Error calling speaker pairing service");
        }
      }
    } else if (command.compare("SE") == 0) {
      /* Selecting broker host */
      if (mqtt_host.compare(kProdMqttHost) == 0) {
        mqtt_host = kDevMqttHost;
      } else if (mqtt_host.compare(kDevMqttHost) == 0) {
        mqtt_host = kProdMqttHost;
      }

      Reconnect();
    } else if (command.compare("TAP") == 0) {
      /* Card Tap event from containers */
      std_msgs::Empty msg;
      card_tap_event_pub_.publish(msg);
    } else if (command.compare("TR") == 0) {
      /* Transaction Response event from containers */
      std_msgs::Bool response_msg;
      response_msg.data = jsonMessage["result"];
      trans_resp_event_pub_.publish(response_msg);
    } else {
      ROS_WARN("Unkown instruction command received");
      ROS_WARN_STREAM(command);
    }
  } else {
    ROS_WARN("Mqtt instrucion has no command field");
    ROS_WARN_STREAM(jsonMessage);
  }
}

void MQTTManager::ProcessCommunication(json jsonMessage) {
  if (jsonMessage.contains("text")) {
    audio_msg.mp3Id = "";
    audio_msg.lang = "";
    audio_msg.period = 0;

    audio_msg.text = jsonMessage["text"];
    if (jsonMessage.contains("lang")) {
      /* en (English) / es (Español)*/
      audio_msg.lang = jsonMessage["lang"];
    }
    audio_pub.publish(audio_msg);
  } else {
    ROS_WARN("Communication has no text field");
    ROS_WARN_STREAM(jsonMessage);
  }
}

void MQTTManager::GetMonitorFeedback() {
  json selected_camera;
  json detected_cameras;
  modules::console_service broadcaster_msg;

  broadcaster_msg.request.command = "window.currentCamera()";
  if (console_broadcaster_clnt.call(broadcaster_msg)) {
    if (broadcaster_msg.response.success) {
      try {
        selected_camera = json::parse(broadcaster_msg.response.message);
      } catch (const std::exception& e) {
        ROS_WARN("Error parsing mqtt json: %s",
                 broadcaster_msg.response.message.data());
        ROS_WARN_STREAM(e.what());
      }
    } else {
      ROS_WARN("Error in broadcaster console service");
    }
  } else {
    ROS_WARN("Error calling broadcaster console service");
  }

  broadcaster_msg.request.command = "window.listCameras()";
  if (console_broadcaster_clnt.call(broadcaster_msg)) {
    if (broadcaster_msg.response.success) {
      try {
        detected_cameras = json::parse(broadcaster_msg.response.message);
      } catch (const std::exception& e) {
        ROS_WARN("Error parsing mqtt json: %s",
                 broadcaster_msg.response.message.data());
        ROS_WARN_STREAM(e.what());
      }
    } else {
      ROS_WARN("Error in broadcaster console service");
    }
  } else {
    ROS_WARN("Error calling broadcaster console service");
  }

  json mn_feedback;
  mn_feedback["exposure"] = joystick_msg.cameraExp;
  mn_feedback["spin"] = joystick_msg.MovCamera;
  mn_feedback["curr_camera"] = selected_camera;
  mn_feedback["list_cameras"] = detected_cameras;
  ROS_DEBUG_STREAM(mn_feedback.dump());

  try {
    modules::mqtt_publishers_msg mqtt_feedback_msg;
    mqtt_feedback_msg.raw_msg = mn_feedback.dump();
    mqtt_feedback_msg.mqtt_topic = "sensor/imei/monitor";
    mqtt_feedback_msg.qos = 0;
    Publishers_cb(mqtt_feedback_msg);
  } catch (const exception& e) {
    ROS_ERROR_STREAM(e.what());
  }

  if (selected_camera.empty()) {
    ReloadBroadcaster();
  }
}

void MQTTManager::ReloadBroadcaster() {
  std_srvs::Trigger broadcaster_msg;
  if (reload_broadcaster_clnt.call(broadcaster_msg)) {
    ROS_INFO("Broadcaster reload: msg = %s",
             broadcaster_msg.response.message.data());
  } else {
    ROS_WARN("Error calling broadcaster reload service");
  }
}

void MQTTManager::noConnectionAudio() {
  ROS_DEBUG_STREAM(node_name << "--- noConnectionAudio()");
  audio_msg.mp3Id = "140";
  audio_pub.publish(audio_msg);
}

MQTTManager::~MQTTManager() {
  disconnect();
  loop_stop();
  mosqpp::lib_cleanup();
}

void MQTTManager::publishManagerStatus() {
  ROS_DEBUG("Checking connection, status: %s",
            (connection_status ? "true" : "false"));
  if (!connection_status) {
    Reconnect();
  }

  modules::manager_status_msg msg;
  msg.publish_status = publish_status;
  msg.connection_status = connection_status;
  msg.publication_id = publication_id;
  msg.disconnection_error = disconnection_error;
  manager_status_pub.publish(msg);
}
