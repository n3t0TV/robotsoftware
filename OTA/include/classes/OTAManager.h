#include <pwd.h>
#include <vector>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <sys/types.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/master.h>

#include <mosquittopp.h>
#include "libraries/json.hpp"

#include <modules/mqtt_subscribers_msg.h>
#include <modules/imei_service.h>
#include <modules/mqtt_subscribe_srv.h>

#define BRIDGE_PARAM "\"[{factory: 'mqtt_bridge.bridge:RosToMqttBridge', msg_type: '{dataPackage}.msg:{dataType}',topic_from: {topicFrom}, topic_to: ~/out}]\""

using namespace std;
using json = nlohmann::json;

string replaceFirstOccurrence(string& s, const string& toReplace, const string& replaceWith);
vector<string> split(string data, const char spliter);

class OTAManager
{
    public:
	OTAManager(ros::NodeHandle nh_priv);
	~OTAManager(){};
    private:
	string base_topic, node_name, imei;
	ros::Subscriber subMQTT;
	ros::NodeHandle nh;
	ros::ServiceClient imei_service_client, mqtt_subscribe_client;
	bool brainStarted{false}, bridgeStarted{false}, initBrain;
	const char *homedir;
	pid_t pid, bridgePid;
	ros::master::V_TopicInfo topics;

	void Subscriber_cb(const modules::mqtt_subscribers_msg& subscribers_msg);
	void ProcessInstruction(json jsonMessage);
	void InitBrainCommand();
};

OTAManager::OTAManager(ros::NodeHandle nh_priv)
{
    int log_level;
    nh_priv.param("log_level", log_level,0);
    ros::console::levels::Level console_level;
    console_level = (ros::console::levels::Level)log_level;
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    node_name = ros::this_node::getName	();
    nh_priv.param<string>("base_topic", base_topic,"ota");
    nh_priv.param("initBrain", initBrain,false);
    subMQTT = nh.subscribe("mqtt_subscribers",10, &OTAManager::Subscriber_cb, this);
    imei_service_client = nh.serviceClient<modules::imei_service>("imei_service");
    mqtt_subscribe_client = nh.serviceClient<modules::mqtt_subscribe_srv>("mqtt_subscribe");

    if ((homedir = getenv("HOME")) == NULL) {
	homedir = getpwuid(getuid())->pw_dir;
    }

    modules::imei_service imei_srv;
    ros::service::waitForService ("imei_service");
    if (!imei_service_client.call(imei_srv))
	ROS_ERROR_STREAM(node_name << " --- Cannot call imei_service");
    else 
	ROS_DEBUG_STREAM(node_name << " --- imei_service called");
    imei = imei_srv.response.imei;
    string topic = base_topic + "/" + imei;
    modules::mqtt_subscribe_srv subscribe_srv;
    ros::service::waitForService ("mqtt_subscribe");
    subscribe_srv.request.mqtt_topic = topic;
    subscribe_srv.request.node_id = node_name;
    if(!mqtt_subscribe_client.call(subscribe_srv))
    {
	ROS_ERROR_STREAM(node_name << " node couldn't call MQTT subscribe service ");
    }
    //~ if (initBrain) InitBrainCommand();
}

void OTAManager::InitBrainCommand()
{
    json jsonRequest;
    jsonRequest["command"]="LB";
    jsonRequest["rosPackage"]="modules";
    jsonRequest["rosLaunch"]="teleops.launch";
    ROS_DEBUG_STREAM(node_name << " --- InitBrainCommand");
    ProcessInstruction(jsonRequest);
}

void OTAManager::Subscriber_cb(const modules::mqtt_subscribers_msg& subscribers_msg)
{
    if (subscribers_msg.mqtt_topic == base_topic) {
	json jsonRequest = json::parse(subscribers_msg.raw_msg);
	ROS_DEBUG_STREAM(node_name << " --- Command received content: ");
	ROS_DEBUG_STREAM(jsonRequest);
	ProcessInstruction(jsonRequest);
    }
}

void OTAManager::ProcessInstruction(json jsonMessage)
{
    if(jsonMessage.contains("command"))
    {
	try {
	    int nRet = 0;
	    string command = jsonMessage["command"];

	    if (command.compare("RB")==0)
	    {
			ROS_DEBUG_STREAM(node_name << " --- reboot");
			nRet = system("shutdown -r now");			
	    }
	    else if (command.compare("GP")==0)
	    {
			ROS_DEBUG_STREAM(node_name << " --- git pull and compile");
			string scriptDir = string{homedir} + string{ "/braintemp/gitCompile.sh"};
			string rosCommand = scriptDir + " pull";

			nRet = system(rosCommand.c_str());
	    }
	    else if (command.compare("GC")==0)
	    {
			ROS_DEBUG_STREAM(node_name << " --- git checkout and compile");
			string scriptDir = string{homedir} + string{ "/braintemp/gitCompile.sh"};
			string gitID = jsonMessage["gitID"];
			string rosCommand = scriptDir + " checkout " + gitID;

			nRet = system(rosCommand.c_str());
	    }		
	    else if (command.compare("LB")==0)
	    {	
			ROS_DEBUG_STREAM(node_name << " --- launch brain");
			if (!brainStarted) {
				string scriptDir = string{homedir} + string{ "/braintemp/launchBrain.sh"};
				string rosPackage = jsonMessage["rosPackage"];
				string rosLaunch = jsonMessage["rosLaunch"];
				string rosCommand = scriptDir + " " + rosPackage + " " + rosLaunch;

				ROS_DEBUG_STREAM(node_name << " --- launch ros command: " << rosCommand);
				
				pid = fork();
				if (pid > 0) {	/** Parent process */
				brainStarted = true;
				nRet = 0;
				ROS_DEBUG_STREAM(node_name << " --- launch brain pid: " + to_string((int)pid));
				} else if (pid == 0) {	/** Child process */
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
		}
	    else if (command.compare("KB")==0)
	    {
			ROS_DEBUG_STREAM(node_name << " --- kill brain");
			if (brainStarted && pid > 0) {		/** Kill child and grandchild (all group process) */
				string rosCommand = "pkill -TERM -g " + to_string((int)pid);
				system(rosCommand.c_str());
				wait(NULL);				/** Wait for status to avoid zombie process */
				brainStarted = false;
			} else {
		    ROS_WARN_STREAM(node_name << " --- brain already killed!");
		}
	    }
	    else if (command.compare("ET")==0)
	    {	
			ROS_DEBUG_STREAM(node_name << " --- launch bridge echo topic");
			if (!bridgeStarted) {
				string rosTopic = jsonMessage["rosTopic"];
				vector<string> rosType;

				topics.clear();
				ros::master::getTopics(topics);
				for (ros::master::V_TopicInfo::iterator it = topics.begin() ; it != topics.end(); it++) {
				const ros::master::TopicInfo& info = *it;
				if (info.name.compare(rosTopic)==0)
					rosType = split(info.datatype, '/');
				}

				if (!rosType.empty()) {
				ROS_DEBUG_STREAM("topic: " << rosTopic << ", datatype: " << rosType.back());

				string bridgeParam = BRIDGE_PARAM;
				replaceFirstOccurrence(bridgeParam, "{topicFrom}", rosTopic);
				replaceFirstOccurrence(bridgeParam, "{dataPackage}", rosType.front());
				replaceFirstOccurrence(bridgeParam, "{dataType}", rosType.back());

				string scriptDir = string{homedir} + string{ "/braintemp/launchBrain.sh"};
				string rosPackage = "modules";
				string rosLaunch = "mqtt_bridge.launch";
				string rosCommand = scriptDir + " " + rosPackage + " " + rosLaunch + " " + imei + " " + bridgeParam;
				
				bridgePid = fork();
				if (bridgePid > 0) {	/** Parent process */
					bridgeStarted = true;
					nRet = 0;
					ROS_DEBUG_STREAM(node_name << " --- launch bridge pid: " + to_string((int)bridgePid));
				} else if (bridgePid == 0) {	/** Child process */
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
	    }
	    else if (command.compare("ST")==0)
	    {
			ROS_DEBUG_STREAM(node_name << " --- kill bridge");
			if (bridgeStarted && bridgePid > 0) {		/** Kill child and grandchild (all group process) */
				string rosCommand = "pkill -TERM -g " + to_string((int)bridgePid);
				system(rosCommand.c_str());
				wait(NULL);				/** Wait for status to avoid zombie process */
				bridgeStarted = false;
			} else {
		    ROS_WARN_STREAM(node_name << " --- bridge already killed!");
		}
	    }
	    else if (command.compare("SD")==0)
	    {
			ROS_DEBUG_STREAM(node_name << " --- shutdown ");
			nRet = system("shutdown -P now");			
	    }
	    else
	    {
			ROS_WARN_STREAM(node_name << " --- command unknown");			
	    }
	    
	    if (nRet)
			ROS_ERROR_STREAM(node_name << " --- command system error");

	} catch (const exception &ex) {
	    ROS_ERROR_STREAM(node_name << " --- Error parsing control json: "  << jsonMessage);
	    ROS_ERROR_STREAM(ex.what());
	}
    }
    else
    {	
	ROS_WARN_STREAM(node_name << " --- Mqtt message has no command field");
	ROS_WARN_STREAM(jsonMessage);
    }
}

string replaceFirstOccurrence(string& s, const string& toReplace, const string& replaceWith)
{
    size_t pos = s.find(toReplace);
    if (pos == std::string::npos) return s;
    return s.replace(pos, toReplace.length(), replaceWith);
}

vector<string> split(string data, const char spliter)
{
    vector<string> tokens;
    string temp;
    stringstream check1(data);
    while(getline(check1, temp, spliter))
    {
	tokens.push_back(temp);
    }
    return tokens;
}
