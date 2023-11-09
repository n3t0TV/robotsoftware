#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <pwd.h>
#include <vector>
#include <fstream>
#include <stdio.h>      /* printf, NULL */
#include <string.h>
#include <iostream>
#include <stdlib.h>     /* strtoull */
#include <unistd.h>
#include <sys/types.h>

#include "libraries/json.hpp"

#include <modules/nodebeat_msg.h>

#include <std_srvs/Trigger.h>
#include <modules/mac_service.h>
#include <modules/git_service.h>
#include <modules/imei_service.h>
#include <modules/provider_service.h>

#define RYML_SINGLE_HDR_DEFINE_NOW
#include "libraries/ryml_all.hpp"

using namespace std;
using namespace ros;
using json = nlohmann::json;

vector<string> split(string data, char spliter)
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

class VersionManager{
    public:
        VersionManager(ros::NodeHandle);
        ~VersionManager();	
    
    private:
        string node_name, imei, imeidir, provider0dir, provider1dir, provider0, provider1, gitdir, macdir, mac, sw_version, sw_version_dir;
		string commit, branch, macs;
        const char *homedir;

        ros::NodeHandle nh;
		ros::Timer timerNodebeat;
		ros::Publisher pubNodebeat;
        ros::ServiceServer servImei, servProvider, servGit, servMac, version_srv;

		modules::nodebeat_msg beat_msg;

		void gitLog();
		void readMac();
		void nodebeat();
        string readImei();
        void readProviders();
        void saveImei(uint64_t imei_ull);
		bool ReadVersionFile();
		bool gitService(modules::git_service::Request& request,modules::git_service::Response& response);
		bool macService(modules::mac_service::Request& request, modules::mac_service::Response& response);
		bool ImeiService(modules::imei_service::Request& request,modules::imei_service::Response& response);
        bool providerService(modules::provider_service::Request& request,modules::provider_service::Response& response);
		bool VersionService(std_srvs::Trigger::Request& request,std_srvs::Trigger::Response& response);
};

VersionManager::VersionManager(ros::NodeHandle nh_priv){
	node_name = ros::this_node::getName();
	ROS_DEBUG_STREAM(node_name);
	int log_level;
	nh_priv.param("log_level", log_level,0);
	ros::console::levels::Level console_level;
	console_level = (ros::console::levels::Level)log_level;
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level) ) {
	ros::console::notifyLoggerLevelsChanged();
	}

	if ((homedir = getenv("HOME")) == NULL) {
		homedir = getpwuid(getuid())->pw_dir;
	}
	imeidir =  string{homedir} + string{ "/braintemp/imei.txt"};
	macdir = "/sys/class/net/eth0/address";
	macs = string{homedir} + string{"/braintemp/macs.json"};
	
	gitLog();
	readMac();
	
	pubNodebeat = nh.advertise<modules::nodebeat_msg>("diagnostic/nodebeat",5);
	
	timerNodebeat = nh.createTimer(Duration(3.0), bind(&VersionManager::nodebeat, this));
	
	servGit = nh.advertiseService("git_service",&VersionManager::gitService,this);
	servMac = nh.advertiseService("mac_service",&VersionManager::macService, this);
	servImei = nh.advertiseService("imei_service",&VersionManager::ImeiService,this);
	servProvider = nh.advertiseService("provider_service",&VersionManager::providerService,this);
	version_srv = nh.advertiseService("version_service",&VersionManager::VersionService,this);
}

string VersionManager::readImei(){	
	ROS_DEBUG_STREAM("readImei()");  
	ifstream infile(imeidir.c_str());
	if(infile.good())
	{
		try
		{
			json j = json::parse(infile);
			if(j.contains("imei"))
			{
				uint64_t imei_ull = j.at("imei");
				return 	to_string(imei_ull);
			}			
			else
			{	ROS_DEBUG_STREAM("No IMEI key in file: ");
				return "";
			}
		} catch(...){
			ROS_ERROR_STREAM("Error al leer imei.");
			return "";
		}
	}
	else{
		ROS_ERROR_STREAM("Error al leer archivo de imei.");
		return "";
	}
}

void VersionManager::readMac(){
	ifstream infile(macdir.c_str());
	if(infile.good()){
		getline(infile, mac);
	} else {
		ROS_ERROR_STREAM("Error al leer mac.");
	}
}

void VersionManager::readProviders(){	  
	provider0dir =  string{homedir} + string{ "/braintemp/carrier-ppp0.txt"};
	provider1dir =  string{homedir} + string{ "/braintemp/carrier-ppp1.txt"};
	
	ifstream infile0(provider0dir.c_str());
	if(infile0.good())
	{
	    getline(infile0, provider0);
	}
	else{
		ROS_WARN_STREAM(node_name << "Provider 0 corrupted");
	}
	ifstream infile1(provider1dir.c_str());
	if(infile1.good())
	{
	    getline(infile1, provider1);
	}
	else{
		ROS_WARN_STREAM(node_name << "Provider 1 corrupted");
	}
}

void VersionManager::saveImei(uint64_t imei_ull){
	json j;
	j["imei"] = imei_ull;
	ofstream outfile(imeidir.c_str());
	outfile << j.dump();
	outfile.close();
}

bool VersionManager::ImeiService(modules::imei_service::Request& request,modules::imei_service::Response& response){
	ROS_DEBUG_STREAM("IMEI service");
    string package_path = ros::package::getPath("modules");
	string imei;
		
	if(imei.empty()) 	//Dont have an imei ready yet otherwise just return it
	{	
		imei = readImei();		
		ROS_DEBUG_STREAM("IMEI from file: " << imei);
	}
	else
	{
		ROS_DEBUG_STREAM("IMEI already known.");
	}	
	ROS_DEBUG_STREAM("IMEI service: " << imei);
	response.imei = imei;
	return true;
}

bool VersionManager::ReadVersionFile(){
	bool success = false;
	sw_version_dir = string{homedir} + string{ "/braintemp/sw_version.yaml"};
	ifstream rfile(sw_version_dir.c_str());
	string yml_buff = "";
	if(rfile.good())
	{
		try {
			string rline = "";
			while(getline(rfile, rline))
			{
				yml_buff += rline + "\n";
			}

			ryml::Tree tree = ryml::parse(ryml::to_csubstr(yml_buff.c_str()));
			if (tree.find_child(tree.root_id(), "version") != ryml::NONE) {
				ryml::NodeRef version = tree["version"];
				version >> sw_version;
				success = true;
			}
		} catch (const exception &ex) {
			ROS_ERROR_STREAM(node_name << " --- Error reading version yaml: "  << ex.what());
		} catch ( ... ) {
			ROS_ERROR_STREAM(node_name << " --- Error reading version yaml");
		}
	}

	rfile.close();
	return success;
}

bool VersionManager::VersionService(std_srvs::Trigger::Request& request,std_srvs::Trigger::Response& response) {
	ROS_DEBUG_STREAM("Version service");

	if(sw_version.empty()) 	//Dont have a version ready yet otherwise just return it
	{
		response.success = ReadVersionFile();
		response.message = sw_version;
	} else {
		response.success = true;
		response.message = sw_version;
	}

	ROS_DEBUG_STREAM("SW version: " << sw_version);
	return response.success;
}

bool VersionManager::gitService(modules::git_service::Request& request,modules::git_service::Response& response){
	ROS_DEBUG_STREAM("Git service");	
	response.commit = commit;
	response.branch = branch;
	return true;
}

bool VersionManager::providerService(modules::provider_service::Request& request,modules::provider_service::Response& response){
	string active_interface{""};
	string active_interface_dir =  string{homedir} + string{ "/braintemp/active-interface.txt"};
	
	ifstream infile(active_interface_dir.c_str());
	if(infile.good())
	{
	    getline(infile, active_interface);
	}
	else{
		ROS_WARN_STREAM(node_name << "--- Active interface corrupted");
	}
	ROS_DEBUG_STREAM(active_interface);

	readProviders();
	if(active_interface == "ppp0")
		response.provider = provider0;
	else if(active_interface == "ppp1")
		response.provider = provider1;
	else
		response.provider = "ERROR";
	return true;
}

bool VersionManager::macService(modules::mac_service::Request& request, modules::mac_service::Response& response){
	ROS_DEBUG_STREAM("Mac service");
	response.mac = mac;
	return true;
}

void VersionManager::gitLog(){

	string command, file;
	ifstream infile;

	command = "cd " + string{homedir} + string{"/catkin_ws/src/brainjetson/; git rev-parse --short HEAD > "} + string{homedir} + string{"/braintemp/commit.txt"};
	system(command.c_str());
	file = string{homedir} + string{"/braintemp/commit.txt"};
	infile.open(file.c_str());
	if(infile.good()){
		string line;
		getline(infile,commit);
		// ROS_DEBUG_STREAM(commit);
	}
	else{
		ROS_WARN_STREAM("Error reading commit file.");
	}
	infile.close();

	command = "cd " + string{homedir} + string{"/catkin_ws/src/brainjetson/; git rev-parse --abbrev-ref HEAD > "} + string{homedir} + string{"/braintemp/branch.txt"};
	system(command.c_str());
	file = string{homedir} + string{"/braintemp/branch.txt"};
	infile.open(file.c_str());
	if(infile.good()){
		string line;
		getline(infile,branch);
		// ROS_DEBUG_STREAM(branch);
	}
	else{
		ROS_WARN_STREAM("Error reading branch file.");
	}
	infile.close();
}

void VersionManager::nodebeat(){
	// ROS_DEBUG_STREAM(node_name << " --- <3");
	beat_msg.node = node_name;
	beat_msg.status = true;
	beat_msg.timestamp = ros::Time::now();
	pubNodebeat.publish(beat_msg);
}

VersionManager::~VersionManager(){

}
