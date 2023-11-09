#include <regex>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <sstream>
#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h> 
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <algorithm>
#include <iterator>
#include <bits/stdc++.h>

#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>

#include "drivers/gps_msg.h"
#include "drivers/gps_dgnst_msg.h"

#include "libraries/json.hpp"
#include <CppLinuxSerial/SerialPort.hpp>

using namespace mn::CppLinuxSerial;
using namespace std;
using namespace std::chrono;

vector<string> split(string data, char spliter);

class GPS_GSM
{
    public:
		GPS_GSM(ros::NodeHandle nh_priv);
		~GPS_GSM();
		void publishGpsData();
		int initModule();
		string node_name;

	private:
		SerialPort serialPort;

		bool gpsStatus();
		int deinitModule();
		string sendAT(string command);
		string readSerial();
		void getPosicion();
		void getLatitud(string latitude);
		void getLongitud(string longitud);
		void getAltitud(string altitud);
		void simInserted();
		void getCalidad();		
		void getUTCTime(string utc);
		void getUTCDate(string date);
		void getAccuracy(string hdop);
		int getSatellites(string satellites);
		void getNetworkInformation();
		void getServingCell();
		void gpsSession();
	
		struct module_data
		{
			int sim=0;
			int rssi=0,rssiDBM=0,rssiPercentage=0;
			double longitud=0;
			double latitud=0;
			double altitud=0;
			string latHemisphere;
			string lonHemisphere;
			string UTCTime = "";
			string UTCDate = "";
			double accuracy = 0;
			int satellites = 0;
			string iccid = "";    
			string provider = "";
			string tech = "";
			string band = "";
			bool latValid=false, longValid=false, gpsValid=false;
			vector<string> satellitesSNR,satellitesPRN,satellitesUsed, satelliteInfo;
		}data;
		struct module_dgnst
		{
			bool session = false;
			bool satellites = false;
			bool coordinates = false;
			bool sim = false;
			bool fixed = false;
		}dgnst;	
		int status;
		bool fd;
		string serial_port;

		ros::NodeHandle nh;		
		ros::Publisher pubGpsData, pubGpsDgnst;
		drivers::gps_msg gpsData;
		drivers::gps_dgnst_msg gpsDgnst;		
};

GPS_GSM::GPS_GSM(ros::NodeHandle nh_priv)
{
	node_name = ros::this_node::getName();
    int log_level;
    nh_priv.param("log_level", log_level,0);
	string param_path = node_name +"/serial_port";
	nh_priv.getParam(param_path, serial_port);
	ros::console::levels::Level console_level;
    console_level = (ros::console::levels::Level)log_level;    
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    
	if(serial_port == "/dev/ttyUSB6")
    	pubGpsData = nh.advertise<drivers::gps_msg>("sensor_topic/gps/A", 10);
	else if (serial_port == "/dev/ttyUSB2")
		pubGpsData = nh.advertise<drivers::gps_msg>("sensor_topic/gps/B", 10);
	else
		ROS_ERROR_STREAM("Puerto serial invalido.");
	pubGpsDgnst = nh.advertise<drivers::gps_dgnst_msg>("diagnostic_topic/gps",10);
}

GPS_GSM::~GPS_GSM()
{
	string command = "AT+QGPSEND\r";
    sendAT(command);
	string reply = readSerial();
    serialPort.Close();
}

void GPS_GSM::publishGpsData()
{
	ROS_DEBUG_STREAM(node_name << "--- publishGpsData()");
	// ROS_DEBUG_STREAM("gpsSession()");
	gpsSession();
	// ROS_DEBUG_STREAM("getCalidad()");
	getCalidad();
	if(data.sim < 0)
		data.rssi += 300;
	// ROS_DEBUG_STREAM("getPosicion()");	
	getPosicion();
	// ROS_DEBUG_STREAM("getNetworkInformation()");
	getNetworkInformation();
	// ROS_DEBUG_STREAM("getServingCell()");
	getServingCell();
		
	gpsData.validGPS=data.gpsValid;	
	gpsData.Latitude=data.latitud;
	gpsData.Longitude=data.longitud;
	gpsData.Altitude=data.altitud;
	gpsData.Sim=data.sim;
	gpsData.rssi=data.rssiPercentage;
	gpsData.LatHemisphere = data.latHemisphere;
	gpsData.LonHemisphere = data.lonHemisphere;
	gpsData.Accuracy = data.accuracy;
	gpsData.UTCTime = data.UTCTime;
	gpsData.UTCDate = data.UTCDate;
	gpsData.Satellites = data.satellites;
	gpsData.provider = data.provider;
	gpsData.accessTechnology = data.tech;
	gpsData.band = data.band;
	gpsData.ModuleConnected = fd;

	gpsDgnst.gpsSession = dgnst.session;
	gpsDgnst.sim = dgnst.sim;
	gpsDgnst.fixed = dgnst.fixed;

	pubGpsData.publish(gpsData);
	pubGpsDgnst.publish(gpsDgnst);
}

vector<string> split(string data, char spliter)
{
	vector<string> tokens;
	string temp;
	stringstream check1(data);
	while(getline(check1, temp, spliter))
	{
		tokens.push_back(temp);
	}
	/* for(int i = 0; i < tokens.size(); i++)
        cout << i << "(" << tokens[i].length() <<"):" << tokens[i] << endl; */
	return tokens;
}

string GPS_GSM::readSerial()
{
	string buffer;
	bool waiting = true;
	while(waiting){
        serialPort.Read(buffer);		
		if (buffer.length() > 1)
			waiting = false;
    }
	return buffer;
}

string GPS_GSM::sendAT(string command)
{	
	// ROS_DEBUG_STREAM(command);
	serialPort.Write(command);
	ros::Duration(0.3).sleep();
    string reply = readSerial();
	// ROS_DEBUG_STREAM(reply);
    return reply;
}

/*
 * name: init_module()
 * @return 1: module physically connected 0: module physically not connected
 */
int GPS_GSM::initModule()
{
	try
	{
		ROS_INFO_STREAM("GPS puerto: " << serial_port);
		serialPort.SetDevice(serial_port);
		serialPort.SetBaudRate(BaudRate::B_115200);
		serialPort.SetTimeout(-1);
		serialPort.Open();
		fd = true;
		status = 1;
		
		string command,reply;
		
		command = "AT+QGPS=1\r";	// Turn on GNSS engine		
		reply = sendAT(command);		

		command = "AT+QSIMSTAT=1\r"; 	// Enable SIM Inserted Status Report
		reply = sendAT(command);

		command = "AT+QGPSXTRA=0\r"; 	// Disable XTRA
		reply = sendAT(command);
	}
	catch(const exception& e)
	{
		ROS_WARN_STREAM("Error en puerto serial");
		ROS_WARN_STREAM(e.what());
		fd = false;
	}
	
	return 1;
}

int GPS_GSM::deinitModule()
{
    string command = "AT+QGPSEND\r";
    sendAT(command);
	string reply = readSerial();
	// ROS_DEBUG_STREAM(reply);
    serialPort.Close();
    return 1;
}

bool GPS_GSM::gpsStatus()
{
    string res = "NO GPS";		
	string reply; 

	serialPort.Write("AT+QGPS?\r");
	reply = readSerial();	
    vector<string> reply_split = split(reply, '\n');
    for(int i=0;i<reply_split.size();i++)
    {
		if(reply_split[i].size() > 5)
		{
			try
			{
				vector<string> split_1 = split(reply_split[i],' ');
				if(stoi(split_1.at(1)) == 1)
				{
					return true;
				}
				else 
				{
					return false;
				}					
			}
			catch(...)
			{
				return false;
			}
		}
    } 
    return false;  
}

void GPS_GSM::getPosicion()
{
	regex error_regex("\\+CME ERROR:", regex_constants::ECMAScript | regex_constants::icase);
	regex success_regex("\\+QGPSLOC:", regex_constants::ECMAScript | regex_constants::icase);

	string reply = sendAT("AT+QGPSLOC?\r");
	try
	{
		vector<string> reply_split = split(reply, '\n'); 
		for(int i=0;i<reply_split.size();i++)
		{
			if (regex_search(reply_split[i], error_regex)) {
				vector<string> split_1 = split(reply_split[i],' ');
				string str = split_1.at(2);
				string::iterator end_pos = remove(str.begin(), str.end(), ' ');
				str.erase(end_pos, str.end());
				int err = stoi(str);
				if (err == 505)
				{
					dgnst.session = true;
				}
				else if (err == 516)
				{
					dgnst.fixed = false;
				}
			} else if (regex_search(reply_split[i], success_regex)) {
				vector<string> split_1 = split(reply_split[i],' ');
				vector<string> split_2 = split(split_1[1],',');
				getUTCTime(split_2[0]);
				getLatitud(split_2[1]); 
				getLongitud(split_2[2]);
				getAccuracy(split_2[3]);
				getAltitud(split_2[4]);
				getUTCDate(split_2[9]);
				data.satellites = getSatellites(split_2[10]);
				
				if(data.latValid == true and data.longValid == true)
				{
					if(data.latitud != 0 and data.longitud != 0)
					{
						data.gpsValid = true;
						dgnst.fixed = true;
					}
					else
					{
						data.gpsValid = false;
						dgnst.fixed = false;
					}
				}
				else
				{
					data.gpsValid = false;
					dgnst.fixed = false;
				}
			}
		}
	}catch(...){
		ROS_WARN_STREAM("getPosicion() error. Not fatal. Just ignore it.");
    }
}

void GPS_GSM::getUTCDate(string date)
{ 
    data.UTCDate = date;
}

int GPS_GSM::getSatellites(string satellites)
{
    try
    {
		if(satellites.size() > 0)
			return stoi(satellites);
		else
			return 0;
    }
    catch(...)
    {
		return 0;
    }	
}

void GPS_GSM::getAccuracy(string hdop)
{
    data.accuracy = stod(hdop);
}

void GPS_GSM::getUTCTime(string utc){  
    if(utc.size() > 0)
    {
		vector<string> split_utc = split(utc,'.');
		data.UTCTime = split_utc.at(0);
    }
    else
    {
		data.UTCTime = "0";
    }
}

void GPS_GSM::getLatitud(string latitude){
    if(latitude.size() > 0)
    {
		double decimal;
		data.latHemisphere = latitude.back();    
		vector<string> split_2 = split(latitude,'.');
		string grados_str = split_2.at(0).substr(0,split_2.at(0).size()-2);
		string minutos_str = split_2.at(0).substr(split_2.at(0).size()-2,split_2.at(0).size());
		string segundos_str = "." + split_2.at(1).substr(0,split_2.at(1).size()-1);
		double grados = atof(grados_str.c_str());
		double minutos = atof(minutos_str.c_str());
		double segundos = atof(segundos_str.c_str());
		decimal = grados + minutos/60 + segundos*60/3600;
		if (data.latHemisphere == "S") decimal *= -1;
		data.latitud = decimal;
		data.latValid = true;
    }
    else
    {
		data.latValid = false;
    }
}

void GPS_GSM::getLongitud(string longitud){
    if(longitud.size() > 0)
    {
		double decimal;
		char long_dir = longitud.back();
		data.lonHemisphere = longitud.back();
		vector<string> split_2 = split(longitud,'.');
		string grados_str = split_2.at(0).substr(0,split_2.at(0).size()-2);
		string minutos_str = split_2.at(0).substr(split_2.at(0).size()-2,split_2.at(0).size());
		string segundos_str = "." + split_2.at(1).substr(0,split_2.at(1).size()-1);
		double grados = atof(grados_str.c_str());
		double minutos = atof(minutos_str.c_str());
		double segundos = atof(segundos_str.c_str());
		decimal = grados + minutos/60 + segundos*60/3600;
		if (data.lonHemisphere == "W") decimal *= -1;
		data.longitud = decimal;
		data.longValid = true;
    }
    else
    {
		data.longValid = false;
    }
}

void GPS_GSM::getAltitud(string altitud){	
    if(altitud.size() > 0)
    {
		const char *alt = altitud.c_str();
		data.altitud = atof(alt);
    }
}

void GPS_GSM::simInserted(){
	string reply = sendAT("AT+QSIMSTAT?\r");
	if(reply.size() > 24)
    {
		try
		{
			vector<string> split_1 = split(reply,'\n');    
			vector<string> split_2 = split(split_1.at(1),' ');
			vector<string> split_3 = split(split_2.at(1),',');
			char inserted = split_3.at(1)[0];
			if(inserted == '1'){
				data.sim = 1;
				dgnst.sim = true;
			}
			else
			{
				data.sim = 0;
				dgnst.sim = false;
			}
		} catch(...){
			ROS_WARN_STREAM("simInserted() error. Not fatal. Just ignore it.");
			data.sim = 0;
			dgnst.sim = false;
		}
	}
    else
		data.sim = 0;
}

void GPS_GSM::getCalidad(){
	string reply = sendAT("AT+CSQ\r");
    simInserted();    
    if(data.sim > 0){
		if(reply.size() > 10)
		{
			try
			{
				vector<string> split_1 = split(reply,' ');
				vector<string> split_2 = split(split_1.at(1),',');
				data.rssi = stoi(split_2.at(0));
			}
			catch(...)
			{
				ROS_WARN_STREAM("getCalidad() error. Not fatal. Just ignore it.");
				data.rssi=0;
			}
		}
		else
			data.rssi = 200;
		}
		else
			data.rssi = 200; 
		
		if(data.rssi == 200)
		{
			data.rssiDBM=0;
			data.rssiPercentage=0;
		}
		else
		{
		
		float originalDBM = data.rssi;
		float aproxDBM = 0;
		if (data.rssi == 0)
			aproxDBM = -114;
		else if (data.rssi == 1)
			aproxDBM = -111;
		else if (data.rssi >= 2 && data.rssi <= 30)
			aproxDBM = (originalDBM - 2) / (30 - 2) * (-53 - (-109)) + (-109);
		else if (data.rssi >= 102 && data.rssi <= 190)
			aproxDBM = (data.rssi - 102) / (190 - 102) * (-26 - (-114)) + (-114);
		else if (data.rssi == 31)
			aproxDBM = -60;
		else if (data.rssi == 99)
			aproxDBM = -200;
		else if (data.rssi == 100)
			aproxDBM = -116;
		else if (data.rssi == 101)
			aproxDBM = -115;
		else if (data.rssi == 191)
			aproxDBM = -25;
		else if (data.rssi >= 199)
			aproxDBM = -200;
		
		data.rssiDBM = aproxDBM;
		int rssiPerc = aproxDBM*1.0989+127.4725;
		data.rssiPercentage=rssiPerc;
    }    
}

void GPS_GSM::getNetworkInformation(){
	string reply = sendAT("AT+QNWINFO\r");
	// ROS_DEBUG_STREAM(reply);
	try
	{
		vector<string> reply_split = split(reply, '\n');
		for(int i=0;i<reply_split.size();i++)
		{
			if(reply_split[i].size() > 20)
			{
				vector<string> split_1 = split(reply_split[i],':');
				vector<string> split_2 = split(split_1[1],',');
				// data.provider = split_2.at(1).c_str();
				string provider_full = split_2.at(1).c_str();
				string provider = provider_full.substr(1,6);
				if (provider == "310410"){
					data.provider = "AT&T";
				}
				else if (provider == "310260"){
					data.provider = "T-Mobile";
				}
				else {
					data.provider = "";
				}
				data.tech = split_2.at(0).c_str();
				data.band = split_2.at(2).c_str();
			}
		}
	} catch(...){
		ROS_WARN_STREAM("getNetworkInformation() error. Not fatal. Just ignore it.");
    }

}

void GPS_GSM::getServingCell(){
	string reply = sendAT("AT+QENG=\"servingcell\"\r");
	// ROS_DEBUG_STREAM(reply);
    try {
		vector<string> split_1 = split(reply,'\n');
		vector<string> split_2 = split(split_1.at(1),' ');
		vector<string> split_3 = split(split_2.at(1),',');
		/* 
		info.mcc = split_3.at(4);
		info.mnc = split_3.at(5);
		info.cellid = split_3.at(6); */
		if (split_3.at(2) == "\"LTE\""){
			data.rssi = stoi(split_3.at(15));
		} 
    } catch(...){
		ROS_WARN_STREAM("getServinCell() error. Not fatal. Just ignore it.");
    }
}

void GPS_GSM::gpsSession(){
	string reply = sendAT("AT+QGPS?\r");
	try{
		vector<string> split_1 = split(reply,'\n');
		vector<string> split_2 = split(split_1.at(1),' ');
		string str = split_2.at(1);
		string::iterator end_pos = remove(str.begin(), str.end(), ' ');
		str.erase(end_pos, str.end());
		int gss_int = stoi(str);
		if (gss_int == 1){
			dgnst.session = true;
		}else{
			dgnst.session = false;
		}
	}catch(...){
		ROS_WARN_STREAM("gpsSession() error. Not fatal. Just ignore it.");
	}
}