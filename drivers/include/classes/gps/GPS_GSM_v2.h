#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <sstream>
#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h> 
#include <fstream>
#include <iostream>
#include <thread>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <wiringSerial.h>
#include <unistd.h>
#include <bits/stdc++.h> 

int wait = 1;

using namespace std;

class GPS_GSM
{
    public:
	struct module_data
	{
	    int sim=0;
	    int rssi=0,rssiDBM=0,rssiPercentage=0;
	    double longitud=0;
	    double latitud=0;
	    double altitud=0;
	    string latHemisphere;
	    string lonHemisphere;
	    string imei = "";
	    string UTCTime = "";
	    string UTCDate = "";
	    double accuracy = 0;
	    int satellites = 0;
	    string iccid = "";    
	    string provider = "";
	    string tech = "";
	    string band = "";
	    bool latValid=false, longValid=false, gpsValid=false;
	    int satellitesGGA = 0, satellitesGSV = 0;
	    vector<string> satellitesSNR,satellitesPRN,satellitesUsed, satelliteInfo;
	}data;
	int fd;
	int status;
	
	
	int initModule();
	int deinitModule();
	void getPosicion();
	vector<string> splitString(string position, char delimeter);
	void getLatitud(string latitude);
	void getLongitud(string longitud);
	void getAltitud(string altitud);
	void simInserted();
	void enableSIMReport();
	void disableXtra();
	void getCalidad();
	void getImei();
	void getICCID();
	void getUTCTime(string utc);
	void getUTCDate(string date);
	void getAccuracy(string hdop);
	int getSatellites(string satellites);
	bool gpsStatus();
	void getNetworkInformation();
	void getServingCell();

	void getGGA();
	void getGSA();
	void getRMC();
	void getGSV();
	void getSatellitesInfo();
	bool utf8_check_is_valid(const string& string);
	int sendAT(char* command,int fd);
};

int GPS_GSM::sendAT(char* command,int fd) {
    size_t cmdlen = strlen(command);
    //~ ROS_DEBUG_STREAM("Command: " << command);

    int n = write(fd,command, cmdlen);
    if (n != cmdlen)
    {
      ROS_DEBUG_STREAM("Error in command length: " << n);
      return -1;
    }
    ros::Duration(0.3).sleep();
    char reply[1024];
    //~ ROS_DEBUG_STREAM("Reading buffer...");
    n = read(fd,reply,sizeof(reply));
    reply[n] = '\0' ;
    //~ ROS_DEBUG_STREAM("Reply: " << reply);
    string str(reply);
    vector<string> split = splitString(str,'\n');
    for(int i=0;i<split.size();i++)
    {
	//~ if(split[i].size()==3)
	    //~ ROS_DEBUG_STREAM(split[i]);
	//~ if(split[i]=="OK\0")
	    //~ ROS_DEBUG_STREAM(split[i]);
    }
    return 0;
}

/*
 * name: init_module()
 * @return 1: module physically connected 0: module physically not connected
 */
int GPS_GSM::initModule(){
    //~ ROS_DEBUG_STREAM("Initializing module...");
    //~ fd = open("/dev/ttyUSB2",O_RDWR | O_NOCTTY ); 
    fd = serialOpen("/dev/ttyUSB2",115200); 
    ros::Duration(0.25).sleep();
    
    if(fd > 0){
	char* command;
	command = "AT+QGPS=1\r";	// Turn on GNSS engine
	status = 1;
	sendAT(command,fd);
	command = "AT+QSIMSTAT=1\r"; 	// Enable SIM Inserted Status Report
	sendAT(command,fd);
	command = "AT+QGPSXTRA=0\r"; 	// Disable XTRA
	sendAT(command,fd);
	
	return 1;
    }
    status = 0; //module not connected
    return 0;
}

int GPS_GSM::deinitModule(){
    char* command = "AT+QGPSEND\r";
    sendAT(command,fd);
    close(fd);
    return 1;
}

bool GPS_GSM::gpsStatus(){
    string res = "NO GPS";
    char response[1024];
    char write_posicion[] = "AT+QGPS?\r";
    write(fd,write_posicion,sizeof(write_posicion));
    int n = read(fd,&response,sizeof(response));
    //~ ROS_DEBUG_STREAM("Response: " << response << "\n=========");
    vector<string> response_split = splitString(response, '\n');
    for(int i=0;i<response_split.size();i++)
    {
	if(response_split[i].size() > 5)
        {
	    try
	    {
		vector<string> split_1 = splitString(response_split[i],' ');
		if(stoi(split_1.at(1)) == 1)
		    return true;
		else
		    return false;
	    }
	    catch(...)
	    {
		return false;
	    }
        }
    } 
    return false;  
}

void GPS_GSM::getPosicion(){
    char response[1024];
    char write_posicion[] = "AT+QGPSLOC?\r";
    write(fd,write_posicion,sizeof(write_posicion));
    sleep(wait);
    int n = read(fd,&response,sizeof(response));   
    response[n] = '\0' ;
    try{
		vector<string> response_split = splitString(response, '\n'); 
		for(int i=0;i<response_split.size();i++)
		{
			if(response_split[i].size() > 70 and response_split[i].size() < 100)
			{
			vector<string> split_1 = splitString(response_split[i],' ');
			vector<string> split_2 = splitString(split_1[1],',');
			getUTCTime(split_2[0]);
			getLatitud(split_2[1]); 
			getLongitud(split_2[2]);
			getAccuracy(split_2[3]);
			getAltitud(split_2[4]);
			getUTCDate(split_2[9]);
			data.satellites = getSatellites(split_2[10]);
			
			if(data.latValid == true and data.longValid == true)
				if(data.latitud != 0 and data.longitud != 0)
				data.gpsValid = true;
				else
				data.gpsValid = false;
			else
				data.gpsValid = false;
			}
		}
    }catch(...){
		ROS_WARN_STREAM("getPosicion() error. Not fatal. Just ignore it.");
    }
}

void GPS_GSM::getUTCDate(string date){ 
    data.UTCDate = date;
}

int GPS_GSM::getSatellites(string satellites){
    //~ data.satellites = stoi(satellites);
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

void GPS_GSM::getAccuracy(string hdop){  
    /*double acc = stod(hdop);
    acc += 0.5;
    data.accuracy = (int) acc;*/
    data.accuracy = stod(hdop);
}

void GPS_GSM::getUTCTime(string utc){  
    if(utc.size() > 0)
    {
	vector<string> split = splitString(utc,'.');
	data.UTCTime = split.at(0);
	
    }
    else
    {
	data.UTCTime = "0";
    }
}

vector<string> GPS_GSM::splitString(string position, char delimeter){
    //~ ROS_DEBUG_STREAM("A");
    stringstream ss(position);
    //~ ROS_DEBUG_STREAM("B");
    string item;
    vector<string> split_position;
    while (getline(ss, item, delimeter))
    {
    //~ ROS_DEBUG_STREAM("C");
       split_position.push_back(item);
    }
    //~ ROS_DEBUG_STREAM("D");
    return split_position;
}

void GPS_GSM::getLatitud(string latitude){
    if(latitude.size() > 0)
    {
	double decimal;
	data.latHemisphere = latitude.back();    
	vector<string> split_2 = splitString(latitude,'.');
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
	vector<string> split_2 = splitString(longitud,'.');
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
    char response[1024];
    char write_posicion[] = "AT+QSIMSTAT?\r"; //Query SIM Inserted Status 
    write(fd,write_posicion,sizeof(write_posicion));
    sleep(wait);
    int n = read(fd,&response,sizeof(response));
    response[n] = '\0' ;
    string str(response);
    if(str.size() > 24)
    {
		try{
			vector<string> split_1 = splitString(response,'\n');    
			vector<string> split_2 = splitString(split_1.at(1),' ');
			vector<string> split_3 = splitString(split_2.at(1),',');
			char inserted = split_3.at(1)[0];
			if(inserted == '1') 
				data.sim = 1;
			else 
				data.sim = 0;
		} catch(...){
			ROS_WARN_STREAM("simInserted() error. Not fatal. Just ignore it.");
			data.sim = 0;
		}
    }
    else
    {
		data.sim = 0;
    }
}

void GPS_GSM::getCalidad(){
    char response[1024];
    char write_buffer[] = "AT+CSQ\r";
    write(fd,write_buffer,sizeof(write_buffer));
    sleep(wait);
    int n = read(fd,&response,sizeof(response));
    response[n] = '\0' ;
    string str(response);
    simInserted();    
    if(data.sim > 0){
	//~ if(str.size() > 10 & str.size() <15)
	if(str.size() > 10)
	{
	    try
	    {		
		vector<string> split_1 = splitString(str,' ');
		vector<string> split_2 = splitString(split_1.at(1),',');
		data.rssi = stoi(split_2.at(0));
		//~ ROS_DEBUG_STREAM("data.rssi: " << data.rssi);
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
	    aproxDBM = -51;
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
    char response[1024];
    char write_buffer[] = "AT+QNWINFO\r";
    write(fd,write_buffer,sizeof(write_buffer));
    sleep(wait);
    int n = read(fd,&response,sizeof(response));
    response[n] = '\0' ;
    string str(response);
    try
	{
		vector<string> response_split = splitString(str,'\n');
		if(response_split.size() > 0)
		{
			for(int i=0;i<response_split.size();i++)
			{	
				if(response_split[i].size() > 21)
				{
					vector<string> split_1 = splitString(response_split[i],':');
					vector<string> split_2 = splitString(split_1[1],',');
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
		} else {
			ROS_WARN_STREAM("getNetworkInformation() error. Not fatal. Just ignore it.");
		}
    } catch(...){
		ROS_WARN_STREAM("getNetworkInformation() error. Not fatal. Just ignore it.");
    }
}

/*
 * name: get_imei
 * @return imei is stored in object property
 */
void GPS_GSM::getImei(){
    string imei_temp = "";
    char command[] = "AT+GSN\r";
    char response[1024];
    int maxTries=15;
    bool n = true;
    int l,i=0;
    data.imei="";
    
    do{	
	write(fd,command,sizeof(command));
	l = read(fd,&response,sizeof(response));
	vector<string> split_1 = splitString(response,'\n');
	for(int i=0;i<split_1.size();i++)
	{
	    if(split_1.at(i).size() == 16 or split_1.at(i).size() == 15 and split_1.at(i) != "+CME ERROR: 504")
	    {
		try
		{
		    long long ll = stoll(split_1.at(i));
		    imei_temp = split_1.at(i);
		    //~ ROS_INFO_STREAM("Valid IMEI argument :D" );
		}
		catch(const invalid_argument)
		{
		    ROS_WARN_STREAM("Invalid IMEI argument D:" );
		}		
		n = false;
	    }	    
	}
	if(i>=maxTries)
	{
	    n=false;
	    data.imei="";
	    return;
	}
	i++;
    }while(n);
    stringstream ss;
    for(int i=0; i<imei_temp.length();i++){
	    ss << imei_temp[i];		
    }    
    ss >> data.imei;
}

/* ICCID mide 20 digitos */
void GPS_GSM::getICCID(){
    string iccid = "";
    char command[] = "AT+QCCID\r";
    char response[1024];
    int maxTries=500;
    bool n = true;
    int l,i=0;
    do{	
	sleep(0.4);
	//~ ROS_DEBUG_STREAM("GPS retreiving SIM iccid...");
	write(fd,command,sizeof(command));
	l = read(fd,&response,sizeof(response));
	vector<string> split_1 = splitString(response,'\n');
	//~ ROS_INFO_STREAM(response);
	for(int i=0;i<split_1.size();i++)
	{
	    if(split_1.at(i).size() > 20 and split_1.at(i) != "+CME ERROR: 504")
	    {
		vector<string> split_2 = splitString(response,' ');
		iccid = split_2.at(1);
		n = false;
	    }	    
	}
	if(i>=maxTries)
	{
	    n=false;
	    iccid="";
	}
	i++;
    }while(n);
    stringstream ss;
    for(int i=0; i<iccid.length();i++){
	    ss << iccid[i];		
    }    
    ss >> data.iccid;
}

/* Essential fix data */
void GPS_GSM::getGGA(){
     //~ ROS_DEBUG_STREAM("getGGA");
    char response[1024];
    char write_posicion[] = "AT+QGPSGNMEA=\"GGA\"\r";
    write(fd,write_posicion,sizeof(write_posicion));
    int n = read(fd,&response,sizeof(response));
    vector<string> response_split = splitString(response, '\n');
    for(int i=0;i<response_split.size();i++)
    {
	if(response_split[i].size() > 30)
        {
	    try
	    {
		vector<string> split_1 = splitString(response_split[i],' ');
		vector<string> split_2 = splitString(split_1[1],',');
		data.satellitesGGA = getSatellites(split_2[7]);
		//~ ROS_DEBUG_STREAM("data.satellitesGGA: " << data.satellitesGGA);
	    }catch(...){
		ROS_ERROR_STREAM("Error in GGA sentence parcing!");
	    }
        }
	else
	{
	    //~ ROS_WARN_STREAM("No GGA sentence!");
	}
    }
}

/* Recommended Minimum: essential gps pvt (position, velocity, time) data*/
void GPS_GSM::getRMC(){
    char response[1024];
    char write_posicion[] = "AT+QGPSGNMEA=\"RMC\"\r";
    write(fd,write_posicion,sizeof(write_posicion));
    int n = read(fd,&response,sizeof(response));
    vector<string> response_split = splitString(response, '\n'); 
    //~ ROS_ERROR_STREAM(response); 
    for(int i=0;i<response_split.size();i++)
    {
        if(response_split[i].size() > 10 and response_split[i].size() < 20)
        {
	    //~ ROS_ERROR_STREAM("RMC not available"); 
        }
        else if(response_split[i].size() > 30)
        {	    
            vector<string> split_1 = splitString(response_split[i],' ');
	    //~ ROS_ERROR_STREAM(response_split[i]);
            vector<string> split_2 = splitString(split_1[1],',');
	    getUTCTime(split_2[1]);
	    string l;
	    l = split_2[3] + split_2[4];
            getLatitud(l);
	    l = split_2[5] + split_2[6];
            getLongitud(l);
	    data.accuracy = 0;         
	    getUTCDate(split_2[9]);
	    data.satellites = '0';
	    data.altitud = 0;
        }
    }
}

/* Detailed Satellite data. SNR (Signal to Noise Ratio, 0-99): higher is better*/
void GPS_GSM::getGSV(){
    //~ ROS_DEBUG_STREAM("getGSV");
    char response[1024];
    char write_posicion[] = "AT+QGPSGNMEA=\"GSV\"\r";
    write(fd,write_posicion,sizeof(write_posicion));    
    ros::Duration(0.3).sleep();
    int n = read(fd,&response,sizeof(response));
    //~ ROS_DEBUG_STREAM(response);
    vector<string> response_split = splitString(response, '\n'); 
    bool satellites = false;
    int satellitesPos,prnPos;
    vector<string> satellitesSNR, satellitesPRN;
    for(int i=0;i<response_split.size();i++)
    {
	//~ ROS_WARN_STREAM(response_split[i]);
	satellitesPos = 3;
	prnPos = 4;
	if(response_split[i].size() > 20)
	{	    
	    //~ ROS_DEBUG_STREAM(i << ": " << response_split.at(i));
	    vector<string> split_1 = splitString(response_split[i],' ');
	    vector<string> split_2 = splitString(split_1[1],',');
	    if(!satellites)
	    {
		data.satellitesGSV = getSatellites(split_2[satellitesPos]);
		//~ ROS_DEBUG_STREAM("Satellites in view: " << data.satellitesGSV);
		satellites = true;
	    }
	    satellitesPos += 4;
	    while( satellitesPos < split_2.size())
	    {
		//PRN of satellites
		satellitesPRN.push_back(split_2.at(prnPos));
		prnPos += 4;
		
		// SNR of satellites
		if(split_2.at(satellitesPos).size() > 2)
		{
		    vector<string> split_3 = splitString(split_2.at(satellitesPos),'*');
		    satellitesSNR.push_back(split_3.at(0));
		}
		else
		{
		    satellitesSNR.push_back(split_2.at(satellitesPos));
		}
		satellitesPos += 4;
	    }
	}
	else
	{
	    //~ ROS_WARN_STREAM("No GSV sentence!");
	}
    }
    data.satellitesSNR = satellitesSNR;
    data.satellitesPRN = satellitesPRN;
    /*for(int i=0;i<satellitesSNR.size();i++)
    {
	ROS_DEBUG_STREAM(i << "-> PRN: " << satellitesPRN.at(i) << " SNR: " << satellitesSNR.at(i));
    }*/
}

/* This sentence provides details on the nature of the fix. It includes the numbers of the satellites being used in the current solution and the DOP (dilution of precision) */
void GPS_GSM::getGSA(){
    ROS_DEBUG_STREAM("getGSA()");
    char response[1024];
    char write_posicion[] = "AT+QGPSGNMEA=\"GSA\"\r";    
    write(fd,write_posicion,sizeof(write_posicion));    
    int n = read(fd,&response,sizeof(response));
    ROS_DEBUG_STREAM("==========\n" << response << "\n==========");
    /*
    ROS_DEBUG_STREAM("splitString");
    vector<string> response_split = splitString(response, '\n');         
    vector<string> res;
    data.satellitesUsed.clear();
    for(int i=0;i<response_split.size();i++)
    {
	if(response_split[i].size() > 20)
	{	    
	    ROS_DEBUG_STREAM(response_split.at(i));
	    vector<string> split_1 = splitString(response_split[i],' ');
	    vector<string> split_2 = splitString(split_1[1],',');
	    vector<string>::const_iterator first = split_2.begin() + 3;
	    vector<string>::const_iterator last = split_2.begin() + 15;
	    vector<string> satellitesUsed(first,last);
	    for(int i=0;i<satellitesUsed.size();i++)
	    {
		if(satellitesUsed.at(i) != "")
		    data.satellitesUsed.push_back(satellitesUsed.at(i));
	    }
	}
    }
    */
}

void GPS_GSM::getSatellitesInfo()
{
    /*
    ROS_DEBUG("Getting GGA");    
    getGGA();
    */
    
    //~ ROS_DEBUG("Getting GSV");
    getGSV();
    
    /*
    data.satelliteInfo.clear();
    if(data.satellitesGGA != 0)
    {
	ROS_DEBUG("Getting GSA");
	getGSA();
	for(int i=0;i<data.satellitesUsed.size();i++)
	    ROS_DEBUG_STREAM(data.satellitesUsed.at(i));
	if(data.satellitesUsed.size() > 0)
	{
	    ROS_DEBUG_STREAM("Potencias:");
	    for(int i=0;i<data.satellitesUsed.size();i++)
	    {
		ROS_DEBUG_STREAM("Satellite: " << data.satellitesUsed.at(i));
		ptrdiff_t pos =find(data.satellitesPRN.begin(),data.satellitesPRN.end(), data.satellitesUsed.at(i)) - data.satellitesPRN.begin();
		if(!(pos >= data.satellitesPRN.size()))
		{
		    ROS_DEBUG_STREAM("ID: " << data.satellitesUsed.at(i) << " SNR: " << data.satellitesSNR.at(pos));
		    data.satelliteInfo.push_back(data.satellitesUsed.at(i));
		    data.satelliteInfo.push_back(data.satellitesSNR.at(pos));
		}
	    }
	    
	    for(int i=0;i<data.satelliteInfo.size();i++)
		ROS_DEBUG_STREAM(data.satelliteInfo.at(i));
	}
    }
    */
}

bool GPS_GSM::utf8_check_is_valid(const string& string)
{
    int c,i,ix,n,j;
    for (i=0, ix=string.length(); i < ix; i++)
    {
        c = (unsigned char) string[i];
        //if (c==0x09 || c==0x0a || c==0x0d || (0x20 <= c && c <= 0x7e) ) n = 0; // is_printable_ascii
        if (0x00 <= c && c <= 0x7f) n=0; // 0bbbbbbb
        else if ((c & 0xE0) == 0xC0) n=1; // 110bbbbb
        else if ( c==0xed && i<(ix-1) && ((unsigned char)string[i+1] & 0xa0)==0xa0) return false; //U+d800 to U+dfff
        else if ((c & 0xF0) == 0xE0) n=2; // 1110bbbb
        else if ((c & 0xF8) == 0xF0) n=3; // 11110bbb
        //else if (($c & 0xFC) == 0xF8) n=4; // 111110bb //byte 5, unnecessary in 4 byte UTF-8
        //else if (($c & 0xFE) == 0xFC) n=5; // 1111110b //byte 6, unnecessary in 4 byte UTF-8
        else return false;
        for (j=0; j<n && i<ix; j++) { // n bytes matching 10bbbbbb follow ?
            if ((++i == ix) || (( (unsigned char)string[i] & 0xC0) != 0x80))
                return false;
        }
    }
    return true;
}

void GPS_GSM::getServingCell(){
    char response[1024];
    char write_buffer[] = "AT+QENG=\"servingcell\"\r";
    write(fd,write_buffer,sizeof(write_buffer));
    sleep(wait);
    int n = read(fd,&response,sizeof(response));
    response[n] = '\0' ;
    string str(response);
    //~ ROS_DEBUG_STREAM(str);	
    //~ ROS_DEBUG_STREAM(str.size());	
    try {
	vector<string> split_1 = splitString(str,'\n');
	vector<string> split_2 = splitString(split_1.at(1),' ');
	vector<string> split_3 = splitString(split_2.at(1),',');
	//~ info.mcc = split_3.at(4);
	//~ info.mnc = split_3.at(5);
	//~ info.cellid = split_3.at(6);
	//~ ROS_DEBUG_STREAM(split_3.at(2));
	if (split_3.at(2) == "\"LTE\""){
	    data.rssi = stoi(split_3.at(15));
	    //~ ROS_DEBUG_STREAM("data.rssi: " << data.rssi);
	} 
    } catch(...){
	ROS_WARN_STREAM("getServinCell() error. Not fatal. Just ignore it.");
    }
}
