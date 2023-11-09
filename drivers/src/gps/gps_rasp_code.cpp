#include <stdio.h>
#include <vector>
#include <sstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <math.h>     
#include <ctime>  

#include <drivers/gps_msg.h>
#include <std_srvs/SetBool.h>
#include <brainmodules/state_msg.h>
#include <brainmodules/module_status_service.h>
#include <brainmodules/interval_msg.h>

#include "classes/structs/EstadoTeleops.h"
#include "classes/gps/GPS_GSM_v2.h"	
#include "libraries/json.hpp"

#define DTTMFMT "%Y-%m-%d %H:%M:%S "
#define DTTMSZ 21

using namespace std;
using json = nlohmann::json;

GPS_GSM gps;
bool random_imei = false;
bool real_imei = false;
string node_name;

/*Publisher object for gps topic. Path to execute ping messages*/
void publishGPSTopicMessage(ros::Publisher pub)
{	
	drivers::gps_msg message;
	if(gps.fd>=0)
	{	
		ROS_DEBUG_STREAM("getCalidad()");
		gps.getCalidad();
		if(gps.data.sim < 0)
		    gps.data.rssi += 300;
		ROS_DEBUG_STREAM("getPosicion()");
		gps.getPosicion();
		ROS_DEBUG_STREAM("getNetworkInformation()");
		gps.getNetworkInformation();
		ROS_DEBUG_STREAM("getServingCell()");
		gps.getServingCell();
		
		message.validGPS=gps.data.gpsValid;
		message.Latitude=gps.data.latitud;
		message.Longitude=gps.data.longitud;
		message.Altitude=gps.data.altitud;
		message.Sim=gps.data.sim;
		message.rssi=gps.data.rssiPercentage;
		message.LatHemisphere = gps.data.latHemisphere;
		message.LonHemisphere = gps.data.lonHemisphere;
		message.Accuracy = gps.data.accuracy;
		message.UTCTime = gps.data.UTCTime;
		message.UTCDate = gps.data.UTCDate;
		message.Satellites = gps.data.satellitesGSV;
		message.provider = gps.data.provider;
		message.accessTechnology = gps.data.tech;
		message.band = gps.data.band;
		message.ModuleConnected = true;
	}
	else
	{
		message.validGPS=false;
		message.Latitude=0;
		message.Longitude=0;
		message.Altitude=0;
		message.Sim=0;
		message.rssi=0;
		message.LatHemisphere = "";
		message.LonHemisphere = "";
		message.Accuracy = 0;
		message.UTCTime = "";
		message.UTCDate = "";
		message.Satellites = 0;
		message.ModuleConnected = false;
	}
	
	pub.publish(message);
}

bool moduleStatus(brainmodules::module_status_service::Request& request,brainmodules::module_status_service::Response& response)
{
	
	response.status = gps.status;		// Initial condition	
	
	if(gps.status)
	{		
		//~ ROS_DEBUG_STREAM(" Module connected");
		response.imei = gps.data.imei;		// Updated imei
		gps.simInserted();		
		response.sim = gps.data.sim;		// Updated sim status
		gps.getPosicion();		
		response.longitude = gps.data.longitud;	// Updated coordinates
		response.latitude = gps.data.latitud;
		response.rssi = gps.data.rssi;
	}
	else
	{
		ROS_WARN_STREAM(" Module not connected");	
		response.imei = gps.data.imei;
		response.sim = 0;		
		response.longitude = 0;
		response.latitude = 0;
		response.rssi = 0;
	}
		
	return true;
}

int main( int argc, char**argv){ 
	
    ros::init(argc,argv,"gpsNode");
    node_name = ros::this_node::getName	();
    ros::NodeHandle nh;
    drivers::gps_msg message;
    string path = ros::package::getPath("brainmodules")+"/files";
    string node_name = ros::this_node::getName();
    int log_level;
    nh.param("log_level", log_level,0);
    ros::console::levels::Level console_level;
    console_level = (ros::console::levels::Level)log_level;
    ROS_DEBUG_STREAM(node_name << " --- Level: " << console_level);
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::Rate rate(0.33);
    
    //Services for one lecture request	
    ros::ServiceServer status = nh.advertiseService("module_status_service",moduleStatus);
        
    //Broadcast topic
    ros::Publisher pubGpsData = nh.advertise<drivers::gps_msg>("sensor_topic/gps", 10);
    
    gps.initModule();
    
	if(gps.fd<0)
		ROS_ERROR_STREAM("Error: GSM/GPS fd < 0");

	while(ros::ok())
	{	
		publishGPSTopicMessage(pubGpsData);		
		rate.sleep();
		ros::spinOnce();
    }
          
    gps.deinitModule(); 
    return 0;
}
