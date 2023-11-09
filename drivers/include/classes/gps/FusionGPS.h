#include <string>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <fstream>

#include "drivers/gps_msg.h"

using namespace std;

class FusionGPS
{
    public:
        FusionGPS(ros::NodeHandle nh_priv);
        ~FusionGPS();
        void publishGps();
    
    private:
        int log_level, sampleNo, samples;
        string node_name;

        ros::NodeHandle nh;
        ros::Subscriber subGpsA, subGpsB;
        ros::Publisher pubGps;

        drivers::gps_msg gpsDataA, gpsDataB, gpsData;

        ofstream gpsTest;

        void GpsACallback(const drivers::gps_msg& msg);
        void GpsBCallback(const drivers::gps_msg& msg);
};

FusionGPS::FusionGPS(ros::NodeHandle nh_priv)
{
    node_name = ros::this_node::getName();
    nh_priv.param("log_level", log_level,0);
	ros::console::levels::Level console_level;
    console_level = (ros::console::levels::Level)log_level;    
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    nh_priv.param("samples",samples,0);
    
    subGpsA = nh.subscribe("sensor_topic/gps/A", 10, &FusionGPS::GpsACallback, this);
    subGpsB = nh.subscribe("sensor_topic/gps/B", 10, &FusionGPS::GpsBCallback, this);
    pubGps = nh.advertise<drivers::gps_msg>("sensor_topic/gps", 10);

    if(samples)
    {
        time_t t = time(0);
        struct tm * now = localtime(&t);
        char buffer[80];
        strftime(buffer,80,"%Y-%m-%d-%H-%M", now);
        string path = "/home/jetson/Desktop/samples/";
        path += buffer;
        path += "-gps.csv";
        gpsTest.open(path);
        gpsTest << "sampleNo,latA,longA,hdopA,nsatA,latB,longB,hdopB,nsatB\n";
        sampleNo = 0;
    }
}

FusionGPS::~FusionGPS(){
    if(samples)
        gpsTest.close();
}

void FusionGPS::GpsACallback(const drivers::gps_msg& msg)
{
    // ROS_DEBUG_STREAM(node_name << " <--- Gps A Callback");
    gpsDataA = msg;
}

void FusionGPS::GpsBCallback(const drivers::gps_msg& msg)
{
    // ROS_DEBUG_STREAM(node_name << " <--- Gps B Callback");
    gpsDataB = msg;
}

void FusionGPS::publishGps()
{
    if(samples)
    {
        string row = to_string(sampleNo) + "," 
            + to_string(gpsDataA.Latitude) + "," + to_string(gpsDataA.Longitude) + "," + to_string(gpsDataA.Accuracy) + "," + to_string(gpsDataA.Satellites) + ","
            + to_string(gpsDataB.Latitude) + "," + to_string(gpsDataB.Longitude) + "," + to_string(gpsDataB.Accuracy) + "," + to_string(gpsDataB.Satellites) + "\n";
        gpsTest << row;
        sampleNo += 1;
    }

    if(gpsDataA.validGPS)
    {
        ROS_DEBUG_STREAM(node_name << " ---> A");
        gpsData = gpsDataA;
    }
    else if(gpsDataB.validGPS)
    {
        ROS_DEBUG_STREAM(node_name << " ---> B");
        gpsData = gpsDataB;
    }
    else
    {
        ROS_DEBUG_STREAM(node_name << " ---> X");
        gpsData.validGPS=false;
		gpsData.Latitude=0;
		gpsData.Longitude=0;
		gpsData.Altitude=0;
		gpsData.Sim=0;
		gpsData.rssi=0;
		gpsData.LatHemisphere = "";
		gpsData.LonHemisphere = "";
		gpsData.Accuracy = 0;
		gpsData.UTCTime = "";
		gpsData.UTCDate = "";
		gpsData.Satellites = 0;
    }
    pubGps.publish(gpsData);
}

