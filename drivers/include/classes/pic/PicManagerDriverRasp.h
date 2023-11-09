#include <iostream>
#include <string>
#include <vector>
#include <numeric>

#include <wiringPi.h>

#include <ros/console.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <drivers/sensor_picmanager_msg.h>
#include "classes/SPI/SPIDriver.h"

#define RESET_PIN 	2
#define FUENTE_24V	22
#define FUENTE_3V3	23
#define VENTILADOR	30
#define UPS		5

class PicManagerDriver:virtual public SPIDriver
{
	public:
		PicManagerDriver(ros::NodeHandle nh_priv);
		virtual ~PicManagerDriver();
		int Initialize();
		int GetSensors();
		void InitUPS();
		void DeinitUPS();

	private:
		string package_path;
		std::vector<int> battery_data;

		ros::ServiceServer reset_pic_service, force_bootloader_srv, init_program_pic_srv;
		ros::ServiceClient program_pic_srv;
		ros::Publisher pubSensors;
		drivers::sensor_picmanager_msg sensor_msg;

		//srvs
		bool ForceBootloaderSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
		bool Reset_PIC_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
		bool ProgramPICInitialize(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

		int InitResetPIC();
		void ResetPIC();
	    
	    
		void ForceBootloader();
		bool ProgramPICVehicle();
};

PicManagerDriver::PicManagerDriver(ros::NodeHandle nh_priv):SPIDriver(nh_priv)
{
    commandsDir = "/braintemp/wagonIndiaPicManager.json";
    package_path = ros::package::getPath("drivers");

    pubSensors = nh.advertise <drivers::sensor_picmanager_msg>("sensor_topic/picmanager", 10);
    reset_pic_service = nh.advertiseService("reset_pic_service", &PicManagerDriver::Reset_PIC_service, this);
    force_bootloader_srv = nh.advertiseService("force_bootloader", &PicManagerDriver::ForceBootloaderSrv, this);
    init_program_pic_srv = nh.advertiseService("init_program_pic", &PicManagerDriver::ProgramPICInitialize, this);
    program_pic_srv = nh.serviceClient<std_srvs::Trigger>("program_pic");

    int nRet = wiringPiSetup();
    if (nRet!=0)
    {
	ROS_WARN_STREAM(node_name << " --- WiringPiSetup failed");
    }        
}

PicManagerDriver::~PicManagerDriver()
{
    ROS_DEBUG_STREAM("destructor PicManagerDriver");
}

void PicManagerDriver::InitUPS()
{
    pinMode(UPS,OUTPUT);
    digitalWrite(UPS, HIGH);
}

void PicManagerDriver::DeinitUPS()
{
    digitalWrite(UPS, LOW);
}

int PicManagerDriver::Initialize()
{
    int nRet;
    //Initialization for generic driver
    nRet = SPIDriver::Initialize();

    //Initialization for specific driver
    InitResetPIC();
    ResetPIC();       

    return nRet;
}

int PicManagerDriver::InitResetPIC()
{
    pinMode(RESET_PIN,OUTPUT);
    digitalWrite(RESET_PIN, LOW);
    //Inicializacion de puerto de des/activacion de fuentes
    pinMode(FUENTE_24V,OUTPUT);
    pinMode(FUENTE_3V3,OUTPUT);
    pinMode(VENTILADOR,OUTPUT);
    pinMode(4,OUTPUT);
    digitalWrite(FUENTE_24V, HIGH);
    digitalWrite(FUENTE_24V, HIGH);
    digitalWrite(4, HIGH);
    digitalWrite(VENTILADOR, HIGH);

    return 0;
}

void PicManagerDriver::ResetPIC()
{
    digitalWrite(RESET_PIN, HIGH);
    delay(50);
    digitalWrite(RESET_PIN, LOW);
}

bool PicManagerDriver::Reset_PIC_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	ROS_DEBUG_STREAM(node_name << " --- Reset_PIC service called");
	ResetPIC();
	return true;
}

void PicManagerDriver::ForceBootloader()
{
    string force_bootloader_script= std::string{homedir} + "/braintemp/force_bootloader.sh";
    system(force_bootloader_script.c_str());
}

bool PicManagerDriver::ForceBootloaderSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    ForceBootloader();
    return true;
}

bool PicManagerDriver::ProgramPICVehicle()
{
	bool return_value;
	std_srvs::Trigger program_pic_msg;
	SetCommand("DRIVE_ENABLE_DRIVERS", 0);
	ros::Duration(0.5).sleep();
	SetCommand("START_BOOTLOADER", 0);
	ros::Duration(0.5).sleep();
	GetCommand<int>("WAGON_STATUS", 0);
	ros::Duration(0.5).sleep();
	if (program_pic_srv.call(program_pic_msg))
		return_value = program_pic_msg.response.success;
	else
		return_value = false;
	SetCommand("DRIVE_ENABLE_DRIVERS", 1);
	//TODO: Test enable command
	return return_value;
}

bool PicManagerDriver::ProgramPICInitialize(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    res.success = ProgramPICVehicle();
    return true;
}

int PicManagerDriver::GetSensors()
{
	int nRet = 0;
	unsigned long version =  GetCommand<unsigned int>("WAGON_VERSION");
	char * version_ch = (char*) &version;
    string version_str = version_ch;
    bool valid_version = true;
    if ( version_str.size() <=0)
        valid_version = false;
    else 
    {
		for (int i = 0; i < 4 ; i++)
		{
			if (!((version_str.at(i)>=65 && version_str.at(i)<=90)||(version_str.at(i)>=97 && version_str.at(i)<=122)||(version_str.at(i)>=48 && version_str.at(i)<=57)))
			{
				valid_version = false ;
				break;
			}
		}
	}
    if (valid_version)
    {
		sensor_msg.Version = version_str.substr(0,4);
	}
	else
	{
		/** Error de comunicacion SPI */
		sensor_msg.Version = "";

		nRet = -1;
	}
	
	int battery_temp = (int) GetCommand<int>("BATTERY_LEVEL", false);
	 
	//~ ROS_DEBUG_STREAM("BATTERY: " << (unsigned int) GetCommand<unsigned int>("BATTERY_LEVEL"));
	if ((battery_temp >=128)/*&&(battery_temp<=228)*/)
	{
		sensor_msg.Charging=true;
		battery_temp -= 128;
	}
	else
	{
		sensor_msg.Charging=false;
	}
	if ((battery_temp < 0)/*||(battery_temp > 100)*/)
	{
		ROS_WARN_STREAM("Battery value out of range: " << battery_temp);	
		nRet = -1;
	}
	else
	{
		battery_data.push_back(battery_temp);
		if (battery_data.size()>100)
			battery_data.erase(battery_data.begin());
		sensor_msg.BatteryLevel = (int) accumulate(battery_data.begin(), battery_data.end(), 0.0)/ battery_data.size();
	}

    /*Obtencion de errores*/
    //Generales
    sensor_msg.WagonError1 = GetCommand<unsigned int>("WAGON_ERROR_1");
    sensor_msg.WagonError2 = GetCommand<unsigned int>("WAGON_ERROR_2");
    sensor_msg.WagonSensor = GetCommand<unsigned int>("WAGON_SENSOR");

    //Particulares
    sensor_msg.WagonDriveRightError = GetCommand<unsigned int>("WAGON_DRIVE_R_ERROR", false);
    sensor_msg.WagonDriveLeftError = GetCommand<unsigned int>("WAGON_DRIVE_L_ERROR", false);

    pubSensors.publish(sensor_msg);

    return nRet;
}
