#include <thread>

#include "libraries/json.hpp"
#include "libraries/exceptions/Exceptions.h"

#include "../Structs/Sensor.h"
#include "../UDP/ClientConnection.h"
#include "../Structs/EstadoTeleops.h"

#define BUFFER_SENSORES 1024
using namespace std::chrono;
using json = nlohmann::json;

class TeleopsSensor
{	
	public:
		//EstadoTeleops estadoTeleops; 
		
		bool initSensores;
		bool deinitSensores;
		ClientConnection *SensorConnection;
		
		int currentSocketStatus;
		int desiredSocketStatus;
		string imei;
				
		teleoperacion teleops;
		PICStatus picStatus;
		GPSData gpsData;
		
        VideoProcessing	vpData;

		TeleopsSensor();
		//int Initialize();
		void stateMachine();
		
		bool clientConnection, sensorThread, bufferSent;
		
	protected: 
		int port;
		high_resolution_clock::time_point lastSendTime;
		//std::thread *SensorThread;
		//static void TeleopsSensorLoop(TeleopsSensor *sensorTeleops);
};

TeleopsSensor::TeleopsSensor()
{
	initSensores = false;
	deinitSensores = false;
	
	picStatus.batteryLevel=0;
	picStatus.yaw=0;
	picStatus.currentAngularRate=0;
	picStatus.currentSpeed=0;
	picStatus.desiredAngularRate=0;
	picStatus.desiredSpeed=0;
	picStatus.version = "xxx";
	
	gpsData.latitud=0;
	gpsData.longitud=0;
	gpsData.altitud=0;
	gpsData.rssi=0;
	
	currentSocketStatus=EstadoSocket::SOCKET_STOPPED;
	desiredSocketStatus=EstadoSocket::SOCKET_STOPPED;
}

void TeleopsSensor::stateMachine()
{
	
	high_resolution_clock::time_point currentTime;
	duration<double> elapsedSendTime;
	bool startedReceiving=false;
	int imeiLength=15;
	char imeiBuffer[15] = "";

	if(currentSocketStatus==EstadoSocket::SOCKET_STOPPED)
	{
		// ROS_INFO_STREAM("Sensor socket stopped");
		if(desiredSocketStatus==1)
		{
			currentSocketStatus=EstadoSocket::SOCKET_STARTING;
			ros::Duration(0.5).sleep();
			//this_thread::sleep_for(std::chrono::milliseconds(500));
		}
		//else do nothing wait for socket desired state to change
		
	}
	else if(currentSocketStatus==EstadoSocket::SOCKET_STARTING)
	{
		ROS_INFO_STREAM("Initializing sensor socket");
		startedReceiving=false;
		SensorConnection = new ClientConnection( teleops.servidor, teleops.sensores, BUFFER_SENSORES, false);
		int res = SensorConnection->Initialize();
		(res == 0)? clientConnection = true:clientConnection = false;
		if (res == 0)
		{
			//~ sensorTeleops->initSensores = true;
			ROS_INFO_STREAM("Sensor connection starts successfully");				
			currentSocketStatus=EstadoSocket::SOCKET_STARTED;
		}
		else//Error starting socket
		{ 
			//~ sensorTeleops->initSensores = false;
			ROS_INFO_STREAM("Sensor connection failed");
			if(desiredSocketStatus==0)//Abort trying to start socket, desired changed to off
			{
				currentSocketStatus=EstadoSocket::SOCKET_STOPPED;
			}
		}
		ros::Duration(0.5).sleep();
		//this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	else if(currentSocketStatus==EstadoSocket::SOCKET_STARTED)
	{
		ROS_INFO_STREAM("Sensor socket started");
		if(desiredSocketStatus==1)//Started and desired state is on
		{
			if(!imei.empty())
			{
				strncpy(imeiBuffer,imei.c_str(),sizeof(imeiBuffer));
				//sprintf(imeiBuffer,"%s",sensorTeleops->imei.c_str());
				
				currentTime = high_resolution_clock::now();
				elapsedSendTime = duration_cast<duration<double>>(currentTime - lastSendTime);
				char buffer[BUFFER_SENSORES];
				
				json jsonSensor;
				jsonSensor["desiredSpeed"]= picStatus.desiredSpeed;
				jsonSensor["currentSpeed"]= picStatus.currentSpeed;
				jsonSensor["desiredSpeedRight"]= picStatus.desiredSpeedRight;
				jsonSensor["currentSpeedRight"]= picStatus.currentSpeedRight;
				jsonSensor["desiredSpeedLeft"]= picStatus.desiredSpeedLeft;
				jsonSensor["currentSpeedLeft"]= picStatus.currentSpeedLeft;
				jsonSensor["desiredAngle"]= picStatus.desiredAngularRate;
				jsonSensor["currentAngle"]= picStatus.currentAngularRate;
				jsonSensor["networkPower"]= gpsData.rssi;
				jsonSensor["udpt"]= picStatus.udpTimeStamp;
				jsonSensor["qrt"]= picStatus.qrTimeStamp;
				jsonSensor["cc"]= picStatus.lastControlMsg;
				
				jsonSensor["gpsLat"]= gpsData.latitud;
				jsonSensor["gpsLon"]= gpsData.longitud;
				jsonSensor["gpsAlt"]= gpsData.altitud;
				jsonSensor["gpsSatellites"]= gpsData.satellites;
				jsonSensor["batteryLevel"]= picStatus.batteryLevel;
				jsonSensor["batteryVoltage"]= picStatus.batteryVoltage;
				
				jsonSensor["yaw"]= picStatus.yaw;
				jsonSensor["pitch"]= picStatus.pitch;
				jsonSensor["roll"]= picStatus.roll;
				jsonSensor["gpsFlag"]=to_string( gpsData.validGPS?1:0);
				
				jsonSensor["kangarooError"]=(picStatus.errorVector & FLAG_UART_KANGAROO)?1:0;
				jsonSensor["spiError"]=(picStatus.errorVector & FLAG_SPI_COM)?1:0;
				jsonSensor["brainError"]=(picStatus.errorVector & FLAG_BRAIN_INTRN)?1:0;
				jsonSensor["teleopError"]=(picStatus.errorVector & FLAG_TELEOP_COM)?1:0;
				jsonSensor["failSafeMode"]=(picStatus.errorVector & FLAG_SAFE_MODE)?1:0;
				jsonSensor["failSafeRelease"]=(picStatus.errorVector & FLAG_SAFE_RELEASE)?1:0;
				
				// video processing
                //TODO(rbt): Don't send data if it is not necessary.
                jsonSensor["whereiam_id"] = vpData.whereiam_cls_id;
                jsonSensor["whereiam_conf"] = vpData.whereiam_cls_conf;
                jsonSensor["lpt"] = vpData.lpt;
                jsonSensor["rpt"] = vpData.rpt;
                jsonSensor["AP"] = json::object({{"EN", vpData.auto_pilot?1:0}, {"INF", vpData.inferring?1:0}});

				string jsonStr = jsonSensor.dump();
				// ROS_DEBUG_STREAM("Sensor json");
				ROS_DEBUG_STREAM(jsonStr);
				// ROS_DEBUG_STREAM(jsonStr.length());
				
				int i;
				int bufferSize=jsonStr.length(); 
				// ROS_DEBUG_STREAM("JSON lenght: " << bufferSize);
				for (i = 0; i <jsonStr.length(); i++) { 
					buffer[i] = jsonStr[i]; 
	
				}
				buffer[jsonStr.length()]='\0';
				
				
				
				//if (){
				//~ ROS_INFO_STREAM(elapsedSendTime.count());
				if(!startedReceiving || elapsedSendTime.count()>3)
				{
					 ROS_INFO_STREAM("***Sending imei  message in sensor socket!!***");
					
					SensorConnection->Send(imeiBuffer, imeiLength);
					startedReceiving=true;
					lastSendTime = currentTime;
				}
				

				
				(SensorConnection->Send(buffer, bufferSize) == 0)? bufferSent = true:bufferSent = false;
								
				//this_thread::sleep_for(std::chrono::milliseconds(200));
				//ros::Duration(0.2).sleep();

			}
			else
			{
				ROS_INFO_STREAM("Sensor socket imei undefined.. skipping send");
			}
		}
		else
		{
			currentSocketStatus=EstadoSocket::SOCKET_STOPPING;
		} 
	}
	else if(currentSocketStatus==EstadoSocket::SOCKET_STOPPING)
	{
		// ROS_INFO_STREAM("Sensor socket stopping");
		int nRet = SensorConnection->DeInitialize();
		(nRet == 0)? clientConnection = false:clientConnection = true;
		currentSocketStatus=EstadoSocket::SOCKET_STOPPED;
		//this_thread::sleep_for(std::chrono::milliseconds(1000));
		ros::Duration(1.0).sleep();
	}
	lastSendTime = high_resolution_clock::now();
}

