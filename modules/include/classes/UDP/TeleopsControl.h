#include <ros/ros.h>
#include <ros/console.h>
#include "../Structs/EstadoTeleops.h"
#include "../UDP/ClientConnection.h"
#include "../Structs/Control.h"
#include <modules/imei_service.h>
#include "classes/Structs/EstadoTeleops.h"
#include <modules/state_msg.h>
#include <modules/socket_state_msg.h>
#include <modules/motion_ctrl_msg.h>
#include <modules/misc_ctrl_msg.h>
#include <modules/controlsocket_status_msg.h>
#include <modules/speaker_msg.h>
#include <chrono>
#include "libraries/json.hpp"
#include <fstream>
#include <stdlib.h>
#include <iostream>

using namespace std;
using json = nlohmann::json;

int previous_lag,jitter;

#define SO_RCV_MAX_BUFFER 2048		//avoid UDP queueing limit de recv buffer
#define BUFFER_CONTROL 1024
#define DIFFERENTIAL_PI_MODE 2

using namespace std::chrono;

class TeleopsControl
{
	public:
		//EstadoTeleops estadoTeleops;
		teleoperacion teleops;
		MusclesState musculos;
		bool initControl;
		bool deinitControl;
		bool IdleSetting;
		int teleopEmergencyStop;

		int currentSocketStatus;
		int desiredSocketStatus,numControlReceived;

		string imei;
		bool clientConnection, controlThread, bufferReceived,startedReceiving;

		TeleopsControl(ros::NodeHandle);
		~TeleopsControl();
		void Initialize();
		void stateMachine();
		double getControlMsgTime();
		bool exists(const json& j, const std::string& key);
	protected:
		int port;
		ClientConnection *ControlConnection;
		high_resolution_clock::time_point lastSendTime;
		high_resolution_clock::time_point currentTime;
		duration<double> elapsedSendTime;
		ros::Duration controlMsgTime ;
	private:
		ros::ServiceClient imeiClient;
		ros::Subscriber subStatus;
		ros::Subscriber subSocket;
		ros::Publisher pubControlMotion, pubControlMisc;
		ros::Publisher pubAudioCommand;
		modules::imei_service imei_srv_msg;
		modules::motion_ctrl_msg  motion_msg;
		modules::misc_ctrl_msg  misc_msg;
		modules::speaker_msg audio_msg;
		ros::NodeHandle nh;
		void StatusCallback(const modules::state_msg msg);
		void requestImei();

		double msgTimeStamp;
		int i = 0;
		char buffer[BUFFER_CONTROL];
		int imeiLength;
		string prevAudioCommand="0";
		char imeiBuffer[15];
		string node_name;
		ros::Time time;
		//ofstream control_log;
		long int temp_sec,temp_msec, time_diff, temp_timesent;

};

TeleopsControl::TeleopsControl(ros::NodeHandle nh_priv)
{
	node_name = ros::this_node::getName	();
    int log_level;
    nh_priv.param("log_level", log_level,0);
    ros::console::levels::Level console_level;
    console_level = (ros::console::levels::Level)log_level;
    ROS_INFO_STREAM(node_name << " --- Level: " << console_level);
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    imei="";
	imeiClient = nh.serviceClient<modules::imei_service>("imei_service");
	subStatus = nh.subscribe("status_topic",10,&TeleopsControl::StatusCallback, this);
	pubControlMotion = nh.advertise<modules::motion_ctrl_msg>("teleops_topic/udp",10);
	pubControlMisc = nh.advertise<modules::misc_ctrl_msg>("teleops_topic/misc",5);
	pubAudioCommand = nh.advertise<modules::speaker_msg>("speaker_topic",5);
	initControl = false;
	deinitControl = false;
	currentSocketStatus=EstadoSocket::SOCKET_STOPPED;
	desiredSocketStatus=EstadoSocket::SOCKET_STOPPED;
	clientConnection = false;
	bufferReceived = false;
	teleopEmergencyStop=0;
	numControlReceived=1;//avoid emergency stop before start receiving messages
}
TeleopsControl::~TeleopsControl()
{
	//control_log.close();
}
void TeleopsControl::Initialize(){
	int nRet=0;

	motion_msg.Speed = 0;
	motion_msg.AngularRate = 0;
	motion_msg.Brake = 0;
	misc_msg.Illumination = 0;
	misc_msg.TurningLights = 0;
	misc_msg.Reset = 0;
	misc_msg.DrivingMode = DIFFERENTIAL_PI_MODE;
	misc_msg.MovCamera = 0;
	misc_msg.AutoPilot = 0;

	teleops.estatus=EstadoScooter::PARKED;
	controlThread = true;
	//nLength = BUFFER_CONTROL;
	imeiLength=15;
	lastSendTime = high_resolution_clock::now();
	startedReceiving=false;
	requestImei();
/*	time = ros::Time::now();
	string str_time = to_string(time.sec);
	string file_name = "/home/pi/control_log_"+str_time+".json";
	control_log.open(file_name.c_str(),ios::out);
	control_log << "Init file"<<endl;*/
}

bool TeleopsControl::exists(const json& j, const std::string& key)
{
    if (j.contains(key))
		return true;
	else
		ROS_WARN_STREAM("Key: " << key << "doesn't exist");
    return 	false;
}

void TeleopsControl::stateMachine(){

	//~ BOOST_LOG_TRIVIAL(info) << "Thread: Control";
	if(currentSocketStatus==EstadoSocket::SOCKET_STOPPED)
	{
		//~ ROS_INFO_STREAM("Control socket stoped");
		startedReceiving=false;
		if(desiredSocketStatus==1)
		{
			currentSocketStatus=EstadoSocket::SOCKET_STARTING;
			ros::Duration(0.5).sleep();
			//~ this_thread::sleep_for(std::chrono::milliseconds(500));
		}
	}
	else if(currentSocketStatus==EstadoSocket::SOCKET_STARTING)
	{
		//~ ROS_INFO_STREAM("Initializing control socket");
		startedReceiving=false;
		ControlConnection = new ClientConnection( teleops.servidor, teleops.control, BUFFER_CONTROL, false, SO_RCV_MAX_BUFFER);
		int res = ControlConnection->Initialize();
		(res == 0)? clientConnection = true:clientConnection = false;
		if (res == 0)
		{
			initControl = true;
			ROS_INFO_STREAM("Control connection starts successfully");
			currentSocketStatus=EstadoSocket::SOCKET_STARTED;
			//ah.SendCommand(SET_BRAIN,1);
		}
		else
		{
			initControl = false;
			ROS_INFO_STREAM("Control connection failed");
			if(desiredSocketStatus==0)
			{
				currentSocketStatus=EstadoSocket::SOCKET_STOPPED;
			}
		}
		ros::Duration(0.5).sleep();
		//~ this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	else if(currentSocketStatus==EstadoSocket::SOCKET_STARTED)
	{
		//~ ROS_INFO_STREAM("Control socket started...");

		if(desiredSocketStatus==1)
		{
			//ROS_INFO("Control socket desired status 1...");
			currentTime = high_resolution_clock::now();
			elapsedSendTime = duration_cast<duration<double>>(currentTime - lastSendTime);
			//~ ROS_INFO_STREAM(elapsedSendTime.count());
			if(!startedReceiving || elapsedSendTime.count()>3)
			{
				if(imei.empty())
					requestImei();

				if(!imei.empty())
				{
					//~ ROS_INFO_STREAM("Sending control imei message: ");
					ControlConnection->Send(imeiBuffer, imeiLength);
					lastSendTime = currentTime;
				}
			}
			int nLength=BUFFER_CONTROL;//max length expected
			ControlConnection->Receive( buffer, nLength );

			if(nLength>0)
			{
				const auto now = chrono::system_clock::now();
				uint64_t unix_arrival = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()).count();
				//~ ROS_DEBUG_STREAM("Unix received: " << unix_arrival);

				bufferReceived = true;
				startedReceiving=true;
				int i;
				char bufferStr[BUFFER_CONTROL];

				//~ ROS_DEBUG_STREAM("buffer len");
				//~ ROS_DEBUG_STREAM(nLength);
				strncpy(bufferStr,buffer,nLength);
				//cout<<bufferStr;
				if (strcmp(bufferStr, "Die!")==0)
				{
					ROS_ERROR_STREAM("Control socket died");
					currentSocketStatus=EstadoSocket::SOCKET_STOPPING;
					return;
				}

				bufferStr[nLength]='\0';//end of line
				//~ ROS_DEBUG_STREAM("Control string");
				ROS_DEBUG_STREAM(bufferStr);

				try
				{
					json jsonControl = json::parse(bufferStr);

					if(jsonControl.contains("s"))
					{
						motion_msg.TimeStamp = jsonControl["t"].get<uint64_t>();
						motion_msg.Speed = jsonControl["s"].get<int>();
						motion_msg.AngularRate = jsonControl["a"].get<float>();
						motion_msg.Brake = jsonControl["b"].get<int>();

						pubControlMotion.publish(motion_msg);
					}
					else
					{
						misc_msg.Illumination = jsonControl["l"].get<int>();
						misc_msg.TurningLights = jsonControl["tl"].get<int>();
						misc_msg.Reset = jsonControl["r"].get<int>();
						misc_msg.MovCamera = jsonControl["c"].get<int>();
						misc_msg.cameraExp = jsonControl["x"].get<int>();
						misc_msg.AutoPilot = jsonControl["AP"].get<int>();


						if(jsonControl.contains("m"))
							misc_msg.DrivingMode = jsonControl["m"].get<int>();
						else
							misc_msg.DrivingMode = DIFFERENTIAL_PI_MODE;

						pubControlMisc.publish(misc_msg);

						string audioId=jsonControl["ac"].get<std::string>();
						if(audioId.compare("0")!=0 && prevAudioCommand.compare(audioId)!=0)	//not 0 or repeated command
						{
							audio_msg.mp3Id = audioId;
							pubAudioCommand.publish(audio_msg);
						}
						prevAudioCommand = audioId;
					}

					/*uint64_t unix_sent = jsonControl["timestamp"].get<int64_t>();
					uint64_t unix_lag = unix_arrival - unix_sent;
					jitter = unix_lag - previous_lag;
					ROS_DEBUG_STREAM("Lag: " << unix_lag << " | Jitter: " << jitter);
					previous_lag = unix_lag;*/


				}
				catch(const exception &ex)
				{
					ROS_ERROR_STREAM("Error parsing control json: "  << bufferStr);
					ROS_ERROR_STREAM(ex.what());
				}

			}
		}
		else
		{
			//~ ROS_INFO("Socket stopping.. desired control is 0");

			currentSocketStatus=EstadoSocket::SOCKET_STOPPING;
		}
	}
	else if(currentSocketStatus==EstadoSocket::SOCKET_STOPPING)
	{
		//~ ROS_INFO("Stopping control socket");
		startedReceiving=false;
		int nRet = ControlConnection->DeInitialize();
		//~ ROS_DEBUG_STREAM("Control connection ended: " << nRet);
		(nRet == 0)? clientConnection = false:clientConnection = true;
		if (nRet == 0)
		{
			ROS_INFO_STREAM("Control connection ends successfully");
			deinitControl=true;
		}
		else
		{
			ROS_ERROR_STREAM("Control connection end failed");
			deinitControl=false;
		}
		currentSocketStatus=EstadoSocket::SOCKET_STOPPED;
		//~ this_thread::sleep_for(std::chrono::milliseconds(1000));
		ros::Duration(0.5).sleep();
	}

}

double TeleopsControl::getControlMsgTime()
{
	return 0;// controlMsgTime.count();
}

void TeleopsControl::requestImei()
{
	//ros::service::waitForService("imei_service");
	if(imeiClient.call(imei_srv_msg))
	{
		imei=imei_srv_msg.response.imei;
		strncpy(imeiBuffer,imei.c_str(),sizeof(imeiBuffer));
		//sprintf(imeiBuffer,"%s",imei.c_str());
	}
	else
		imei="";
	ROS_INFO_STREAM(imei);
}

void TeleopsControl::StatusCallback(const modules::state_msg msg)
{
    /*ROS_DEBUG("Teleop control state message");
	ROS_INFO_STREAM(msg.status);
	ROS_INFO_STREAM(msg.servidor);
	ROS_INFO_STREAM(msg.control);
	ROS_INFO_STREAM(msg.video);
	ROS_INFO_STREAM(msg.sensores);*/
	teleops.estatus=msg.status;
	teleops.servidor=msg.servidor;
	teleops.control=msg.control;

	if(msg.status==EstadoScooter::TELEOP)
	{
		desiredSocketStatus=1;
	}
	else
	{
		desiredSocketStatus=0;
	}
}
