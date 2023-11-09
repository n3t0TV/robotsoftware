#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <time.h> 

#include "libraries/json.hpp"
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include "std_msgs/Int32.h"

#include <ros/console.h>
#include <ros/package.h>

#include <modules/joystick_msg.h>
#include <modules/sensor_msg.h>

#include <modules/sensors_service.h>
#include <modules/imei_service.h>

#include <drivers/gps_msg.h>
#include <drivers/sensor_light_msg.h>
#include <drivers/sensor_motion_msg.h>
#include <drivers/sensor_picmanager_msg.h>
#include <drivers/control_light_msg.h>
#include <drivers/control_motion_msg.h>
#include <drivers/control_expcamera_msg.h>
#include <drivers/orientation_msg.h>
#include <std_msgs/Bool.h>
#include <drivers/rpy.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>


#include "classes/cam_exposure_publisher.h"


#define LOW_BATTERY 20

using namespace std;

class VehicleCore
{
    public:
	VehicleCore(ros::NodeHandle);
	~VehicleCore();
	
    int Initialize(string _version, string _vehicleType);
	void SetControl(const modules::joystick_msg & controlMsg);
	void PublishSensors();

    private:
    const std::string kSpeakerVolTopicName = "speaker_volume";
    const std::uint32_t kRosSubQueueSize = 10;
	int lastResetValue;
	float conversion_constant{1.0};
	bool failSafeMode{false};
	string node_name, brain_version, vehicleType, imei, package_path;

	drivers::gps_msg gpsData;
	modules::sensor_msg sensorMessage;	    
	drivers::sensor_light_msg sensorLight;
	drivers::control_light_msg controlLight;
	std_msgs::Bool controlBrake;
	ros::NodeHandle nh;
	ros::ServiceClient clientImei, reset_pic_srv_clnt;
    ros::ServiceServer sensors_service_;
	ros::Subscriber subSensorLight, subSensorMotion, subControl, 
		subSensorImu, subGps, subPicManager;
	ros::Subscriber speaker_vol_ros_sub_;
	ros::Publisher pubSensors, pubDebug, pubControlLight, 
	    pubRobotVelocity, pubControlMotion, pubTurningCamera, 
	    pubControlBrake, pubControlOrientation;

    std::unique_ptr<CamExposurePublisher> cam_exp_pub_;



	void LightSensorCallback(const drivers::sensor_light_msg & msg);
	void MotionSensorCallback(const drivers::sensor_motion_msg & msg);
        void GpsCallback(const drivers::gps_msg & msg);
        void ControlCallback(const modules::joystick_msg& ctrlMsg);
        void PublishRobotVelocity(const modules::joystick_msg& ctrlMsg);
        void SensorImuCallback(const drivers::rpy& imuMsg  );
	void PicManagerCallback(const drivers::sensor_picmanager_msg & msg);
    bool SensorServiceCallback(modules::sensors_service::Request &req, modules::sensors_service::Response &resp);
    void SpeakerVolCallback(const std_msgs::Int32 volume);
};

VehicleCore::VehicleCore(ros::NodeHandle nh_priv)
{
    node_name = ros::this_node::getName	();
    int log_level;
    nh_priv.param("log_level", log_level,0);
    ros::console::levels::Level console_level;
    console_level = (ros::console::levels::Level)log_level;
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    pubSensors = nh.advertise <modules::sensor_msg>("sensor_topic", 1);
    pubRobotVelocity = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    // pubDebug = nh.advertise<modules::debug_msg>("debug_topic",1); //revisar
    pubControlLight = nh.advertise<drivers::control_light_msg>("control_topic/light", 10);
    pubControlMotion = nh.advertise<drivers::control_motion_msg>("control_topic/motion", 10);   
    pubControlBrake = nh.advertise<std_msgs::Bool>("control_topic/brake", 10);
    pubTurningCamera = nh.advertise<std_msgs::Int32>("control_topic/servo_position", 10);
    pubControlOrientation = nh.advertise<drivers::orientation_msg>("control_topic/orientation",10);
    subGps = nh.subscribe("sensor_topic/gps", 10, &VehicleCore::GpsCallback, this);
    subControl = nh.subscribe("control_topic",10,&VehicleCore::ControlCallback, this);
    subSensorImu = nh.subscribe("sensor_topic/ortn_deg", 10, &VehicleCore::SensorImuCallback, this);
    subSensorLight = nh.subscribe("sensor_topic/light", 10, &VehicleCore::LightSensorCallback, this);
    subSensorMotion = nh.subscribe("sensor_topic/motion", 10, &VehicleCore::MotionSensorCallback, this);
    subPicManager = nh.subscribe("sensor_topic/picmanager", 10, &VehicleCore::PicManagerCallback, this);
    speaker_vol_ros_sub_ = nh.subscribe(kSpeakerVolTopicName, kRosSubQueueSize,
                                        &VehicleCore::SpeakerVolCallback, this);
    
    sensors_service_ = nh.advertiseService("sensors_service", &VehicleCore::SensorServiceCallback, this);

    package_path = ros::package::getPath("modules");
    cam_exp_pub_ = std::make_unique<CamExposurePublisher>(nh);
    lastResetValue=0;

    reset_pic_srv_clnt = nh.serviceClient<std_srvs::Empty>("reset_pic_service");
    clientImei = nh.serviceClient<modules::imei_service>("imei_service");
	modules::imei_service imei_srv;
	if (!clientImei.call(imei_srv))
	{
		ROS_ERROR_STREAM(node_name << " --- IMEI unknown");
		imei = "";
	}		
	else
	{
		imei = imei_srv.response.imei;
	}
}

VehicleCore::~VehicleCore()
{
	ROS_DEBUG_STREAM("destructor Wagon");
}

int VehicleCore::Initialize(string _version, string _vehicleType)
{
    brain_version = _version;
    vehicleType = _vehicleType;

    return 0;
}

void VehicleCore::PublishSensors()
{
	//Aqui va el sensado	
	pubSensors.publish(sensorMessage);
}

void VehicleCore::SetControl(const modules::joystick_msg & controlMsg)
{
	sensorMessage.UDPTimeStamp = controlMsg.UDPTimeStamp;
	sensorMessage.QRTimeStamp = controlMsg.QRTimeStamp;
	sensorMessage.LastControlMsg = controlMsg.LastControlMsg;
	sensorMessage.AutoPilot = controlMsg.AutoPilot;
	sensorMessage.Inferring = controlMsg.Inferring;

	//Aqui va el control
    drivers::control_light_msg controlLight;
	controlLight.PoleState = 1; // Turn on solid

	if (sensorMessage.batteryLevel < LOW_BATTERY)
		controlLight.BatteryState = 2;
	else
		controlLight.BatteryState = 0;

	controlLight.IlluminationFPWMDuty = controlMsg.Illumination;
	controlLight.IlluminationRPWMDuty = controlMsg.Illumination;
	controlLight.HeadPWMDuty = controlMsg.Illumination; //Falta var de ui

	if (controlMsg.TurningLights == 3)
		controlLight.BlinkerState = 3;
	else if (controlMsg.AngularRate > 10)
		controlLight.BlinkerState = 2;
	else if (controlMsg.AngularRate < -10)
		controlLight.BlinkerState = 1;
	else
		controlLight.BlinkerState = 0;

	if (controlMsg.Brake == 1 && controlMsg.TurningLights == 4) {
		controlLight.BlinkerState = 4; //cdmx2
		controlLight.StopState = 1;
	} else {
		controlLight.StopState = 0;
	}

	if (failSafeMode)
		controlLight.BlinkerState = 3;

	if((controlMsg.RightWheel<0 && controlMsg.LeftWheel<0)|| controlMsg.Speed < 0)
		controlLight.ReverseState = 1;
	else
		controlLight.ReverseState = 0;

	if(controlMsg.Illumination>0)
	{
		controlLight.HeadState = 1;
		if (controlMsg.Speed >= 0)
			controlLight.TailState = 1;
		else
			controlLight.TailState = 0;
	} else
	{
		controlLight.HeadState = 0;
		controlLight.TailState = 0;
	}

    drivers::control_motion_msg controlMotion;
    drivers::orientation_msg controlOrientation;
    controlBrake.data = controlMsg.Brake;

	controlMotion.DrivingMode = controlMsg.DrivingMode;
	controlMotion.DriveSpeed = controlMsg.Speed;
	if (failSafeMode && abs(controlMsg.AngularRate) < 15)
		controlMotion.DriveAngularRate = controlMsg.AngularRate * 2;
	else
		controlMotion.DriveAngularRate = controlMsg.AngularRate;
	pubControlMotion.publish(controlMotion);

	if (lastResetValue == 0 && controlMsg.Reset== 1) {
		std_srvs::Empty empty;
		reset_pic_srv_clnt.call(empty);
	}
	lastResetValue = controlMsg.Reset;
    
    cam_exp_pub_->Publish(controlMsg.cameraExp, controlMsg.videoMode);

    //Turning Camera - Servo
    std_msgs::Int32 turning_cam_msg;
    turning_cam_msg.data = controlMsg.MovCamera;
    
    pubTurningCamera.publish(turning_cam_msg); 
    pubControlLight.publish(controlLight);
    // PublishRobotVelocity(controlMsg); deactivated
    pubControlBrake.publish(controlBrake);
}

void VehicleCore::LightSensorCallback(const drivers::sensor_light_msg & msg)
{
	sensorMessage.turningLights = msg.BlinkerState;
	sensorMessage.illumination_f = msg.IlluminationFPWMDuty;
	sensorMessage.illumination_r = msg.IlluminationRPWMDuty;
	sensorMessage.lights = 0; //cambio de variables
}

void VehicleCore::MotionSensorCallback(const drivers::sensor_motion_msg & msg)
{
	sensorMessage.desiredSpeed = msg.DriveSpeed;
	sensorMessage.currentSpeed = msg.DriveSpeedCurr;
	sensorMessage.desiredSpeedLeft = msg.DriveSpeedLeft;
	sensorMessage.currentSpeedLeft = msg.DriveSpeedLeftCurr;
	sensorMessage.desiredSpeedRight = msg.DriveSpeedRight;
	sensorMessage.currentSpeedRight = msg.DriveSpeedRightCurr;
	sensorMessage.desiredAngle = msg.DriveAngle;
	sensorMessage.currentAngle = msg.DriveAngleCurr;

	sensorMessage.angular_rate_sp = msg.DriveAngularRate;
	sensorMessage.angular_rate_curr = msg.DriveAngularRateCurr;
}

void VehicleCore::PicManagerCallback(const drivers::sensor_picmanager_msg & msg)
{
	sensorMessage.version = msg.Version;
	sensorMessage.batteryLevel = msg.BatteryLevel;
	sensorMessage.batteryVoltage = msg.BatteryVoltage;
	sensorMessage.charging = msg.Charging;
	failSafeMode = msg.WagonFailSafeMode;
}

bool VehicleCore::SensorServiceCallback(modules::sensors_service::Request &req, modules::sensors_service::Response &resp)
{
	(void) req;
	resp.battery = sensorMessage.batteryLevel;
	resp.currentAngle  = sensorMessage.currentAngle;
	resp.currentSpeed = sensorMessage.currentSpeed;
	resp.desiredAngle = sensorMessage.desiredAngle;
	resp.desiredSpeed = sensorMessage.desiredSpeed;
	resp.yaw = sensorMessage.yaw;
	resp.turningLights = sensorMessage.turningLights;
	resp.illumination = sensorMessage.illumination;
	resp.picOK = sensorMessage.picOK;
    return true;
}

void VehicleCore::GpsCallback(const drivers::gps_msg & msg)
{
	// gpsData = msg;
    
	sensorMessage.Imei=imei;
	sensorMessage.Latitude=msg.Latitude;
	sensorMessage.Longitude=msg.Longitude;
	sensorMessage.Altitude=msg.Altitude;
	sensorMessage.Sim=msg.Sim;
	sensorMessage.rssi=msg.rssi;
	sensorMessage.LatHemisphere = msg.LatHemisphere;
	sensorMessage.LonHemisphere = msg.LonHemisphere;
	sensorMessage.Accuracy = msg.Accuracy;
	sensorMessage.UTCTime = msg.UTCTime;
	sensorMessage.UTCDate = msg.UTCDate;
	sensorMessage.Satellites = msg.Satellites;
    sensorMessage.validGPS=msg.validGPS;
	sensorMessage.provider = msg.provider;
	sensorMessage.accessTechnology = msg.accessTechnology;
	sensorMessage.band = msg.band;
	sensorMessage.ModuleConnected = msg.ModuleConnected;
}

void VehicleCore::ControlCallback(const modules::joystick_msg& ctrlMsg)
{
    SetControl(ctrlMsg);
    ROS_DEBUG_STREAM("Mensaje de control: " << ctrlMsg);
}

void VehicleCore::PublishRobotVelocity(const modules::joystick_msg& ctrlMsg)
{
    geometry_msgs::Twist tempRobotVelocity;
    tempRobotVelocity.linear.x = 1.0 * ctrlMsg.Speed * conversion_constant;
    tempRobotVelocity.angular.z = ctrlMsg.AngularRate;
    pubRobotVelocity.publish(tempRobotVelocity);
}

void VehicleCore::SensorImuCallback(const drivers::rpy& imuMsg)
{
	// double roll, pitch, yaw;
	// tf::Quaternion quat(imuMsg.orientation.x,imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w);
	// tf::Matrix3x3 m(quat);
	// tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	// sensorMessage.yaw = yaw * 180 / M_PI;
	// sensorMessage.pitch = pitch * 180 / M_PI;
	// sensorMessage.roll = roll * 180 / M_PI;
	sensorMessage.yaw = imuMsg.yaw;
	sensorMessage.pitch = imuMsg.pitch;
	sensorMessage.roll = imuMsg.roll;
}


void VehicleCore::SpeakerVolCallback(const std_msgs::Int32 volume)
{
	sensorMessage.speaker_volume = volume.data;
}
