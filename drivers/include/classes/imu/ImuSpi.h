#include <cstring>
#include <iostream>
#include <fstream>

#include "libraries/DataUtil.hpp"
#include "classes/SPI/SPIDriver.h"
#include "classes/JSON/json_functions.h"

#include <ros/ros.h>
#include <drivers/rpy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <drivers/control_light_msg.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//static const float YAW_OFFSET = -180;

using namespace std;

class ImuSpi: virtual public SPIDriver
{
    public:
	ImuSpi(ros::NodeHandle nh_priv);
	virtual ~ImuSpi();
	int Initialize();
	int GetSensors();

    private:
        int sampleNo, samples;
        string vehicle{"wagonIndia"};
        bool mag_offset_file{false};
        float mag_x_offset, mag_y_offset, mag_z_offset;

        ros::Publisher pubImu, pubMag, pubOrntDeg;
        ros::Timer calib_timer;

        sensor_msgs::Imu imu_msg;
        sensor_msgs::MagneticField mag_msg;
        drivers::rpy rollitchaw;

        ofstream imuTest;
        json mag_calib;

        void saveData();
        void CalibMag();

};

ImuSpi::~ImuSpi(){
    if(samples)
        imuTest.close();
}

ImuSpi::ImuSpi(ros::NodeHandle nh_priv):SPIDriver(nh_priv)
{
    nh_priv.param("samples",samples,0);
    nh_priv.param("vehicle", vehicle,vehicle);
    pubImu = nh.advertise<sensor_msgs::Imu>("/sensor_topic/imu", 1);
    pubMag = nh.advertise<sensor_msgs::MagneticField>("/sensor_topic/mag", 1);
    pubOrntDeg = nh.advertise<drivers::rpy>("/sensor_topic/ortn_deg", 1);
    
    if (vehicle == "wagonIndia")
		commandsDir = "/config/wagonIndiaImu.json";
	else
		ROS_ERROR("Invalid vehicle type");
    
    if(samples)
    {
        time_t t = time(0);
        struct tm * now = localtime(&t);
        char buffer[80];
        strftime(buffer,80,"%Y-%m-%d-%H-%M", now);
        string path = "/home/jetson/Desktop/samples/";
        path += buffer;
        path += "-imumag.csv";
        imuTest.open(path);
        imuTest << "sampleNo,ornt_x,ornt_y,ornt_z,ornt_w,acc_raw_x,acc_raw_y,acc_raw_z,ang_vel_x,ang_vel_y,ang_vel_z,mag_x,mag_y,mag_z\n";
        sampleNo = 0;
    }

    mag_calib = OpenJSONFile((string{homedir} + "/braintemp/mag_calib.json").c_str());
    mag_offset_file = !mag_calib.empty();
}

int ImuSpi::Initialize()
{
    int nRet;
    //Initialization for generic driver
    nRet = SPIDriver::Initialize();

    //Initialization for specific driver
    if (mag_offset_file) {
        try {
            mag_x_offset = mag_calib["x_offset"].get<float>();
            mag_y_offset = mag_calib["y_offset"].get<float>();
            mag_z_offset = mag_calib["z_offset"].get<float>();
            ROS_DEBUG_STREAM(node_name << " --- xyz_offsets in json: " << mag_x_offset << ", " << mag_y_offset << ", " << mag_z_offset);

            calib_timer = nh.createTimer(ros::Duration(2), std::bind(&ImuSpi::CalibMag, this));
        } catch (const exception &ex) {
            ROS_ERROR_STREAM(node_name << " --- Error reading calib offsets: " << ex.what());
        }
    }

    return nRet;
}

int ImuSpi::GetSensors()
{
    float roll, pitch, yaw, roll_raw, pitch_raw, yaw_raw;

    roll_raw = GetCommand<float>("ORIENT_ROLL");
    pitch_raw = GetCommand<float>("ORIENT_PITCH");
    yaw_raw = GetCommand<float>("ORIENT_YAW");
    rollitchaw.roll = roll_raw;
    rollitchaw.pitch = pitch_raw;
    rollitchaw.yaw = yaw_raw;
    pubOrntDeg.publish(rollitchaw);
    
    roll = roll_raw* M_PI /180;
    pitch = pitch_raw* M_PI /180;
    yaw = yaw_raw * M_PI /180;
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( roll, pitch, yaw );  // Create this quaternion from roll/pitch/yaw (in radians)
    myQuaternion.normalize();
    imu_msg.orientation.x = myQuaternion[0];
    imu_msg.orientation.y = myQuaternion[1];
    imu_msg.orientation.z = myQuaternion[2];
    imu_msg.orientation.w = myQuaternion[3];
    
    imu_msg.linear_acceleration.x = GetCommand<float>("LIN_ACC_RAW_X");  //Raw due normal were provided by BNO (deactived)
    imu_msg.linear_acceleration.y = GetCommand<float>("LIN_ACC_RAW_Y");
    imu_msg.linear_acceleration.z = GetCommand<float>("LIN_ACC_RAW_Z");

    imu_msg.angular_velocity.x = GetCommand<float>("ANG_VEL_X");
    imu_msg.angular_velocity.y = GetCommand<float>("ANG_VEL_Y");
    imu_msg.angular_velocity.z = GetCommand<float>("ANG_VEL_Z");
    
    //ROS_DEBUG_STREAM("LIN ACC X: " << xLinAcc << " | LIN ACC Y: " << yLinAcc << " | ANG VEL Z: " << zAngVel);
    pubImu.publish(imu_msg);

    mag_msg.magnetic_field.x = GetCommand<float>("MAGNET_X");
    mag_msg.magnetic_field.y = GetCommand<float>("MAGNET_Y");
    mag_msg.magnetic_field.z = GetCommand<float>("MAGNET_Z");

    pubMag.publish(mag_msg);

    if(samples)
    {
        saveData();
    }

    return 0;
}

void ImuSpi::CalibMag()
{
    //check if float representation is the same/ compare GET to zero instead
    //float m_x, m_y, m_z;
    //m_x = GetCommand<float>("MAG_X_OFFSET", false);
    //m_y = GetCommand<float>("MAG_Y_OFFSET", false);
    //m_z = GetCommand<float>("MAG_Z_OFFSET", false);
    //ROS_WARN_STREAM(node_name << " --- xyz_offsets in FW: " << m_x << ", " << m_y << ", " << m_z);
    //ROS_WARN_STREAM(node_name << " --- xyz_offsets in json: " << mag_x_offset << ", " << mag_y_offset << ", " << mag_z_offset);

    if (GetCommand<float>("MAG_X_OFFSET") == 0 || GetCommand<float>("MAG_Y_OFFSET") == 0 || GetCommand<float>("MAG_Z_OFFSET") == 0) {
        SetCommand("MAG_X_OFFSET", mag_x_offset, false);
        SetCommand("MAG_Y_OFFSET", mag_y_offset, false);
        SetCommand("MAG_Z_OFFSET", mag_z_offset, false);
    }
}

void ImuSpi::saveData()
{
    string row = to_string(sampleNo) + "," 
            + to_string(imu_msg.orientation.x) + "," + to_string(imu_msg.orientation.y) + "," + to_string(imu_msg.orientation.z) + "," + to_string(imu_msg.orientation.w) + "," 
            + to_string(imu_msg.linear_acceleration.x) + "," + to_string(imu_msg.linear_acceleration.y) + "," + to_string(imu_msg.linear_acceleration.z) + "," 
            + to_string(imu_msg.angular_velocity.x) + "," + to_string(imu_msg.angular_velocity.y) + "," + to_string(imu_msg.angular_velocity.z) + "," 
            + to_string(mag_msg.magnetic_field.x) + "," + to_string(mag_msg.magnetic_field.y) + "," + to_string(mag_msg.magnetic_field.z) + "\n";
    imuTest << row;
    sampleNo += 1;
}
