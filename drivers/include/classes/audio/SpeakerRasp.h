#include <ros/ros.h>
#include <ros/package.h>

#include <pwd.h>
#include <string.h>
#include <ros/console.h>
#include <wiringPi.h>

#include <std_srvs/SetBool.h>
#include <modules/speaker_msg.h>
#include <modules/joystick_msg.h>

#define AUDIO_ENABLE 4 //wiring pi naming

using namespace std;

class Speaker
{
    public:
	Speaker(ros::NodeHandle);
	~Speaker(){};
	void play(const modules::speaker_msg msg);
	void rearAlarm(const modules::joystick_msg & musculos);
	void activateAlarm();
	void activateRearAlarm();
	void brainOn();
	void brainOff();
	bool alarmEnable, rearEnable;

    private:
	ros::NodeHandle nh;
	ros::Subscriber subSpeaker,subControl;
	modules::speaker_msg speakerMsg;
	string mp3_folder;
};

Speaker::Speaker(ros::NodeHandle nh_priv)
{
    string drivers_path = ros::package::getPath("drivers");
    int log_level;
    nh_priv.param("log_level", log_level,0);
    ros::console::levels::Level console_level;
    console_level = (ros::console::levels::Level)log_level;
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    subSpeaker = nh.subscribe("speaker_topic",1,&Speaker::play, this);
    subControl = nh.subscribe("control_topic",1,&Speaker::rearAlarm, this);
    wiringPiSetup();
    pinMode(AUDIO_ENABLE,OUTPUT);
    digitalWrite(AUDIO_ENABLE,HIGH);
    mp3_folder = drivers_path + "/mp3/audio";

    alarmEnable = false;
    rearEnable = false;
}

void Speaker::play(const modules::speaker_msg msg)
{
    ROS_DEBUG_STREAM("--- Speaker message received");
    ROS_DEBUG_STREAM("--- Active: " << to_string(msg.active));
    ROS_DEBUG_STREAM("--- Audio id: " << msg.mp3Id);

    if(msg.active)
    {
        string command = "omxplayer --vol 0 " + mp3_folder + msg.mp3Id + ".mp3";
        ROS_DEBUG_STREAM("Playing mp3...");
        system(command.c_str());
        system("killall omxplayer.bin");
        ROS_DEBUG_STREAM("Stop");
    }
}

void Speaker::rearAlarm(const modules::joystick_msg & musculos)
{
    if(musculos.Speed < 0)
    {
        rearEnable = true;
    }
    else
    {
        rearEnable = false;
    }
}

void Speaker::brainOff()
{
    string command = "omxplayer --vol 0 " + mp3_folder + "106.mp3";
    ROS_DEBUG_STREAM("Brainmodules on");
    system(command.c_str());
}

void Speaker::brainOn()
{
    string command = "omxplayer --vol 0 " + mp3_folder + "107.mp3";
    ROS_DEBUG_STREAM("Brainmodules on");
    system(command.c_str());
}

void Speaker::activateRearAlarm()
{
    string command = "omxplayer --vol 0 " + mp3_folder + "108.mp3";
    ROS_DEBUG_STREAM("Playing rear alarm...");
    system(command.c_str());
}

void Speaker::activateAlarm()
{
    string command = "omxplayer --vol 0 " + mp3_folder + "109.mp3";
    ROS_DEBUG_STREAM("Playing alarm...");
    system(command.c_str());
}
