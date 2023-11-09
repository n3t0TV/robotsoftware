#include <ros/ros.h>
#include <wiringPi.h>
#include <softPwm.h>
#include "libraries/PCA9685.h"
#include "std_msgs/Int32.h"

/*
static const int SERVO_CHANNEL = 0;
static const int SERVO_OFFSET = 180;
static const int SERVO_MIN_POSITION = 0;
static const int SERVO_MAX_POSITION = 270;
static const int GEAR_RATIO = 2;
static const int MIN_DUTY_CYCLE_USECS = 500;
static const int MAX_DUTY_CYCLE_USECS = 2500;
static const int SIGNAL_PERIOD_USECS = 20000;
static const int DRIVER_RESOLUTION = 4095;

*/

static const int SERVO_CHANNEL = 0;
static const int SERVO_OFFSET = 180;
static const int SERVO_MIN_POSITION = 0;
static const int SERVO_MAX_POSITION = 180;
static const int GEAR_RATIO = 2;
static const int MIN_DUTY_CYCLE_USECS = 500;
static const int MAX_DUTY_CYCLE_USECS = 2500;
static const int SIGNAL_PERIOD_USECS = 20000;
static const int DRIVER_RESOLUTION = 4095;

static const int REAL_MIN_POSITION = 0;
static const int REAL_MAX_POSITION = 360;

class Servo
{
    public:
	Servo(ros::NodeHandle);
	~Servo(){};
	bool Initialize();
    private:
	    ros::NodeHandle nh;
	    ros::Subscriber servo_sub;
	    std::string node_name;
	    int counter{0};
	    int min_pulses_on;
	    int max_pulses_on;
	    PCA9685 pwmDriver{};
	    int SetServoPosition(int position);
	    void PositionCallback(const std_msgs::Int32 &position_msg);

};

Servo::Servo(ros::NodeHandle nh_priv)
{
    node_name = ros::this_node::getName	();
    int log_level;
    nh_priv.param("log_level", log_level,0);
    ros::console::levels::Level console_level;
    console_level = (ros::console::levels::Level)log_level;
    //~ ROS_INFO_STREAM(node_name << " --- Level: " << console_level);
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    servo_sub = nh.subscribe("servo_position", 5,  &Servo::PositionCallback, this);
}

bool Servo::Initialize()
{
    min_pulses_on = MIN_DUTY_CYCLE_USECS * DRIVER_RESOLUTION / SIGNAL_PERIOD_USECS;
    max_pulses_on = MAX_DUTY_CYCLE_USECS * DRIVER_RESOLUTION / SIGNAL_PERIOD_USECS;
    pwmDriver.set_pwm_freq(50); //50Hz
    SetServoPosition(0);
    return true;
}

int Servo::SetServoPosition(int position)
{
    int goal = position + SERVO_OFFSET;
    ROS_INFO_STREAM("Goal received: " << position);
    if (goal < REAL_MIN_POSITION) goal = REAL_MIN_POSITION;
    else if(goal > REAL_MAX_POSITION) goal = REAL_MAX_POSITION;    
    int pulses_on = min_pulses_on + (goal/GEAR_RATIO) * (max_pulses_on - min_pulses_on) / SERVO_MAX_POSITION;
    ROS_INFO_STREAM("Pulses ON: " << pulses_on);
    pwmDriver.set_pwm(SERVO_CHANNEL, 0, pulses_on);
    return goal - SERVO_OFFSET;
}

void Servo::PositionCallback(const std_msgs::Int32 &position_msg)
{
    SetServoPosition(position_msg.data);
}
