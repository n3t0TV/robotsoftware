#include <ros/ros.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <fstream>
#include <pwd.h>
#include <vector>
#include "libraries/PCA9685.h"
#include "libraries/json.hpp"
#include "std_msgs/Int32.h"

// Constants for servo control
static const int SERVO_CHANNEL = 0;
static const double SERVO_FREQUENCY = 50.0;
static const int SERVO_ZERO_POSITION = 135;
static const int SERVO_RANGE = 270;
static const int GEAR_RATIO = 2;
static const int MIN_DUTY_CYCLE_USECS = 500;
static const int MAX_DUTY_CYCLE_USECS = 2500;
static const int SIGNAL_PERIOD_USECS = 20000;
static const int DRIVER_RESOLUTION = 4095;
static const int REAL_MIN_POSITION = -180;
static const int REAL_MAX_POSITION = 180;

double interpolation(double x, double x0, double x1, double y0, double y1){
    return (x-x0)*(y1-y0)/(x1-x0) + y0;
} 

const char *homedir;

using namespace std;
using json = nlohmann::json;

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
	    int prev_goal{0};
	    bool calibration_enabled{false};
	    json jsonCalib;
	    vector<double> angles_table;
	    vector<int> counts_table;
	    PCA9685 pwmDriver{};
	    int SetServoPosition(int position);
	    void PositionCallback(const std_msgs::Int32 &position_msg);
	    int min_servo_position, max_servo_position;
	    int GetPulsesOnCalibration(int goal);
	    bool OpenCalibFile(const char * file);

};

Servo::Servo(ros::NodeHandle nh_priv)
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
    servo_sub = nh.subscribe("control_topic/servo_position", 5,  &Servo::PositionCallback, this);
    if ((homedir = getenv("HOME")) == NULL) {
	homedir = getpwuid(getuid())->pw_dir;
    }
}

bool Servo::OpenCalibFile(const char * file)
{
    std::ifstream i(file);
    if (i.is_open())
    {
	if (jsonCalib.accept(i))
	{
	    std::ifstream i(file);
	    i >> jsonCalib;
	    return true;
	}
	else
	{
	    ROS_ERROR_STREAM(node_name << " --- Invalid JSON file");
	    return false;
	}
    }
    else
    {
	ROS_ERROR_STREAM(node_name << " --- This File cannot be opened");
	return false;
    }
}

bool Servo::Initialize()
{
    try
    {
	calibration_enabled = OpenCalibFile((std::string{homedir} + "/braintemp/servo_lookup_table.json").c_str());
	if (calibration_enabled)
	{
	    if (jsonCalib.contains("angles_table"))
		angles_table = jsonCalib["angles_table"].get<std::vector<double>>();
	    if (jsonCalib.contains("counts_table"))
		counts_table = jsonCalib["counts_table"].get<std::vector<int>>();
	}
    }
    catch(const exception &ex)
    {
	ROS_ERROR_STREAM(node_name << " --- Error parsing control json");
	ROS_ERROR_STREAM(ex.what());
	calibration_enabled = false;
    }

    min_pulses_on = MIN_DUTY_CYCLE_USECS * DRIVER_RESOLUTION / SIGNAL_PERIOD_USECS;
    max_pulses_on = MAX_DUTY_CYCLE_USECS * DRIVER_RESOLUTION / SIGNAL_PERIOD_USECS;
    min_servo_position = SERVO_ZERO_POSITION - SERVO_RANGE;
    max_servo_position = SERVO_RANGE - SERVO_ZERO_POSITION;

    pwmDriver.set_pwm_freq(SERVO_FREQUENCY); //50Hz
    prev_goal = SetServoPosition(0);
    return true;
}

int Servo::SetServoPosition(int goal)
{
    ROS_INFO_STREAM("Goal received: " << goal);
    if (goal < REAL_MIN_POSITION) goal = REAL_MIN_POSITION;
    else if(goal > REAL_MAX_POSITION) goal = REAL_MAX_POSITION;

    int pulses_on = 0;
    if (calibration_enabled)
    {//Value interpolated from lookup table
	pulses_on = GetPulsesOnCalibration(goal);
	ROS_DEBUG_STREAM(node_name << " --- Calib On");
    }
    else
    {// Values interpolated from theoric
	pulses_on = min_pulses_on + ((goal/GEAR_RATIO) - min_servo_position) * (max_pulses_on - min_pulses_on) / (max_servo_position - min_servo_position);
	ROS_DEBUG_STREAM(node_name << " --- Calib Off");
    }

    ROS_INFO_STREAM("Pulses ON: " << pulses_on);
    pwmDriver.set_pwm(SERVO_CHANNEL, 0, pulses_on);
    return goal;
}

int Servo::GetPulsesOnCalibration(int goal)
{
    // Interpolacion
    int index = 0;
    for(double angle : angles_table){
        if(goal <= angle){ 
            break;
        }
	index++;
    }
    // Valores de Lookup table
    double angle1 = angles_table[index-1];
    double angle2 = angles_table[index];
    int cnt1 = counts_table[index-1];
    int cnt2 = counts_table[index];

    int pulses_on = (int)(interpolation(goal,angle1,angle2,cnt1,cnt2));

    return pulses_on;
    return 0;
}

void Servo::PositionCallback(const std_msgs::Int32 &position_msg)
{
    if (prev_goal != position_msg.data)
	prev_goal = SetServoPosition(position_msg.data);
}
