#include <gpiod.h>
#include <string>
#include <pwd.h>
#include <vector>

#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include <std_msgs/Bool.h>

#include "libraries/json.hpp"
#include "PWM.h"

#define SERVO_EN_PIN 4   /* from gpio expander */
#define PWM_CHANNEL_DRIVER 0   /* from pwmchip0 (pin 168 tegra) */

static const int REAL_MIN_POSITION = -180;
static const int REAL_MAX_POSITION = 180;
static const int MIN_DUTY_CYCLE_USECS_PWM = 820;
static const int MAX_DUTY_CYCLE_USECS_PWM = 2130;
static const int BIAS_DUTY_CYCLE_USECS_PWM = 1480;
static const int PWM_FREQUENCY = 300;

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
		~Servo();
		bool Servo_Initialize();

	private:
		ros::NodeHandle nh;
		ros::Subscriber servo_sub, teleop_sub;
		string node_name;
		int prevGoal{0};
		bool calibration_enabled{false};
		json jsonCalib;
		vector<double> angles_table;
		vector<int> counts_table;
		PWM *pwm_driver = NULL;
		int SetServoPosition(int position);
		void PositionCallback(const std_msgs::Int32 &position_msg);
		void TeleopCallback(const std_msgs::Bool &teleop_msg);
		int GetPulsesOnCalibration(int goal);
		bool OpenCalibFile(const char *file);
		const char *chipname = "gpiochip2"; //gpios del expander
		struct gpiod_chip *chip;
		struct gpiod_line *lineServoEn;
};

Servo::Servo(ros::NodeHandle nh_priv)
{
	node_name = ros::this_node::getName();
	int log_level;
	nh_priv.param("log_level", log_level,0);
	ros::console::levels::Level console_level;
	console_level = (ros::console::levels::Level)log_level;
	ROS_INFO_STREAM(node_name << " --- Level: " << console_level);
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level) ) {
			ros::console::notifyLoggerLevelsChanged();
	}

	servo_sub = nh.subscribe("control_topic/servo_position", 5,  &Servo::PositionCallback, this);
	teleop_sub = nh.subscribe("teleop_running", 5,  &Servo::TeleopCallback, this);

	if ((homedir = getenv("HOME")) == NULL) {
		homedir = getpwuid(getuid())->pw_dir;
	}

	// Open GPIO chip
	chip = gpiod_chip_open_by_name(chipname);
	if(chip == NULL)
		ROS_ERROR_STREAM(node_name << " - GPIO error: chip is null");
	// Open GPIO lines
	lineServoEn = gpiod_chip_get_line(chip, SERVO_EN_PIN);
	if(lineServoEn == NULL)
		ROS_ERROR_STREAM(node_name << " - GPIO error: line are null");
	gpiod_line_request_output(lineServoEn, node_name.c_str(), 0);

	pwm_driver = new PWM(PWM_CHANNEL_DRIVER, PWM_FREQUENCY);
	pwm_driver->Start();
}

Servo::~Servo()
{
	gpiod_line_set_value(lineServoEn, 0);
	gpiod_line_release(lineServoEn);
	gpiod_chip_close(chip);

	pwm_driver->Stop();
	pwm_driver->~PWM();

	ROS_DEBUG_STREAM("destructor Servo");
}

void Servo::TeleopCallback(const std_msgs::Bool &teleop_msg)
{
	ROS_DEBUG_STREAM("Teleop running: " << (int) teleop_msg.data);
	if (teleop_msg.data)
		gpiod_line_set_value(lineServoEn, 1);
	else
		gpiod_line_set_value(lineServoEn, 0);
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

bool Servo::Servo_Initialize()
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
			
			if (angles_table.empty() || counts_table.empty())
				calibration_enabled = false;
		}
	}
	catch (const exception &ex)
	{
		ROS_ERROR_STREAM(node_name << " --- Error parsing control json");
		ROS_ERROR_STREAM(ex.what());
		calibration_enabled = false;
	}

	prevGoal = SetServoPosition(0);
	return true;
}

int Servo::SetServoPosition(int goal)
{
	ROS_INFO_STREAM("Goal received: " << goal);
	if (goal < REAL_MIN_POSITION) goal = REAL_MIN_POSITION;
	else if(goal > REAL_MAX_POSITION) goal = REAL_MAX_POSITION;

	int t_on_us = 0;

	if (calibration_enabled)
	{ //Value interpolated from lookup table
		t_on_us = GetPulsesOnCalibration(goal);
		ROS_DEBUG_STREAM(node_name << " --- Calib On");
	}
	else
	{ // Values interpolated from theoric
		t_on_us = BIAS_DUTY_CYCLE_USECS_PWM + (goal * (MAX_DUTY_CYCLE_USECS_PWM - MIN_DUTY_CYCLE_USECS_PWM) / (REAL_MAX_POSITION - REAL_MIN_POSITION));
		ROS_DEBUG_STREAM(node_name << " --- Calib Off");
	}

	pwm_driver->SetDutyCycle(t_on_us);
  return goal;
}

int Servo::GetPulsesOnCalibration(int goal)
{
	// Interpolacion
	int index = 0;
	for (double angle : angles_table)
	{
		if (goal <= angle)
		{
			break;
		}
		index++;
	}
	// Valores de Lookup table
	double angle1 = angles_table[index - 1];
	double angle2 = angles_table[index];
	int cnt1 = counts_table[index - 1];
	int cnt2 = counts_table[index];

	int pulses_on = (int)(interpolation(goal, angle1, angle2, cnt1, cnt2));

	return pulses_on;
	return 0;
}

void Servo::PositionCallback(const std_msgs::Int32 &position_msg)
{
    if (prevGoal != position_msg.data)
		prevGoal = SetServoPosition(position_msg.data);
}

