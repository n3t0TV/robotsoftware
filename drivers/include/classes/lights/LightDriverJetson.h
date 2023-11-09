#include <iostream>
#include <string>
#include <gpiod.h>
#include <ros/console.h>
#include <drivers/sensor_light_msg.h>
#include <drivers/control_light_msg.h>
#include <drivers/pwm_service.h>
#include "classes/SPI/SPIDriver.h"

#define FRONT_CAM_LIGHT_CHANNEL 2
#define SPIN_CAM_LIGHT_CHANNEL 1
#define POLE_LIGHT_CTRL_PIN 5   /* from gpio expander */
#define BATTERY_LIGHT_PERIOD_MS 2000


class LightDriver:virtual public SPIDriver
{
	public:
		LightDriver(ros::NodeHandle nh_priv);
		virtual ~LightDriver();
		int GetSensors();
		void SetControl(const drivers::control_light_msg& msg);

	private:
		drivers::sensor_light_msg sensor_msg;
		ros::Publisher pubLightSensors;
		ros::ServiceClient pwm_service;
		const char *chipname = "gpiochip2"; //gpios del expander
		struct gpiod_chip *chip;
		struct gpiod_line *linePoleCtrl;
		int pole_count, poleCtrlValue{1};
};

LightDriver::LightDriver(ros::NodeHandle nh_priv):SPIDriver(nh_priv)
{
	commandsDir = "/config/wagonIndiaLight.json";
	pubLightSensors = nh.advertise <drivers::sensor_light_msg>("sensor_topic/light", 10);
    pwm_service = nh.serviceClient<drivers::pwm_service>("pwm_duty_service");
	pole_count = 0;

	// Open GPIO chip
	chip = gpiod_chip_open_by_name(chipname);
	if(chip == NULL)
		ROS_ERROR_STREAM(node_name << " - GPIO error: chip is null");
	// Open GPIO lines
	linePoleCtrl = gpiod_chip_get_line(chip, POLE_LIGHT_CTRL_PIN);
	if(linePoleCtrl == NULL)
		ROS_ERROR_STREAM(node_name << " - GPIO error: line are null");
	gpiod_line_request_output(linePoleCtrl, "light_node", poleCtrlValue);
}

LightDriver::~LightDriver()
{
	gpiod_line_release(linePoleCtrl);
	gpiod_chip_close(chip);

	ROS_DEBUG_STREAM("destructor LightDriver");
}

int LightDriver::GetSensors()
{
	sensor_msg.BlinkerState = GetCommand<int>("BLINKER_STATE", false);
	sensor_msg.ReverseState = GetCommand<int>("LIGHT_REVERSE_STATE", false);
	sensor_msg.StopState = GetCommand<int>("LIGHT_STOP_STATE", false);
	sensor_msg.HeadState = GetCommand<int>("LIGHT_HEAD_STATE", false);
	sensor_msg.HeadPWMDuty = GetCommand<int>("HEAD_LIGHT_PWM_DUTY", false);  //Falta var de ui
	sensor_msg.TailState = GetCommand<int>("LIGHT_TAIL_STATE", false);

	pubLightSensors.publish(sensor_msg);

	return 0;
}

void LightDriver::SetControl(const drivers::control_light_msg& msg)
{
    /* SPI ctrl */
	SetCommand("BLINKER_STATE", msg.BlinkerState, false);
	SetCommand("LIGHT_REVERSE_STATE", msg.ReverseState, false);
	SetCommand("LIGHT_STOP_STATE", msg.StopState, false);
	SetCommand("LIGHT_HEAD_STATE", msg.HeadState, false);
	SetCommand("HEAD_LIGHT_PWM_DUTY", msg.HeadPWMDuty, false);//falta la var del ui
	SetCommand("LIGHT_TAIL_STATE", msg.TailState, false);
	SetCommand("LIGHT_BATTERY", msg.BatteryState, false);
	SetCommand("LIGHT_BATTERY_PERIOD_MS", BATTERY_LIGHT_PERIOD_MS, false);

    /* PWM ctrl */
    drivers::pwm_service pwm_msg;
    pwm_msg.request.channel = FRONT_CAM_LIGHT_CHANNEL;
    pwm_msg.request.pwmDuty = msg.IlluminationFPWMDuty;
    if (!pwm_service.call(pwm_msg))
	    ROS_ERROR_STREAM("light_node - PWM service error!");

    pwm_msg.request.channel = SPIN_CAM_LIGHT_CHANNEL;
    pwm_msg.request.pwmDuty = msg.IlluminationRPWMDuty;
    if (!pwm_service.call(pwm_msg))
	    ROS_ERROR_STREAM("light_node - PWM service error!");

    /* GPIO ctrl */
    if (msg.PoleState == 1) {
        poleCtrlValue = 0;     // turn on
	    gpiod_line_set_value(linePoleCtrl, poleCtrlValue);
    } else if (msg.PoleState == 2) {
        poleCtrlValue ^= 1;     // toggle
	    gpiod_line_set_value(linePoleCtrl, poleCtrlValue);
    } else {
        poleCtrlValue = 1;     // turn off
        gpiod_line_set_value(linePoleCtrl, poleCtrlValue);
    }
}
