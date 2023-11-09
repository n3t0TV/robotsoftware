#include "classes/I2C/I2C_driver.h"
#include "libraries/GPIOExpConstants.h"
#include <drivers/gpio_expander_service.h>

class GPIOEXP : public I2C_Driver{
	public:
		GPIOEXP(ros::NodeHandle nh_priv, uint8_t address = 0x27);
		void Initialize();
	private:
		uint8_t portA{0}, portB{0};
		void check_ret(int ret, std::string msg = "");
        bool update_gpio(drivers::gpio_expander_service::Request& req, drivers::gpio_expander_service::Response& res);
        bool update_port(drivers::gpio_expander_service::Request& req, drivers::gpio_expander_service::Response& res);
		void update_portA(uint8_t value);
		void update_portB(uint8_t value);
        ros::NodeHandle nh;
        ros::ServiceServer gpio_port_service, gpio_exp_service;
		
};

GPIOEXP::GPIOEXP(ros::NodeHandle nh_priv, uint8_t address): I2C_Driver{nh_priv, address}
{
    gpio_port_service = nh.advertiseService("gpio_port_service", &GPIOEXP::update_port, this);
	gpio_exp_service = nh.advertiseService("gpio_service", &GPIOEXP::update_gpio, this);
}

void GPIOEXP::Initialize()
{
	// Configure IOCON
	WriteI2Cbyte( EXPANDER_REGISTER_IOCON, 0X42);

	// Configure IODIRA (all outputs)
	WriteI2Cbyte( EXPANDER_REGISTER_IODIRA, 0X00);
	// Configure GPPUA (no internal pull-ups)
	WriteI2Cbyte( EXPANDER_REGISTER_GPPUA, 0X00);

	// Configure IODIRB (all outputs)
	WriteI2Cbyte( EXPANDER_REGISTER_IODIRB, 0X00);
	// Configure GPPUB (no internal pull-ups)
	WriteI2Cbyte( EXPANDER_REGISTER_GPPUB, 0X00);

	auto con = ReadI2Cbyte( EXPANDER_REGISTER_IOCON);
	check_ret(con, "Unable to talk to GPIO Expander, check connections");
}

bool GPIOEXP::update_gpio(drivers::gpio_expander_service::Request& req, drivers::gpio_expander_service::Response& res)
{
	if (req.channel >= CM_CAM0_PWEN && req.channel <= CM_CAM3_RES)
	{
		req.channel -= 8;
		if (req.value)
			portB |= (1 << req.channel);
		else
			portB &= ~(1 << req.channel);
		
		update_portB(portB);
		ROS_DEBUG_STREAM("PortB after update: " << (int)portB);
	}
	else if (req.channel >= GSM1_SLEEP_CTRL && req.channel <= OUT_SPARE)
	{
		if (req.value)
			portA |= (1 << req.channel);
		else
			portA &= ~(1 << req.channel);

		update_portA(portA);
		ROS_DEBUG_STREAM("PortA after update: " << (int)portA);
	}
	else
	{
		ROS_ERROR_STREAM("Channel is not valid!");
		res.success = false;
	}
	res.success = true;
	return true;
}

bool GPIOEXP::update_port(drivers::gpio_expander_service::Request& req, drivers::gpio_expander_service::Response& res)
{
	if (req.channel) 
	{
		ROS_DEBUG_STREAM("Update portB with data: " << (int)req.value);
		portB = req.value;
		update_portB(portB);
	}
	else
	{
		ROS_DEBUG_STREAM("Update portA with data: " << (int)req.value);
		portA = req.value;
		update_portA(portA);
	}
	res.success = true;
	return true;
}

void GPIOEXP::update_portA(uint8_t value) 
{
	WriteI2Cbyte( EXPANDER_REGISTER_OLATA, value);
}

void GPIOEXP::update_portB(uint8_t value) 
{
	WriteI2Cbyte( EXPANDER_REGISTER_OLATB, value);
}

void GPIOEXP::check_ret(int ret, std::string msg)
{
	if(ret < 0)
	{
		std::cerr << "ERROR" << std::endl;
		std::cerr << std::strerror(errno) << std::endl;
		std::cerr << msg << std::endl;
		exit(1);
	}
}

