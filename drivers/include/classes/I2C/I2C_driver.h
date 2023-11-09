#ifndef I2CDrivr
#define I2CDrivr

#include <ros/ros.h>
#include "drivers/i2c_srv.h"
static const uint8_t I2C_READ = 0;
static const uint8_t I2C_WRITE = 1;
class I2C_Driver
{
	public:
		I2C_Driver(ros::NodeHandle nh_priv, uint8_t address);
	protected:
		ros::NodeHandle nh;
		ros::NodeHandle nh_priv;
		drivers::i2c_srv service_msg;
		ros::ServiceClient srvI2C;
		std::string node_name;
		bool ReadI2C(uint8_t register_address, uint8_t read_size, uint8_t *response);
		uint8_t ReadI2Cbyte(uint8_t register_address);
		bool WriteI2C(uint8_t register_address, uint8_t *data, size_t size);
		bool WriteI2Cbyte(uint8_t register_address, uint8_t data);
};


I2C_Driver::I2C_Driver(ros::NodeHandle nh_priv, uint8_t address): nh_priv(nh_priv)
{
	node_name = ros::this_node::getName	();
    int log_level;
    nh_priv.param("log_level", log_level,0);
    ros::console::levels::Level console_level;
    console_level = (ros::console::levels::Level)log_level;
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
	srvI2C = nh.serviceClient<drivers::i2c_srv>("i2c_srv");
	 service_msg.request.dev_address = address;
}

bool I2C_Driver::ReadI2C(uint8_t register_address, uint8_t read_size, uint8_t *response)
{
	uint8_t *temp_buffer;
	service_msg.request.action = I2C_READ;
	service_msg.request.register_address = register_address;
	service_msg.request.read_size = read_size;
	if(srvI2C.call(service_msg))
	{
		std::copy(service_msg.response.response.begin(), service_msg.response.response.end(), response);
		// ROS_DEBUG_STREAM("Read response" << *response);
		return true;
	}
	return false;
}
uint8_t I2C_Driver::ReadI2Cbyte(uint8_t register_address)
{
	uint8_t response;
	ReadI2C(register_address, 1, &response);
	return response;
}

bool I2C_Driver::WriteI2C(uint8_t register_address, uint8_t *data, size_t size)
{
	service_msg.request.action = I2C_WRITE;
	service_msg.request.register_address = register_address;
	service_msg.request.read_size = 0;
	service_msg.request.data.assign(data,data+size);
	if(srvI2C.call(service_msg))
		return true;
	return false;
}

bool I2C_Driver::WriteI2Cbyte(uint8_t register_address, uint8_t data)
{
	return WriteI2C(register_address, &data, 1);
}

#endif
