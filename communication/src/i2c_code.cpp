#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <ros/ros.h>
#include "libraries/i2c/i2c.h"
#include "drivers/i2c_srv.h"
static const uint8_t I2C_READ = 0;
static const uint8_t I2C_WRITE = 1;

class I2C_Core
{
	public:
		I2C_Core(ros::NodeHandle nh_priv);
		~I2C_Core();
		int Initialize();
	private:
		ros::NodeHandle nh;
		ros::Subscriber subI2Cdevices;
		ros::ServiceServer srvI2C;
		int fd;
		std::string i2c_bus{"/dev/i2c-0"};
		std::string node_name;
		bool I2C_Service(drivers::i2c_srv::Request &req, drivers::i2c_srv::Response &res);
		
};

I2C_Core::I2C_Core (ros::NodeHandle nh_priv)
{
    node_name = ros::this_node::getName	();
    int log_level;
    nh_priv.param("log_level", log_level,0);
    nh_priv.param("i2c_bus", i2c_bus,i2c_bus);
    ros::console::levels::Level console_level;
    console_level = (ros::console::levels::Level)log_level;
    //~ ROS_INFO_STREAM(node_name << " --- Level: " << console_level);
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
	srvI2C = nh.advertiseService("i2c_srv", &I2C_Core::I2C_Service, this);
}


int I2C_Core::Initialize()
{
	if ((fd = i2c_open(i2c_bus.c_str())) == -1) {

        perror("Open i2c bus error");
        return -1;
    }
    return 0;
}

bool I2C_Core::I2C_Service(drivers::i2c_srv::Request &req, drivers::i2c_srv::Response &res)
{
	I2CDevice device;
	/* Fill i2c device struct */
    device.bus = fd;
    device.addr = req.dev_address;
    device.tenbit = 0;
    device.delay = 10;
    device.flags = 0;
    device.page_bytes = 8;
    device.iaddr_bytes = 1; /* Set this to zero, and using i2c_ioctl_xxxx API will ignore chip internal address */
    if (req.action==I2C_WRITE)
    {
		const char *data = (const char *) req.data.data();
		ROS_DEBUG_STREAM("data: " << (int)*data << ", strlen(data): " << req.data.size());
		/* Write data to i2c */
		ssize_t ret = i2c_ioctl_write(&device, req.register_address, data, req.data.size());
		if (ret == -1 || (size_t)ret != strlen(data))
		{
			return false;
		}
		res.response.assign(data, data + strlen(data));
	} else if(req.action==I2C_READ)
	{
		unsigned char buffer[req.read_size];
		ssize_t size{req.read_size};
		memset(buffer, 0, sizeof(buffer));
		if ((i2c_read(&device, req.register_address, buffer, size)) != size) 
		{
			return false;
		}
		res.response.assign(buffer, buffer+req.read_size);
	}
	return true;
}
I2C_Core::~I2C_Core()
{
	i2c_close(fd);
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"i2c_node");
	ROS_INFO_STREAM("i2c_node");
	ros::NodeHandle nh("~");
	ros::Rate rate(100);
	I2C_Core i2c(nh);
	i2c.Initialize();
	
	while(ros::ok())
	{
		ros::spinOnce();		
		rate.sleep();
	}
	return 0;
}

