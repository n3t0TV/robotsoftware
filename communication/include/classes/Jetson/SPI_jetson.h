#include <time.h>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <fstream>
#include <stdio.h>
#include <unistd.h>

#include "libraries/spi/spi.h"

#include <drivers/firmware_command.h>
#include <drivers/spi_test_service.h>
#include <std_srvs/SetBool.h>

#define FW_CHANNEL	0
#define WAIT_SLAVE_RESPONSE 0xFFFF

/*LCD*/
#define COMMAND 0
#define DATA    1

using namespace std;

class SPI
{
    public: 
	ros::NodeHandle nh;
	SPI(ros::NodeHandle nh_priv);
    
    private:
	string node_name;
	    
	ros::ServiceServer spi_firmware_srv;
	ros::ServiceServer spi_test_srv;
    
	int speed_ch0, speed_ch1;
	int SPI_Initialize();
    
	unsigned long FirmwareSPI(unsigned int instruction, unsigned int data);
	unsigned long FirmwareSPI2(unsigned int instruction, int data);
    
	bool spiFirmwareService(drivers::firmware_command::Request& req, drivers::firmware_command::Response& res);
	bool spiTestService(drivers::spi_test_service::Request& req, drivers::spi_test_service::Response& res);
	    
};

SPI::SPI(ros::NodeHandle nh_priv)
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
    //~ nh_priv.param("speed_ch0", speed_ch0,4000000);
    //~ nh_priv.param("speed_ch1", speed_ch1,4000000);
    speed_ch0 = 4000000;
    speed_ch1 = 4000000;
    
    spi_firmware_srv = nh.advertiseService("spi_firmware",&SPI::spiFirmwareService, this);
    spi_test_srv = nh.advertiseService("spi_test", &SPI::spiTestService, this);
    
    SPI_Initialize();
}

int SPI::SPI_Initialize()
{
    int fd;
    
	if (fd = spiSetupMode(FW_CHANNEL, speed_ch0,0) < 0)
    {
        ROS_WARN_STREAM(node_name << " --- Unable to open SPI channel 0: " << strerror (errno));
        return -1;
    }
    else
    {
        ROS_DEBUG_STREAM(fd);
    }

    return 0;
}

unsigned long SPI::FirmwareSPI(unsigned int instruction, unsigned int data)
{
    //~ ROS_DEBUG_STREAM("F");
   uint8_t byte_array[4];
   
   byte_array[3] = data & 0x00FF;
   byte_array[2] = data >> 8;
   byte_array[1] = instruction & 0x00FF;
   byte_array[0] = instruction >> 8;
   
   spiDataRW (FW_CHANNEL, byte_array, 4);
   usleep(2000);
   
   uint8_t dummy_byte_array[4];
   
   dummy_byte_array[3] = WAIT_SLAVE_RESPONSE & 0x00FF;
   dummy_byte_array[2] = WAIT_SLAVE_RESPONSE >> 8;
   dummy_byte_array[1] = 0;
   dummy_byte_array[0] = 0;
   
   spiDataRW (FW_CHANNEL, byte_array, 4);
   usleep(2000);

   return (byte_array[3] | (byte_array[2] << 8) | (byte_array[1] << 16) | (byte_array[0] << 24));
   
}

unsigned long SPI::FirmwareSPI2(unsigned int instruction, int data)
{
   uint8_t byte_array[4];
   
   byte_array[3] = data & 0x00FF;
   byte_array[2] = data >> 8;
   byte_array[1] = instruction & 0x00FF;
   byte_array[0] = instruction >> 8;
   
   spiDataRW (FW_CHANNEL, byte_array, 4);
   
   uint8_t dummy_byte_array[4];
   
   dummy_byte_array[3] = WAIT_SLAVE_RESPONSE & 0x00FF;
   dummy_byte_array[2] = WAIT_SLAVE_RESPONSE >> 8;
   dummy_byte_array[1] = 0;
   dummy_byte_array[0] = 0;
   
   spiDataRW (FW_CHANNEL, byte_array, 4);
   
   return (byte_array[3] | (byte_array[2] << 8) | (byte_array[1] << 16) | (byte_array[0] << 24));
   
}

bool SPI::spiFirmwareService(drivers::firmware_command::Request& req, drivers::firmware_command::Response& res)
{	
    ROS_DEBUG_STREAM("Firmware SPI service called");
    res.result = FirmwareSPI(req.command, req.parameter);
    return true;
}

typedef union
{
    uint32_t as_uint32;
    int32_t as_int32;
    float as_float;
    int16_t as_int16[2];
    uint16_t as_uint16[2];
    int8_t as_int8[4];
    uint8_t as_uint8[4];
} reinterpreted_data_t;

bool SPI::spiTestService(drivers::spi_test_service::Request& req, drivers::spi_test_service::Response& res)
{	
    res.result = true;
    reinterpreted_data_t spi_test;
    for(int i =0;i<10;i++)
    {
        FirmwareSPI(1,'Z');
        spi_test.as_uint8[0] = FirmwareSPI(2,0);
        if(spi_test.as_uint8[0] != 'Z')
            res.result = false;
    }
    return true;
}

