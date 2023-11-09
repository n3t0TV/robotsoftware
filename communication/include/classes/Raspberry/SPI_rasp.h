#include <time.h>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <fstream>
#include <stdio.h>
#include <unistd.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include <drivers/firmware_command.h>
#include <drivers/spi_display_service.h>
#include <drivers/spi_test_service.h>
#include <std_srvs/SetBool.h>

#define FW_CHANNEL	0
#define	DISPLAY_CHANNEL	1
#define WAIT_SLAVE_RESPONSE 0xFFFF

/*Wiring Pi for display*/
// WiringPi
#define DC		5    
#define RESET	4  

/*LCD*/
#define COMMAND 0
#define DATA    1

using namespace std;

class SPI
{
	public: 
		ros::NodeHandle nh;
		SPI(ros::NodeHandle nh_priv);
        bool displaySPIReady;
        
	private:
		string node_name;
		
		ros::ServiceServer spi_firmware_srv;
		ros::ServiceServer spi_display_srv;
		ros::ServiceServer spi_test_srv;
        ros::ServiceServer spi_display_ready_srv;
        
		int speed_ch0, speed_ch1;
		int SPI_Initialize();
        
		unsigned long FirmwareSPI(unsigned int instruction, unsigned int data);
		unsigned long FirmwareSPI2(unsigned int instruction, int data);
        
		bool spiFirmwareService(drivers::firmware_command::Request& req, drivers::firmware_command::Response& res);
		bool spiDisplayService(drivers::spi_display_service::Request& req, drivers::spi_display_service::Response& res);
        bool spiTestService(drivers::spi_test_service::Request& req, drivers::spi_test_service::Response& res);
        bool spiDisplayReadyService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
		
        void initDisplay();
        void writeToDisplay(int type, unsigned char* buffer,int len);
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
    spi_display_srv = nh.advertiseService("spi_display", &SPI::spiDisplayService, this);
    spi_test_srv = nh.advertiseService("spi_test", &SPI::spiTestService, this);
    spi_display_ready_srv = nh.advertiseService("display_ready_service", &SPI::spiDisplayReadyService, this);
    
    SPI_Initialize();
}

void SPI::initDisplay()
{
    pinMode(DC, OUTPUT);
    pinMode(RESET, OUTPUT);
    //~ pinMode(CE, OUTPUT);
     
    //Reset the LCD to a known state
    digitalWrite(RESET, LOW);
    usleep(100000);
    digitalWrite(RESET, HIGH);
    
    // Enable CE of display
    //~ pinMode(CE, LOW);
    
    unsigned char data_out[6];
    data_out[0] = 0x21;	//Tell LCD extended commands follow
    data_out[1] = 0xB8;	//Set LCD Vop (Contrast)
    data_out[2] = 0x04;	//Set Temp coefficent
    data_out[3] = 0x14;	//LCD bias mode 1:48 (try 0x13)
    data_out[4] = 0x20;	//We must send 0x20 before modifying the display control mode
    data_out[5] = 0x0D;	//Set display control, normal mode.
    digitalWrite(DC, LOW);
    wiringPiSPIDataRW (DISPLAY_CHANNEL, data_out, 6);
}

int SPI::SPI_Initialize()
{
    displaySPIReady = false;
    wiringPiSetup();
    int fd;
    
	if (fd = wiringPiSPISetupMode(FW_CHANNEL, speed_ch0,0) < 0)
    {
        ROS_WARN_STREAM(node_name << " --- Unable to open SPI channel 0: " << strerror (errno));
        return -1;
    }
    else
    {
        ROS_DEBUG_STREAM(fd);
    }
       
    if (wiringPiSPISetupMode(DISPLAY_CHANNEL, speed_ch1,0) < 0)
    {
        ROS_WARN_STREAM(node_name << " --- Unable to open SPI channel 1: " << strerror (errno));
        return -1;
    }
    else
    {
        ROS_DEBUG_STREAM(fd);
        displaySPIReady = true;
    }
    
    initDisplay();
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
   
   wiringPiSPIDataRW (FW_CHANNEL, byte_array, 4);
   usleep(2000);
   
   uint8_t dummy_byte_array[4];
   
   dummy_byte_array[3] = WAIT_SLAVE_RESPONSE & 0x00FF;
   dummy_byte_array[2] = WAIT_SLAVE_RESPONSE >> 8;
   dummy_byte_array[1] = 0;
   dummy_byte_array[0] = 0;
   
   wiringPiSPIDataRW (FW_CHANNEL, byte_array, 4);
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
   
   wiringPiSPIDataRW (FW_CHANNEL, byte_array, 4);
   
   uint8_t dummy_byte_array[4];
   
   dummy_byte_array[3] = WAIT_SLAVE_RESPONSE & 0x00FF;
   dummy_byte_array[2] = WAIT_SLAVE_RESPONSE >> 8;
   dummy_byte_array[1] = 0;
   dummy_byte_array[0] = 0;
   
   wiringPiSPIDataRW (FW_CHANNEL, byte_array, 4);
   
   return (byte_array[3] | (byte_array[2] << 8) | (byte_array[1] << 16) | (byte_array[0] << 24));
   
}

bool SPI::spiFirmwareService(drivers::firmware_command::Request& req, drivers::firmware_command::Response& res)
{	
	ROS_DEBUG_STREAM("Firmware SPI service called");
	res.result = FirmwareSPI(req.command, req.parameter);
    return true;
}

bool SPI::spiDisplayService(drivers::spi_display_service::Request& req, drivers::spi_display_service::Response& res)
{	 
    //~ ROS_DEBUG_STREAM("D");
    bool dc = req.data_command;
    uint8_t *buffer = req.write_buffer.data();
    if(dc)
    {
        ROS_DEBUG_STREAM("HIGH");    
         digitalWrite(DC, 1);
    }
    else
    {
        ROS_DEBUG_STREAM("LOW");    
         digitalWrite(DC, 0);             
    }
    wiringPiSPIDataRW (DISPLAY_CHANNEL, buffer, req.buffer_len);
    return true;
}

bool SPI::spiDisplayReadyService(std_srvs::SetBoolRequest&req, std_srvs::SetBoolResponse&res)
{
    res.success = displaySPIReady;
    if(displaySPIReady)
        res.message = "Display SPI ready"; 
    else
        res.message = "Display SPI not ready"; 
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
