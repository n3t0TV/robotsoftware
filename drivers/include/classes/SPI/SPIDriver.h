#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <ros/console.h>
#include <ros/package.h>
#include <time.h> 
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <std_msgs/Byte.h>
#include "libraries/DataUtil.hpp"
#include "libraries/json.hpp"
#include "libraries/exceptions/Exceptions.h"
#include <drivers/firmware_command.h>

#ifndef SPIDrivr
#define SPIDrivr

const char *homedir;

using namespace std;
using json = nlohmann::json;

class SPIDriver
{
    public:
	SPIDriver(ros::NodeHandle);
	virtual ~SPIDriver();
	json OpenJSONFile(const char * file);
	virtual int Initialize();
	
    protected:
	ros::NodeHandle nh;
	unsigned long spiTransmit(unsigned int instruction, unsigned int data);
	ros::ServiceClient spi_firmware_srv;
	drivers::firmware_command::Request  spi_firmware_req;
	drivers::firmware_command::Response  spi_firmware_res;
	ros::Publisher pubSPIError;
	ReinterpretedData temp;

	template <typename T>
	T GetCommand(string, bool validate=true);
	template <typename T>
	int SetCommand(string Command, T data, bool validate=true);

	string commandsDir;
	json jsonCommands;
	string node_name;

};

SPIDriver::SPIDriver(ros::NodeHandle nh_priv)
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

    if ((homedir = getenv("HOME")) == NULL) {
		homedir = getpwuid(getuid())->pw_dir;
	}

    spi_firmware_srv = nh.serviceClient<drivers::firmware_command>("spi_firmware");
    pubSPIError = nh.advertise <std_msgs::Byte>("exception_topic/spi", 1);
}

SPIDriver::~SPIDriver()
{
    ROS_DEBUG_STREAM("destructor DRIVER");
}

int SPIDriver::Initialize()
{
    ROS_DEBUG_STREAM("File:" << ros::package::getPath("drivers") << commandsDir);
    jsonCommands = OpenJSONFile((ros::package::getPath("drivers") + commandsDir).c_str()); 
    ROS_DEBUG_STREAM(ros::package::getPath("drivers") << commandsDir << "> " << jsonCommands);

    return 0;
}

json SPIDriver::OpenJSONFile(const char * file)
{ 
    json j;
    std::ifstream i(file);
    if (i.is_open())
    {
	if (j.accept(i))
	{
	    std::ifstream i(file);
	    json j;
	    i >> j;
	    return j;
	}
	else
	    ROS_ERROR("Invalid JSON file");
    }
    else
	ROS_ERROR_STREAM("This File cannot be opened: " << string{file});
    return j;
}

unsigned long SPIDriver::spiTransmit(unsigned int instruction,unsigned int data)
{
    spi_firmware_req.command = instruction;
    spi_firmware_req.parameter = data;

    if (!spi_firmware_srv.call(spi_firmware_req, spi_firmware_res))
    {	
	std_msgs::Byte err_msg;
	err_msg.data = SPI_ERR_NODE;
	pubSPIError.publish(err_msg);
	ROS_ERROR_STREAM("Cannot call spi firmware service");
	return -1;
    }

    return spi_firmware_res.result; 
}

template <typename T>
T SPIDriver::GetCommand(string Command, bool validate)
{
    try {
	if (jsonCommands.contains(Command))
	{
	    int commandValue = jsonCommands[Command]["GET"];
	    T data;
	    //~ ROS_INFO_STREAM("Command: " << commandValue);
	    temp.as_uint= spiTransmit(commandValue, 0);
	    
	    if (jsonCommands[Command]["DATA TYPE"] == "accum")
	    { 
		data = ((float) temp.as_short[0])/128.0;
	    }
	    else if ((jsonCommands[Command]["DATA TYPE"] == "int32")||
	    (jsonCommands[Command]["DATA TYPE"] == "int32_t"))
	    {
	    
		data = temp.as_int;
	    }
	    else if ((jsonCommands[Command]["DATA TYPE"] == "uint32")||
	    (jsonCommands[Command]["DATA TYPE"] == "uint32_t"))
	    {

		data = temp.as_uint;
	    }
	    else if ((jsonCommands[Command]["DATA TYPE"] == "short")||
	    (jsonCommands[Command]["DATA TYPE"] == "int16_t"))
	    {

		data = temp.as_short[0];
	    }
	    else if ((jsonCommands[Command]["DATA TYPE"] == "ushort")||
	    (jsonCommands[Command]["DATA TYPE"] == "uint16_t"))
	    {

		data = temp.as_ushort[0];
	    }
	    else if ((jsonCommands[Command]["DATA TYPE"] == "uchar")||
	    (jsonCommands[Command]["DATA TYPE"] == "uint8_t"))
	    {

		data = temp.as_uchar[0];
	    }
	    else if (jsonCommands[Command]["DATA TYPE"] == "float")
	    {

		data = temp.as_float;
	    }
	    else
	    {
	    
		ROS_ERROR("No data type specified");
		return -1;
	    }

	    if (validate&&((data < jsonCommands[Command]["MIN"]) || (data > jsonCommands[Command]["MAX"])))
	    {
		ROS_WARN_STREAM("GET Value out of range --- Command: " <<Command << " --- Value: " << (T)data);
		return -1;
	    }
	    else
	    {
		return data;
	    }
	} 
	else
	{
	    ROS_WARN_STREAM("GET Command " << Command << "is not available");
	    return -1;
	}
    }
    catch (int e)
    {
	ROS_ERROR_STREAM("Error reading json file: " << e);
	return -1;
    }
}

template <typename T>
int SPIDriver::SetCommand(string Command, T data, bool validate)
{
    try
    {
	if(jsonCommands.contains(Command))
	{
	    //~ ROS_DEBUG_STREAM("SET Command: " <<Command << " --- Value: " << data);
	    int commandValue = jsonCommands[Command]["SET"];
	    ROS_DEBUG_STREAM("SET Command: " <<Command << " --- Value: " << commandValue << " --- Data: "<<data);

	    if (validate)
	    {
		if (data < jsonCommands[Command]["MIN"])
		{
		    ROS_WARN_STREAM("SET Value out of range --- Command: " <<Command << " --- Value: " << data);
		    data =  jsonCommands[Command]["MIN"];
		}
		else if (data > jsonCommands[Command]["MAX"])
		{
		    ROS_WARN_STREAM("SET Value out of range --- Command: " <<Command << " --- Value: " << data);
		    data =  jsonCommands[Command]["MAX"];
		}
	    }

	    if (jsonCommands[Command]["DATA TYPE"] == "accum")
	    { 
		temp.as_short[0] = (short) round(data *128);
	    }
	    else if ((jsonCommands[Command]["DATA TYPE"] == "int32")||
	    (jsonCommands[Command]["DATA TYPE"] == "int32_t"))
	    {
		temp.as_int = (int) data;
	    }
	    else if ((jsonCommands[Command]["DATA TYPE"] == "uint32")||
	    (jsonCommands[Command]["DATA TYPE"] == "uint32_t"))
	    {
		temp.as_int = (unsigned int) data;
	    }
	    else if ((jsonCommands[Command]["DATA TYPE"] == "short")||
	    (jsonCommands[Command]["DATA TYPE"] == "int16_t"))
	    {
		temp.as_short[0] = (short) data;
	    }
	    else if ((jsonCommands[Command]["DATA TYPE"] == "ushort")||
	    (jsonCommands[Command]["DATA TYPE"] == "uint16_t"))
	    {
		temp.as_ushort[0] = (unsigned short) data;
	    }
	    else if ((jsonCommands[Command]["DATA TYPE"] == "uchar")||
	    (jsonCommands[Command]["DATA TYPE"] == "uint8_t"))
	    {
		temp.as_uchar[0] = (unsigned char) data;
	    }
	    else
	    {
		return -1;
		ROS_ERROR("No data type specified");
	    }
	    if (spiTransmit(commandValue, temp.as_uint) == 0)
	    {
		// ROS_ERROR("SPI SET response equal to zero");
	    }
	    return 0;
	}
	else
	{
	    //~ ROS_WARN_STREAM("SET Command " << Command << "is not available");
	    return -1;
	}
    }
    catch (int e)
    {
	ROS_ERROR_STREAM("Error reading json file: " << e);
	return -1;
    }
}
#endif
