#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include "../pic_defines.h"

#include "libraries/uart/wiringSerial.h"

#include "std_srvs/Trigger.h"
#include "drivers/uart_test_service.h"

using namespace std;

class UART
{
    public:
        UART(ros::NodeHandle);
        ~UART();
        int Initialize();
    private:
        string node_name;
        int fd; 
        string  port;
        string  file;
        unsigned int address;
        int baudrate;
        string  device;
        int erase_size;
        int bootloader_size;
        int sector_size;
        bool boot;
        bool swap;
        string str_address;
        streampos bin_size;
        char * memblock;
        char * temp_buffer;
        char * data;
        //~ CRC::Table<std::uint32_t, 32> table(CRC::CRC_32());
        uint32_t crctable[256];
        std::uint32_t crc;
        bool ProgramPIC (std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
        char send_request(char , unsigned int , char* );
        char get_char();
        bool read_binfile();
        void crc32_tab_gen(uint32_t(&table)[256]);
        uint32_t update(uint32_t (&table)[256],char* buf, size_t len);
        ros::NodeHandle nh;
        ros::ServiceServer program_pic_service;
        ros::ServiceServer uart_test_srv;        
        bool uartTestService(drivers::uart_test_service::Request& req, drivers::uart_test_service::Response& res);
};

UART::UART(ros::NodeHandle nh_priv)
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
    nh_priv.param<string>("port", port, "/dev/ttyTHS2");
    //~ nh_priv.param<string>("file", file, "/home/pi/wagon-pic32mk.production.bin");
    nh_priv.param<string>("file", file, "/usr/bin/share/binarios-pic/firmwarewagon.production.bin");
    nh_priv.param<string>("device", device, "PIC32MK");
    nh_priv.param<string>("hex_address", str_address, "9d000000");
    nh_priv.param("baudrate", baudrate, 115200);
    nh_priv.param("sector_size", sector_size, 0);
    nh_priv.param<bool>("boot", boot, false);
    nh_priv.param<bool>("swap", swap, false);
    //~ nh_priv.param<uint>("address", address, 0x 9d 00 00 00);
    program_pic_service = nh.advertiseService("program_pic", &UART::ProgramPIC, this);
    uart_test_srv = nh.advertiseService("uart_test", &UART::uartTestService, this);
    Initialize();
    
}
UART::~UART()
{
    serialClose (fd) ;
}
int UART::Initialize ()
{
    if ((fd = serialOpen (port.c_str(), baudrate)) < 0)
    {
        fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
        return 1 ;
    }

    return 0;
}

char UART::send_request(char cmd, unsigned int data_size, char * data )
{
    int buffer_size = 9 + data_size;
    char response;
    char buffer[buffer_size];
    memset(buffer, 0x00, buffer_size);
    *reinterpret_cast<int *>(buffer) = BL_GUARD;
    *reinterpret_cast<int *>(buffer+4) = data_size;
    buffer[8] = cmd;
    if (data_size>0)
        memcpy(buffer+9, data, (size_t)data_size);
    
    for (int i=0; i<buffer_size; i++)
    {
        //~ ROS_INFO("%i: %hhx",i, buffer[i]);
        serialPutchar(fd,buffer[i]);
    }
    response = get_char();
    return response;
}

char UART::get_char()
{
    char response;
    for (int i=0; i<3; i++)
    {
        if (serialDataAvail (fd))
        {
            response = serialGetchar (fd) ;
            return response;
        }
        else 
        {
            ROS_DEBUG_STREAM("no response received, retrying");
        }
        ros::Duration(0.2).sleep();
    }
    return response;
}

bool  UART::read_binfile ()
{
    
    ifstream binfile(file.c_str(), ios::in|ios::binary|ios::ate);
    if (binfile.is_open())
    {
        bin_size = binfile.tellg();
        memblock = new char [bin_size];
        binfile.seekg (0, ios::beg);
        binfile.read (memblock, bin_size);
        binfile.close();
        //~ ROS_INFO_STREAM( "the entire file content is in memory size:  "  << bin_size);
        return true;
    }
    else 
    {
        ROS_ERROR_STREAM ("Unable to open file: " << file);
        return false;
    }
}

void UART::crc32_tab_gen(uint32_t(&table)[256])
{
    uint32_t polynomial = 0xEDB88320;
    for (uint32_t i = 0; i < 256; i++) 
    {
        uint32_t c = i;
        for (size_t j = 0; j < 8; j++) 
        {
            if (c & 1) {
                c =  (c >> 1) ^ polynomial;
            }
            else {
                c >>= 1;
            }
        }
        table[i] = c;
        //~ ROS_DEBUG_STREAM("Tab gen "<< i <<": " << c);
    }
}

uint32_t UART::update(uint32_t (&table)[256], char* buffer, size_t len)
	{
		uint32_t c = 0xFFFFFFFF;
		for (size_t i = 0; i < len; ++i) 
		{
			c = table[(c ^ buffer[i]) & 0xFF] ^ (c >> 8);
            //~ ROS_DEBUG_STREAM("crc "<< i <<": " << c);
		}
		return c ;
	}

bool UART::ProgramPIC(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    res.success = false;
    ROS_INFO("PROGRAM PIC SERVICE");
    char response;
    int i = 0;
    bool error=false;
    temp_buffer = new char [4];
    //~ response = get_char();
    serialFlush (fd);
    ros::Duration(0.1).sleep();
    do
    {
        
        response = send_request(BL_CMD_READY, 4, temp_buffer);
        if (response!=BL_RESP_INIT)
        {
            ROS_ERROR_STREAM("Invalid response code: "<< response << ".");
            error = true;
        }
        else
        {
            ROS_INFO_STREAM("Valid response code: "<< response << ".");
            error = false;
        }
        
        i++;
        ros::Duration(0.1).sleep();
    }while(i<5 && error==true);
    if (error == true)
        return false;
    ros::Duration(3.0).sleep();
    serialFlush (fd);
    if (devices.contains(device))
    {
        if (device=="PIC32MX")
        { 
            if (sector_size <= 0)
            {
                ROS_INFO("Device sector size is required"); 
                return false;
            }
            else 
            {
                erase_size = sector_size;
            }
        }
        else
        {
            erase_size = devices[device][0];
            bootloader_size = devices[device][1];
        }
            
    }
    else
    {
        ROS_ERROR("Invalid device");
    }
    if (swap)
    {
        if((device != "SAME5X") && (device != "SAMD5X") && (device != "PIC32MZ") && (device != "PIC32MK") )
        {
            ROS_ERROR("Bank Swapping not supported on this device");
        }
    }
    
    for (int  i= 0; i < strlen(str_address.c_str()); i++)
    {
        if (! isxdigit(str_address[i]))
        {
            ROS_ERROR("Invalid address");
            return false;
        }
    }
    address = stoul (str_address, nullptr, 16);
    ROS_INFO_STREAM("Address: " << address);
    std::size_t found = device.find("SAM");
    if (found!=std::string::npos)
    {
        if (address< bootloader_size && !boot)
        {
            ROS_ERROR("Address is within the bootlaoder area, use --boot options to unlock writes");
            return false;
        }
    }
    else
    {
        if (boot)
        {
            ROS_ERROR("--boot option is not supported on this device");
            return false;
        }
    }
    
    if (!read_binfile())
        return false;
    int nblocks = bin_size/erase_size;
    if (bin_size%erase_size>0)
        nblocks++;
    int size =nblocks*erase_size;
    data = new char [size];
    memset(data, 0xFF, sizeof(data));
    memcpy(data, memblock, (size_t)bin_size);
    
    temp_buffer = new char [8];
    *reinterpret_cast<unsigned int *>(temp_buffer) = address;
    *reinterpret_cast<unsigned int *>(temp_buffer+4) = size;
    ROS_INFO_STREAM("UNLOCKING");
    response = send_request(BL_CMD_UNLOCK, 8, temp_buffer);
    ROS_INFO_STREAM("Response: " << response);
    if (response!=BL_RESP_OK)
    {
        ROS_ERROR_STREAM("Invalid response code: "<< response << ". Check that your file size and address are correct");
        return false;
    }
    ROS_INFO_STREAM("PROGRAMMING");
    temp_buffer = new char [erase_size+4];
    char blocks [nblocks][erase_size];
    ROS_INFO("Uploading %d blocks at address %d (0x%x)\n", nblocks, address, address);
    for (int i=0; i<nblocks; i++)
    {
        *reinterpret_cast<unsigned int *>(temp_buffer) = address+i*erase_size;
        memcpy(temp_buffer+4,data+i*erase_size, (size_t)erase_size);
        response = send_request(BL_CMD_DATA, erase_size+4, temp_buffer);
        if (response!=BL_RESP_OK)
        {
            ROS_ERROR_STREAM("Invalid response code: "<< response );
            return false;
        }
        
    }
    crc32_tab_gen(crctable);
    crc = update(crctable,data,size);
    //~ ROS_INFO("CRC: (0x%x)" , crc);
    ROS_INFO_STREAM("VERIFICATION");
    temp_buffer = new char [4];
    *reinterpret_cast<unsigned int *>(temp_buffer) = crc;
    response = send_request(BL_CMD_VERIFY, 4, temp_buffer);
    if (response!=BL_RESP_CRC_OK)
    {
        ROS_ERROR_STREAM("Invalid response code: "<< response );
        return false;
    }
    temp_buffer = new char [16];
    if (swap)
    {
        ROS_INFO_STREAM("SWAPPING BANK AND REBOOTING");
        response = send_request(BL_CMD_BKSWAP_RESET, 4, temp_buffer);
    }
    else
    {
        ROS_INFO_STREAM("REBOOTING");
        response = send_request(BL_CMD_RESET, 4, temp_buffer);
    }
    if (response!=BL_RESP_OK)
    {
        ROS_ERROR_STREAM("Invalid response code: "<< response );
        return false;
    }else
    {
        res.success = true;
        return true;
    }
}

bool UART::uartTestService(drivers::uart_test_service::Request& req, drivers::uart_test_service::Response& res)
{
    ROS_DEBUG_STREAM("UART test service called!");
    int count;
	char out = 'Z',in;
	//~ int out = 1;
    //~ char in;
	res.result = true;
    
    for(count=0;count<10;count++)
	{
		serialPutchar(fd,out);		
		/*while (serialDataAvail (fd))
		{	
            ROS_DEBUG_STREAM("...");		
			in = serialGetchar(fd);
            ROS_DEBUG_STREAM(in);
			fflush (stdout);
            usleep(10000);
		}*/
        
        usleep(1000);
        in = serialGetchar(fd);
        ROS_DEBUG_STREAM(in);
        fflush (stdout);        
		
		if(count != 0 and in != 'Z')
		{
			res.result = false;
		}
			
		usleep(1000);		
	}
    ROS_DEBUG_STREAM("UART test service done!");
}

