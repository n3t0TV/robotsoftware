#include <gpiod.h>
#include <unistd.h>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include <drivers/sensor_picmanager_msg.h>
#include <ros/console.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include "classes/SPI/SPIDriver.h"

// Pin definitions (Sysfs numbering)
#define RESET_PIN 62
#define BOOT_PIN 66
#define UPS_EN_PIN 216
#define LPM_PIN 149  // low power mode (disable wifi/bt/ucper)

class PicManagerDriver : virtual public SPIDriver {
 public:
  PicManagerDriver(ros::NodeHandle nh_priv);
  virtual ~PicManagerDriver();
  int Initialize();
  int GetSensors();
  void InitUPS();
  void DeinitUPS();

 private:
  std::vector<int> battery_data;

  ros::ServiceServer reset_pic_service, force_bootloader_srv,
      init_program_pic_srv;
  ros::ServiceClient program_pic_srv;
  ros::Publisher pubSensors;
  drivers::sensor_picmanager_msg sensor_msg;

  const char* chipname = "gpiochip0";  // gpios del SOM
  struct gpiod_chip* chip;
  struct gpiod_line *linePicReset, *linePicBoot, *lineUpsEnable, *lineLpm;

  // srvs
  bool ForceBootloaderSrv(std_srvs::Empty::Request& req,
                          std_srvs::Empty::Response& res);
  bool Reset_PIC_service(std_srvs::Empty::Request& req,
                         std_srvs::Empty::Response& res);
  bool ProgramPICInitialize(std_srvs::Trigger::Request& req,
                            std_srvs::Trigger::Response& res);

  int InitResetPIC();
  void ResetPIC();

  bool ForceBootloader();
  bool ProgramPICVehicle();
};

PicManagerDriver::PicManagerDriver(ros::NodeHandle nh_priv)
    : SPIDriver(nh_priv) {
  commandsDir = "/config/wagonIndiaPicManager.json";

  pubSensors = nh.advertise<drivers::sensor_picmanager_msg>(
      "sensor_topic/picmanager", 10);
  reset_pic_service = nh.advertiseService(
      "reset_pic_service", &PicManagerDriver::Reset_PIC_service, this);
  force_bootloader_srv = nh.advertiseService(
      "force_bootloader", &PicManagerDriver::ForceBootloaderSrv, this);
  init_program_pic_srv = nh.advertiseService(
      "init_program_pic", &PicManagerDriver::ProgramPICInitialize, this);
  program_pic_srv = nh.serviceClient<std_srvs::Trigger>("program_pic");

  // Open GPIO chip
  chip = gpiod_chip_open_by_name(chipname);
  if (chip == NULL) {
    ROS_ERROR_STREAM("GPIO error: chip is null");
  }

  // Open GPIO lines
  linePicReset = gpiod_chip_get_line(chip, RESET_PIN);
  linePicBoot = gpiod_chip_get_line(chip, BOOT_PIN);
  lineUpsEnable = gpiod_chip_get_line(chip, UPS_EN_PIN);
  lineLpm = gpiod_chip_get_line(chip, LPM_PIN);
  if (linePicReset == NULL || linePicBoot == NULL || lineUpsEnable == NULL ||
      lineLpm == NULL) {
    ROS_ERROR_STREAM("GPIO error: lines are null");
  }
}

PicManagerDriver::~PicManagerDriver() {
  ROS_DEBUG_STREAM("destructor PicManagerDriver");

  gpiod_line_release(linePicReset);
  gpiod_line_release(linePicBoot);
  gpiod_line_release(lineUpsEnable);
  gpiod_line_release(lineLpm);
  gpiod_chip_close(chip);
}

void PicManagerDriver::InitUPS() {
  // set pin as an output pin with optional initial state of HIGH
  gpiod_line_request_output(lineUpsEnable, "UPS_ENABLE_LINE",
                            1);  // init value=HIGH
}

void PicManagerDriver::DeinitUPS() { gpiod_line_set_value(lineUpsEnable, 0); }

int PicManagerDriver::Initialize() {
  int nRet;
  // Initialization for generic driver
  nRet = SPIDriver::Initialize();

  // Initialization for specific driver
  InitResetPIC();
  ResetPIC();

  return nRet;
}

int PicManagerDriver::InitResetPIC() {
  // Open line for output
  gpiod_line_request_output(linePicReset, "PIC_RESET_LINE", 0);  // init value=0
  gpiod_line_request_output(lineLpm, "LPM_LINE", 0);             // init value=0

  return 0;
}

void PicManagerDriver::ResetPIC() {
  // Set a value (output)
  gpiod_line_set_value(linePicReset, 1);
  usleep(50000);
  gpiod_line_set_value(linePicReset, 0);
}

bool PicManagerDriver::Reset_PIC_service(std_srvs::Empty::Request& req,
                                         std_srvs::Empty::Response& res) {
  ROS_DEBUG_STREAM(node_name << " --- Reset_PIC service called");
  ResetPIC();
  return true;
}

bool PicManagerDriver::ForceBootloader() {
  bool return_value;
  gpiod_line_request_output(linePicBoot, "PIC_BOOT_LINE", 1);
  ResetPIC();
  sleep(1);
  gpiod_line_set_value(linePicBoot, 0);
  sleep(1);

  std_srvs::Trigger program_pic_msg;
  if (program_pic_srv.call(program_pic_msg))
    return_value = program_pic_msg.response.success;
  else
    return_value = false;

  return return_value;
}

bool PicManagerDriver::ForceBootloaderSrv(std_srvs::Empty::Request& req,
                                          std_srvs::Empty::Response& res) {
  ForceBootloader();
  return true;
}

bool PicManagerDriver::ProgramPICVehicle() {
  bool return_value;
  std_srvs::Trigger program_pic_msg;
  SetCommand("DRIVE_ENABLE_DRIVERS", 0);
  ros::Duration(0.5).sleep();
  SetCommand("START_BOOTLOADER", 0);
  ros::Duration(0.5).sleep();
  GetCommand<int>("WAGON_STATUS", 0);
  ros::Duration(0.5).sleep();
  if (program_pic_srv.call(program_pic_msg))
    return_value = program_pic_msg.response.success;
  else
    return_value = false;
  SetCommand("DRIVE_ENABLE_DRIVERS", 1);
  // TODO: Test enable command
  return return_value;
}

bool PicManagerDriver::ProgramPICInitialize(std_srvs::Trigger::Request& req,
                                            std_srvs::Trigger::Response& res) {
  res.success = ProgramPICVehicle();
  if (!res.success) ResetPIC();
  return true;
}

int PicManagerDriver::GetSensors() {
  int nRet = 0;
  unsigned long version = GetCommand<unsigned int>("WAGON_VERSION");
  char* version_ch = (char*)&version;
  string version_str = version_ch;
  bool valid_version = true;
  if (version_str.size() <= 0)
    valid_version = false;
  else {
    for (int i = 0; i < 4; i++) {
      if (!((version_str.at(i) >= 65 && version_str.at(i) <= 90) ||
            (version_str.at(i) >= 97 && version_str.at(i) <= 122) ||
            (version_str.at(i) >= 48 && version_str.at(i) <= 57))) {
        valid_version = false;
        break;
      }
    }
  }
  if (valid_version) {
    sensor_msg.Version = version_str.substr(0, 4);
  } else {
    /** Error de comunicacion SPI */
    sensor_msg.Version = "";

    nRet = -1;
  }

  int battery_temp = (int)GetCommand<int>("BATTERY_LEVEL", false);

  if ((battery_temp > 100)) {
    sensor_msg.Charging = true;
    battery_temp = 100;
  } else {
    sensor_msg.Charging = false;
  }
  if ((battery_temp < 0)) {
    battery_temp = 0;
    ROS_WARN_STREAM("Battery value out of range: " << battery_temp);
    nRet = -1;
  }

  battery_data.push_back(battery_temp);
  if (battery_data.size() > 100) battery_data.erase(battery_data.begin());
  sensor_msg.BatteryLevel =
      (int)accumulate(battery_data.begin(), battery_data.end(), 0.0) /
      battery_data.size();
  sensor_msg.BatteryVoltage = GetCommand<float>("BATTERY_VOLTAGE");

  /*Obtencion de errores*/
  // Generales
  sensor_msg.WagonError1 = GetCommand<unsigned int>("WAGON_ERROR_1");
  sensor_msg.WagonError2 = GetCommand<unsigned int>("WAGON_ERROR_2");
  sensor_msg.WagonSensor = GetCommand<unsigned int>("WAGON_SENSOR");

  // Particulares
  sensor_msg.WagonDriveRightError =
      GetCommand<unsigned int>("WAGON_DRIVE_R_ERROR", false);
  sensor_msg.WagonDriveLeftError =
      GetCommand<unsigned int>("WAGON_DRIVE_L_ERROR", false);
  sensor_msg.WagonFailSafeMode =
      GetCommand<unsigned int>("DRIVE_FAIL_SAFE_MODE");
  sensor_msg.WagonFailSafeRelease =
      GetCommand<unsigned int>("DRIVE_FAIL_SAFE_RELEASE");

  pubSensors.publish(sensor_msg);

  return nRet;
}
