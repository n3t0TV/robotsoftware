#include "classes/I2C/I2C_driver.h"
#include "libraries/Constants.h"
#include <drivers/pwm_service.h>

static const int MIN_DUTY_CYCLE = 0;
static const int MAX_DUTY_CYCLE = 4095;
static const int SIGNAL_PERIOD_USECS = 3333;
static const int DRIVER_RESOLUTION = 4096;

class PCA9685 : public I2C_Driver{
	public:
		PCA9685(ros::NodeHandle nh_priv, uint8_t address = 0x40);
		void Initialize();
		void set_pwm_freq(double freq_hz);
		void set_pwm(int channel, uint16_t on, uint16_t off);
		void set_all_pwm(uint16_t on, uint16_t off);
		void set_pwm_ms(int channel, double ms);
	private:
    ros::NodeHandle nh;
    ros::ServiceServer pwm_service, pwm_duty_service;
    bool update_pwm(drivers::pwm_service::Request& req, drivers::pwm_service::Response& res);
    bool update_pwm_duty(drivers::pwm_service::Request& req, drivers::pwm_service::Response& res);
		// Default frequency pulled from PCA9685 datasheet.
		double frequency = 200.0;
		void check_ret(int ret, std::string msg = "");
		
};
PCA9685::PCA9685(ros::NodeHandle nh_priv, uint8_t address): I2C_Driver{nh_priv, address}
{
	pwm_service = nh.advertiseService("pwm_servo_service", &PCA9685::update_pwm, this);
	pwm_duty_service = nh.advertiseService("pwm_duty_service", &PCA9685::update_pwm_duty, this);
}

void PCA9685::Initialize() {
  set_all_pwm(0,0);
  auto ret = WriteI2Cbyte( MODE2, OUTDRV);
  check_ret(ret, "set mode2");
  ret = WriteI2Cbyte( MODE1, ALLCALL);
  check_ret(ret, "set mode1");
  usleep(5'000);
  auto mode1 = ReadI2Cbyte(MODE1);
  check_ret(mode1, "read mode1");
  mode1 = mode1 & ~SLEEP;
  ret = WriteI2Cbyte( MODE1, mode1);
  check_ret(ret, "write mode1");
  usleep(5'000);
  set_pwm_freq(300); //300Hz
}

void PCA9685::set_pwm_freq(double freq_hz) {
  frequency = freq_hz;

  auto prescaleval = 25000000.0; //    # 25MHz
  prescaleval /= 4096.0; //       # 12-bit
  prescaleval /= freq_hz;
  prescaleval -= 1.0;

  auto prescale = static_cast<int>(std::round(prescaleval));

  auto oldmode = ReadI2Cbyte( MODE1);
  check_ret(oldmode);

  auto newmode = (oldmode & 0x7F) | SLEEP;
  auto ret = WriteI2Cbyte( MODE1, newmode);
  check_ret(ret);
  ret = WriteI2Cbyte( PRESCALE, prescale);
  check_ret(ret);
  ret = WriteI2Cbyte( MODE1, oldmode);
  check_ret(ret);
  usleep(5'000);
  ret = WriteI2Cbyte( MODE1, oldmode | RESTART);
  check_ret(ret);
}

bool PCA9685::update_pwm(drivers::pwm_service::Request& req, drivers::pwm_service::Response& res)
{
  set_pwm(req.channel, req.offTime, req.onTime);
  res.success = true;
  return true;
}

bool PCA9685::update_pwm_duty(drivers::pwm_service::Request& req, drivers::pwm_service::Response& res)
{
  int onTime = (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE)/(100 - 0) * req.pwmDuty;   //Revisar si es negada: (100 - req.pwmDuty);
  set_pwm(req.channel, 0, onTime);
  res.success = true;
  return true;
}

void PCA9685::set_pwm(int channel, uint16_t on, uint16_t off) {
  ROS_DEBUG_STREAM("channel: " << channel << ", on: " << on << ", off: " << off );
  WriteI2Cbyte( LED0_ON_L+4*channel, on & 0xFF);
  WriteI2Cbyte( LED0_ON_H+4*channel, on >> 8);
  WriteI2Cbyte( LED0_OFF_L+4*channel, off & 0xFF);
  WriteI2Cbyte( LED0_OFF_H+4*channel, off >> 8);
}

void PCA9685::set_all_pwm(uint16_t on, uint16_t off) {
  WriteI2Cbyte( ALL_LED_ON_L, on & 0xFF);
  WriteI2Cbyte( ALL_LED_ON_H, on >> 8);
  WriteI2Cbyte( ALL_LED_OFF_L, off & 0xFF);
  WriteI2Cbyte( ALL_LED_OFF_H, off >> 8);
}

void PCA9685::set_pwm_ms(int channel, double ms) {
  auto period_ms = 1000.0 / frequency;
  auto bits_per_ms = 4096 / period_ms;
  auto bits = ms * bits_per_ms;
  set_pwm(channel, 0, bits);
}

void PCA9685::check_ret(int ret, std::string msg) {
  if(ret < 0) {
    std::cerr << "ERROR" << std::endl;
    std::cerr << std::strerror(errno) << std::endl;
    std::cerr << msg << std::endl;
    exit(1);
  }
}
