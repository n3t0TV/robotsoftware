#include <ros/ros.h>
#include <string>
#include <fstream>
#include <sys/stat.h>

using namespace std;

//PWM Class for SYSFS implementation
class PWM
{
	public:
		PWM(int channel, int frequency_hz);
		~PWM();
		void SetDutyCycle(int duty_cycle_us);
		void Start();
		void Stop();
	private:
		int _channel;
		int _period_ns;
		int _duty_cycle_ns{0};
		bool _enable{false};
		bool _pwm_ok{false};
		string pwm_path;
		const string pwm_sys_path = "/sys/class/pwm/pwmchip0";
};

PWM::PWM(int channel, int frequency_hz)
{
	_channel = channel;
    struct stat pwm_dir_st = {0};
	pwm_path = pwm_sys_path + "/pwm" + to_string(_channel);

	if(stat(pwm_path.c_str(), &pwm_dir_st) == -1)
    {
		/* No se ha exportado el pwm driver*/
		{//scope for o_fs
			ofstream f_export(pwm_sys_path + "/export"s);

			if(!f_export.is_open()) throw runtime_error("Can't open " + pwm_sys_path);

			f_export << _channel;
		}

		if(stat(pwm_path.c_str(), &pwm_dir_st) == -1)
		{
			ROS_ERROR_STREAM("PWM pin can't be exported!");
		} else if (S_ISDIR(pwm_dir_st.st_mode))
		{
			ros::Duration(1).sleep();		//Delay required to write fs
			{//scope for o_fs
				_period_ns = int(1000000000.0 / frequency_hz);
				ofstream f_period(pwm_path.c_str() + "/period"s);

				if(!f_period.is_open()) throw runtime_error("Can't open " + pwm_path + "/period");
				f_period << _period_ns;
				f_period.close();
			}
			_pwm_ok = true;
		}
	} else if (S_ISDIR(pwm_dir_st.st_mode))
	{
		_pwm_ok = true;
		return;
	}
}

PWM::~PWM()
{
	if (_pwm_ok)
	{
		ofstream f_unexport(pwm_sys_path + "/unexport"s);
		f_unexport << _channel;
	}
}

void PWM::SetDutyCycle(int duty_cycle_us)
{
	if (_pwm_ok)
	{
		ROS_DEBUG_STREAM("Set PWM duty cycle: " + to_string(duty_cycle_us));
		_duty_cycle_ns = duty_cycle_us * 1000;
		ofstream f_duty(pwm_path.c_str() + "/duty_cycle"s);

		if(!f_duty.is_open()) throw runtime_error("Can't open " + pwm_path + "/duty_cycle");
		f_duty << _duty_cycle_ns;
		f_duty.close();
	}
}

void PWM::Start()
{
	if (_pwm_ok)
	{
		_enable = true;
		ofstream f_enable(pwm_path.c_str() + "/enable"s);

		if(!f_enable.is_open()) throw runtime_error("Can't open " + pwm_path + "/enable");
		f_enable << (int) _enable;
		f_enable.close();
	}
}

void PWM::Stop()
{
	if (_pwm_ok)
	{
		_enable = false;
		ofstream f_enable(pwm_path.c_str() + "/enable"s);
		f_enable << (int) _enable;
		f_enable.close();
	}
}
