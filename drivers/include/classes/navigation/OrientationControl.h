/* *
 * */

#include <ros/ros.h>
#include <string.h>
#include <ros/console.h>
#include <drivers/orientation_msg.h>
#include <drivers/control_motion_msg.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <cmath>
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
  
class OrientationControl
{
    public:
	OrientationControl(ros::NodeHandle);

    private:
	std::string node_name;
	ros::NodeHandle nh;
	ros::Subscriber subImu, subOrientation, subVelocity, subActivate;
	ros::Publisher pubControlMotion;
	void OrientationGoalCallback(const drivers::orientation_msg &orientation);
	void ImuCallback(const sensor_msgs::Imu &msg);
	void goalCB();
	void executeCB(){}
	drivers::orientation_msg orientation_msg, prev_orientation_msg;
	drivers::control_motion_msg control_motion_msg;
	double roll, pitch, yaw;
	double goal;
	double kp, kd, ki, error_i, error_d, prev_error,error_threshold;
	double angular_max{10.0};
	ros::Time current_time, last_time, last_angular_rate, last_wait;
	std::string goal_mode {"setpoint"};
	bool meters_per_second_enabled{false};
	int speed_conversion_const{1};
	double temp_angular_rate{0};
	bool wait_flag;
	ros::Timer gainsTimer;
	void ChangeControlGains();
};
  
OrientationControl::OrientationControl (ros::NodeHandle nh_priv)
{
    node_name = ros::this_node::getName	();
    int log_level;
    nh_priv.param("log_level", log_level,0);
    /** Constantes de control **/
    nh_priv.param("kp", kp,0.01);
    nh_priv.param("kd", kd,0.0);
    nh_priv.param("ki", ki,0.0);
    
    /**  Umbral de error permitido alrededor de orientacion meta  **/
    nh_priv.param("error_threshold", error_threshold,0.1); 
    
    /** Valor maximo al que se satura la velocidad angular **/
    nh_priv.param("angular_max", angular_max,10.0);
    
    /** Se puede elegir entre modo offset o modo setpoint **/
    nh_priv.param("goal_mode", goal_mode, goal_mode);
    
    /** True en caso de que el motor driver maneje m/s **/
    nh.param("meters_per_second_enabled", meters_per_second_enabled, false);
    ros::console::levels::Level console_level;
    console_level = (ros::console::levels::Level)log_level;
    //~ ROS_INFO_STREAM(node_name << " --- Level: " << console_level);
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    if(meters_per_second_enabled) speed_conversion_const = 0.01;
	subOrientation = nh.subscribe("control_topic/orientation",10,&OrientationControl::OrientationGoalCallback, this);
	subImu = nh.subscribe("sensor_topic/imu",10,&OrientationControl::ImuCallback, this);
	pubControlMotion = nh.advertise<drivers::control_motion_msg>("control_topic/motion", 10);
	//~ gainsTimer = nh.createTimer(ros::Duration(1), std::bind(&OrientationControl::ChangeControlGains, this));
	error_d = 0;
	error_i = 0;
	control_motion_msg.DrivingMode = 2;
}
/** Callback que recibe orientacion del IMU, en esta funcion se lleva a 
 * cabo el proceso principal de control**/
void OrientationControl::ImuCallback(const sensor_msgs::Imu &msg)
{
	tf::Quaternion quat(msg.orientation.x,msg.orientation.y, msg.orientation.z, msg.orientation.w);
	tf::Matrix3x3 m(quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	//~ ROS_INFO_STREAM("Yaw: " << yaw);
	if (!prev_orientation_msg.enabled)
		return;
	current_time = ros::Time::now();
	ros::Duration angular_rate_elapsed_time = current_time - last_angular_rate;
	ros::Duration wait_elapsed_time = current_time - last_wait;
	/** Para mayor comodidad de los teleoperadores se agrego el giro tanque 
	 * Si llega una instruccion de giro tanque se ignora el control 
	 * y se deja pasar esta velocidad sin ningun procesamiento**/
	if (temp_angular_rate != 0)
	{
		ROS_DEBUG("Giro Tanque");
		control_motion_msg.DriveAngularRate = temp_angular_rate;
		last_angular_rate = current_time;
	}
	/** Despues de un giro tanque se espera un tiempo en lo que
	 * se sincroniza el valor de la orientacion en la UI y en el brain**/
	//~ else if (angular_rate_elapsed_time.toSec() < 2.25 )
	//~ {
		//~ ROS_DEBUG_STREAM("Time threshold: " << angular_rate_elapsed_time.toSec());
		//~ control_motion_msg.DriveAngularRate = 0;
	//~ }
	/** Cuando inicial la teleoperacion se espera un tiempo en lo que
	 * se sincroniza el valor de la orientacion en la UI y en el brain**/
	else if (wait_elapsed_time.toSec() < 5.0 )
	{
		ROS_DEBUG_STREAM("WAIT Time threshold: " << wait_elapsed_time.toSec());
		control_motion_msg.DriveAngularRate = 0;
		control_motion_msg.DriveSpeed = 0;
	}
	else
	{
		ros::Duration dt = current_time - last_time;
		/** Calculo de error **/
		double error = (goal - yaw);
		/** error % 2 * M_PI **/
		int divisor = error / (2 * M_PI);
		error = error - (2 * M_PI * divisor*1.0);
		/** Se ajusta de manera que siempre haga el recorrido mas corto,
		 * maximo media vuelta**/
		if (abs	(error) > M_PI && error < 0)
			error += 2 * M_PI;
		else if (abs(error) > M_PI && error > 0)
			error -= 2 * M_PI;
		/** Se verifica si el error esta en el rango permitido alrededor de la meta **/
		if (error < error_threshold && error > -error_threshold)
		{
			//~ ROS_INFO_STREAM("Goal succeeded: " << yaw);
			control_motion_msg.DriveAngularRate = 0;
			pubControlMotion.publish(control_motion_msg);
			error_d = 0;
			error_i = 0;
			return;
		}
		/** Se calcula la derivada del error **/
		error_d = (error - prev_error) / dt.toSec();
		/** Se calcula la integral del error **/
		error_i += error * dt.toSec(); 
		/** Se calcula la velocidad angular en funcion del error**/
		float angular = error * kp + error_i * ki + error_d *kd;
		/** En caso de que la velocidad angular calculada sea mayor a la maxima, esta se satura **/
		if (angular > angular_max)
			control_motion_msg.DriveAngularRate = angular_max;
		else if(angular < (-1.0 * angular_max))
			control_motion_msg.DriveAngularRate = -1 * angular_max;
		else 
			control_motion_msg.DriveAngularRate = angular;
		//~ ROS_INFO_STREAM("Current error: " << error);
	}
	pubControlMotion.publish(control_motion_msg);
	last_time = current_time;
		
}
/** Este callback recibe el setpoint/offset, velocidades angulares y lineales,
 * asi como los mensajes de activacion de este modo de control **/
void OrientationControl::OrientationGoalCallback(const drivers::orientation_msg &orientation)
{
	ROS_INFO_STREAM("Orientation callback ");
	if (goal_mode.compare("offset")==0)
	{
		if(orientation.enabled && !prev_orientation_msg.enabled  )
		{
			goal = yaw;
			ROS_DEBUG_STREAM("Goal offset accepted: " << goal);
			last_time = last_wait= ros::Time::now();
			error_i = 0;
			error_d = 0;
		}
		if(orientation.enabled)
		{
			goal = (yaw + orientation.offset);
			control_motion_msg.DriveSpeed = orientation.speed;// * speed_conversion_const;
			temp_angular_rate = orientation.angular_rate;
		}
	}else if (goal_mode.compare("setpoint")==0)
	{
		if(orientation.enabled && !prev_orientation_msg.enabled  )
		{
			goal = orientation.setpoint;
			ROS_DEBUG_STREAM("Goal setpoint accepted: " << goal);
			last_time = last_wait= ros::Time::now();
			error_i = 0;
			error_d = 0;
		} 
		if(orientation.enabled)
		{ 
			goal = orientation.setpoint;
			control_motion_msg.DriveSpeed = orientation.speed;
			temp_angular_rate = orientation.angular_rate;
		}
	}
	if (!orientation.enabled && prev_orientation_msg.enabled)
	{
		control_motion_msg.DriveSpeed = 0;
		control_motion_msg.DriveAngularRate = 0;
		pubControlMotion.publish(control_motion_msg);
	}
	prev_orientation_msg = orientation;
}

void OrientationControl::ChangeControlGains()
{
    /** Constantes de control **/
    nh.param("orientation_control_node/kp", kp,0.01);
    nh.param("orientation_control_node/kd", kd,0.0);
    nh.param("orientation_control_node/ki", ki,0.0);
    ROS_DEBUG_STREAM("Gains accepted - kp: " << kp << ", ki: " << ki << ", kd: " << kd);
}
