/* *
 * References: 
 * 1. A Tutorial and Elementary Trajectory Model
 *    for the Differential Steering System of Robot Wheel Actuators
 *    http://rossum.sourceforge.net/papers/DiffSteer/ 
 * 2. Publishing Odometry Information over ROS
 * 	  http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
 * */

#include <ros/ros.h>
#include <string.h>
#include <ros/console.h>
#include <modules/sensor_msg.h>
#include <modules/joystick_msg.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include "libraries/json.hpp"

using json = nlohmann::json;
  
class DiffOdometry
{
    public:
	DiffOdometry(ros::NodeHandle);

    private:
	std::string node_name;
	ros::NodeHandle nh;
	ros::Subscriber subSensors, subControl;
	ros::Publisher pubOdometry;
	tf::TransformBroadcaster odom_broadcaster;
	ros::Time current_time, last_time;
	nav_msgs::Odometry odom;
	void SensorsCallback(const modules::sensor_msg &sensor_msg);
	void ControlCallback(const modules::joystick_msg &control_msg);
	struct position {
		double x{0};
		double y{0};
		double th{0};
	} delta, current, velocity, prev;
	double b{1.0}; //Distance between wheels
	double dt;
	geometry_msgs::Quaternion odom_quat;
	geometry_msgs::TransformStamped odom_trans;
	json position_points;
	std::string input{"sensors"};
};

DiffOdometry::DiffOdometry(ros::NodeHandle nh_priv)
{
    node_name = ros::this_node::getName	();
    int log_level;
    nh_priv.param("log_level", log_level,0);
    nh_priv.param("wheels_distance", b, 1.0);
    nh_priv.param("input", input, input);
    ros::console::levels::Level console_level;
    console_level = (ros::console::levels::Level)log_level;
    //~ ROS_INFO_STREAM(node_name << " --- Level: " << console_level);
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    if (input == "control")
	subControl = nh.subscribe("/control_topic",10,&DiffOdometry::ControlCallback, this);
    else 
	subSensors = nh.subscribe("/sensor_topic",10,&DiffOdometry::SensorsCallback, this);
    pubOdometry = nh.advertise<nav_msgs::Odometry>("odometry_topic", 1);
    tf::TransformBroadcaster odom_broadcaster;
    
    current_time = ros::Time::now();
	last_time = ros::Time::now();
}

void DiffOdometry::SensorsCallback(const modules::sensor_msg &sensor_msg)
{
    current_time = ros::Time::now();
    const double Vl = sensor_msg.currentSpeedLeft / 100.0;
    const double Vr = sensor_msg.currentSpeedRight / 100.0;
    dt = (current_time - last_time).toSec();
    
    velocity.th = (Vr - Vl) / b;
    delta.th = velocity.th * dt;
    current.th = delta.th + prev.th;
    
    velocity.x = cos(current.th) * (Vr + Vl) / 2 ;
    velocity.y = sin(current.th) * (Vr + Vl) / 2 ;
        
    delta.x = velocity.x * dt;
    delta.y = velocity.y * dt;
    
    ROS_DEBUG_STREAM ("Secs: " << current_time.sec << " nSecs: " << current_time.nsec );
    ROS_DEBUG_STREAM ("dt: " << dt );
    ROS_DEBUG_STREAM ("dth: " << delta.th );
    ROS_DEBUG_STREAM ("th: " << current.th );
    ROS_DEBUG_STREAM ("dX: " << delta.x );
    ROS_DEBUG_STREAM ("dY: " << delta.y );
	
	current.x = prev.x + delta.x;
	current.y = prev.y + delta.y;
    prev = current;
    odom_quat = tf::createQuaternionMsgFromYaw(current.th);
    
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = current.x;
    odom_trans.transform.translation.y = current.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    
    odom_broadcaster.sendTransform(odom_trans);

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = current.x;
    odom.pose.pose.position.y = current.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = velocity.x;
    odom.twist.twist.linear.y = velocity.y;
    odom.twist.twist.angular.z = velocity.th;
    
    pubOdometry.publish(odom);
    last_time = current_time;
    
}

void DiffOdometry::ControlCallback(const modules::joystick_msg &control_msg)
{
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    if (control_msg.DrivingMode == 1)
    {
	const double Vl = control_msg.LeftWheel / 100.0;
	const double Vr = control_msg.RightWheel / 100.0;
    
	velocity.th = (Vr - Vl) / b;
	delta.th = velocity.th * dt;
	current.th = delta.th + prev.th;
	
	velocity.x = cos(current.th) * (Vr + Vl) / 2 ;
	velocity.y = sin(current.th) * (Vr + Vl) / 2 ;

    }
    else if (control_msg.DrivingMode == 2)
    {
	velocity.th = -1 *control_msg.AngularRate * M_PI / 180.0;
	delta.th = velocity.th * dt;
	current.th = delta.th + prev.th;
	
	velocity.x = cos(current.th) * control_msg.Speed / 100.0;
	velocity.y = sin(current.th) * control_msg.Speed / 100.0;
    }   
    delta.x = velocity.x * dt;
    delta.y = velocity.y * dt;
    
    ROS_DEBUG_STREAM ("Secs: " << current_time.sec << " nSecs: " << current_time.nsec );
    ROS_DEBUG_STREAM ("dt: " << dt );
    ROS_DEBUG_STREAM ("dth: " << delta.th );
    ROS_DEBUG_STREAM ("th: " << current.th );
    ROS_DEBUG_STREAM ("dX: " << delta.x );
    ROS_DEBUG_STREAM ("dY: " << delta.y );
	
    current.x = prev.x + delta.x;
    current.y = prev.y + delta.y;
    prev = current;
    odom_quat = tf::createQuaternionMsgFromYaw(current.th);
    
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = current.x;
    odom_trans.transform.translation.y = current.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    
    odom_broadcaster.sendTransform(odom_trans);

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = current.x;
    odom.pose.pose.position.y = current.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = velocity.x;
    odom.twist.twist.linear.y = velocity.y;
    odom.twist.twist.angular.z = velocity.th;
    
    pubOdometry.publish(odom);
    last_time = current_time;
    
}



