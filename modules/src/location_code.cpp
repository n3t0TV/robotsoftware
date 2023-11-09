#include <string.h>
 
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

using namespace std;

tf2::Quaternion qRot;
double angle;

ros::Subscriber subImu;

void imuCallback(const sensor_msgs::Imu& imuMsg)
{
    double roll, pitch, yaw;
	qRot = tf2::Quaternion(imuMsg.orientation.x,imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w);
    tf2::Matrix3x3(qRot).getRPY(roll, pitch, yaw);
    // ROS_DEBUG_STREAM("Yaw: " << yaw << " | Pitch: " << pitch << " | Roll: " << roll);
    angle = yaw;
    ROS_DEBUG_STREAM("angle: " << angle);
}

int main (int argc,char **argv)
{
    ros::init(argc,argv,"state_publisher");
    ros::NodeHandle nh;
    string node_name = ros::this_node::getName();
	ROS_DEBUG_STREAM(node_name);
    int log_level;
    nh.param("log_level", log_level,0);
    ros::console::levels::Level console_level;
    console_level = (ros::console::levels::Level)log_level;
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    //ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states",1);
    tf2_ros::TransformBroadcaster broadcaster;
    ros::Rate rate(3);

    /*q.w=1;
    q.x=0;
    q.y=0;
    q.z=0;*/

    subImu = nh.subscribe("sensor_topic/imu", 10, imuCallback);

    // double angle=0;
    // double posX=0;
    // double posY=0;

    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id="world";
    odom_trans.child_frame_id="base_link";
    
    while(ros::ok())
    {
        /* joint_state.header.stamp=ros::Time::now();
        joint_state.name.resize(1);
        joint_state.position.resize(1);
        joint_state.name[0]="base_link";
        joint_state.position[0]=0;*/

        //Rotation around up vector
        tf2::Quaternion qRot=tf2::Quaternion(tf2::Vector3(0,0,1),angle);
        odom_trans.header.stamp=ros::Time::now();

        //Translation x, y plane
        odom_trans.transform.translation.x=0;
        odom_trans.transform.translation.y=0;
        odom_trans.transform.translation.z=0;

        odom_trans.transform.rotation.w =(double) qRot.w();
        odom_trans.transform.rotation.x =(double) qRot.x();
        odom_trans.transform.rotation.y =(double) qRot.y();
        odom_trans.transform.rotation.z =(double) qRot.z();


        //joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        // angle+=0.01;
        //posX+=0.01;
        //posY+=0.01;
        
		rate.sleep();
        ros::spinOnce();
    }
}