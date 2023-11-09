#include "classes/imu/bno055/bno055.h"

int main(int argc, char **argv)
{
	ros::init(argc,argv,"imu_i2c_driver");
	ROS_INFO_STREAM("imu_i2c_driver");
	ros::NodeHandle nh("~");
	ros::Rate rate(3);

	BNO055_I2C bno055(nh);
	
	if(!bno055.begin())
	{
		ROS_ERROR("BNO055 initialization failed");
		return 0;
	}
	//~ bno055.setAxisRemap(AXIS_REMAP_Y,  AXIS_REMAP_X, AXIS_REMAP_Z, AXIS_REMAP_NEGATIVE, AXIS_REMAP_POSITIVE, AXIS_REMAP_POSITIVE);  
	while(ros::ok())
	{
		float yaw, pitch, roll;
		bno055.publishMessage();
		ros::spinOnce();		
		rate.sleep();
	}
	return 0;
}
