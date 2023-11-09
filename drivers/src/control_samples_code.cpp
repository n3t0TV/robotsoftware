#include <ros/ros.h>
#include <ros/console.h>

#include "classes/ControlSamples.h"

int main(int argc, char **argv)
{
	ros::init(argc,argv,"control_samples_node");
	ROS_INFO_STREAM("control_samples_node");	
	ros::NodeHandle nh("~");
	ControlSamples controlSamples(nh);
	
	ros::Rate rate(10);	
		
	time_t start = time(0);	
	int duration;
	
	while(ros::ok())
	{
		time_t now = time(0);
		duration = now - start;
		ROS_DEBUG_STREAM(to_string(duration));
		if(duration < controlSamples.time)
		{
			if (duration > 1)
				controlSamples.go(controlSamples.angle);
			else
				controlSamples.go(0);
		}
		else 
		{
			ROS_DEBUG_STREAM("STOP!!!!!");
			controlSamples.stop();
		}
		rate.sleep();
		ros::spinOnce();
	}
	
	return 0;	
}
