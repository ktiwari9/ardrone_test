#include <ros/ros.h>
#include "std_msgs/String.h"
#include "ardrone_autonomy/Navdata.h"
#include <iostream>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

int main(int argc, char **argv)
{
	std::cout << "Starting ARDRONE node" << std::endl;
	ros::init(argc,argv, "main");
	ros::NodeHandle n;
	ros::spin();
	return 0;
}