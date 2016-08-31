#include <ros/ros.h>
#include "std_msgs/String.h"
#include "ardrone_autonomy/Navdata.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

	cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    std_msgs::Header h = msg->header;
    std::cout << h << std::endl;
  	try
  	{
  	  cv::imshow("view", img);
  	  cv::waitKey(30);
  	}
  	catch (cv_bridge::Exception& e)
  	{
  	  ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  	}
}


int main(int argc, char **argv)
{
	std::cout << "Starting ARDRONE node" << std::endl;
	ros::init(argc,argv, "main");
	ros::NodeHandle n;
  	cv::namedWindow("view");
  	cv::startWindowThread();
  	image_transport::ImageTransport it(n);
  	image_transport::Subscriber sub = it.subscribe("ardrone/image_raw", 1, imageCallback);
  	ros::spin();
  	cv::destroyWindow("view");
}