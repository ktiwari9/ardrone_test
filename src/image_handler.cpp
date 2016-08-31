#include <ros/ros.h>
#include "std_msgs/String.h"
#include "ardrone_autonomy/Navdata.h"
#include <image_transport/image_transport.h>
#include "opencv2/core/cvstd.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <ctime>
#include <sstream>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

	cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
   	std_msgs::Header h = msg->header;

std::stringstream ss;
ss << h.stamp;
std::string ts = ss.str();
    cv::putText(img, ts, cv::Point(10,10), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(94.0, 206.0, 165.0, 0.0))	;
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

void navdataCallback(const ardrone_autonomy::Navdata& data){
	std::cout << "Battery: " << data.batteryPercent << std::endl;
	std::cout << "Rotacion Izquierta/derecha: " << data.rotX << std::endl;
	std::cout << "Rotacion adelante/atras: " << data.rotY << std::endl;
	std::cout << "Orientacion: " << data.rotZ << std::endl;
	std::cout << "Temperatura: " << data.temp << std::endl;
	std::cout << "Altura estimada: " << data.altd << std::endl;
	std::cout << "Velocidad lineal: vx: " << data.vx << "vy: " << data.ay << "vz: " << data.vz <<std::endl;
	std::cout << "Aceleracion lineal: ax: " << data.ax << "ay: " << data.ay << "az: " << data.az <<std::endl;
	std::cout << "Time stamp: " << data.tm << std::endl;

	std::cout << std::string(30, '\n');
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
  	ros::Subscriber nav = n.subscribe("ardrone/navdata", 1, navdataCallback);
  	ros::spin();
  	cv::destroyWindow("view");
}