#ifndef ARDRONE_TEST_H
#define ARDRONE_TEST_H

/* Include Section */
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "ardrone_autonomy/Navdata.h"
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include "opencv2/core/cvstd.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <ctime>
#include <sstream>
#include <QKeyEvent>
#include <QMainWindow>
#include <QMessageBox>
#include <QApplication>
#include <QDebug>

/* Callback functions*/
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void navdataCallback(const ardrone_autonomy::Navdata& data);

/*Class definitions*/

class keyboard_controller : public QWidget
{
	public:
		keyboard_controller(ros::NodeHandle &node);
		~keyboard_controller() {};
		void keyPressEvent(QKeyEvent* e);

	private:
		geometry_msgs::Twist twist_msg;
		std_msgs::Empty emp_msg;
		ros::Publisher pub_empty_land;
		ros::Publisher pub_twist;
		ros::Publisher pub_empty_takeoff;
		ros::Publisher pub_empty_reset;
};


#endif

