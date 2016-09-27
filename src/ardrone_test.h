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
#include <sstream>
#include <QKeyEvent>
#include <QMainWindow>
#include <QMessageBox>
#include <QApplication>

/*Definitions*/


#define UNKNOW 		0
#define INITED		1
#define LANDED 		2
#define FLYING 		3
#define HOVERING	4
#define TEST 		5
#define TAKINGOFF	6
#define FLYING2		7
#define LOOPING		8

/*Class definitions */
/**
 *	@brief Class to handle the keyboard controller
 *	
 *	This class is derived from QWidget class, for further 
 *	reference go to http://doc.qt.io/qt-4.8/qwidget.html
*/

class keyboard_controller : public QWidget
{	//Q_OBJECT
	public:
		keyboard_controller(ros::NodeHandle node);
		~keyboard_controller(void);
		void imageCallback(const sensor_msgs::ImageConstPtr& msg);
		void navdataCallback(const ardrone_autonomy::Navdata& data);

	private:
		int state;
		float velocity;
		geometry_msgs::Twist twist_msg;
		std_msgs::Empty emp_msg;
		ros::Publisher pub_empty_land;
		ros::Publisher pub_twist;
		ros::Publisher pub_empty_takeoff;
		ros::Publisher pub_empty_reset;
		void keyPressEvent(QKeyEvent* e);
		void keyReleaseEvent(QKeyEvent *key);
};

#endif
