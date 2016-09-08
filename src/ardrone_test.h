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
#include <pthread.h>

/*Class definitions */


/**
 *	@brief Class to handle the keyboard controller
 *	
 *	This class is derived from QWidget class, for further 
 *	reference go to http://doc.qt.io/qt-4.8/qwidget.html
*/
class keyboard_controller : public QWidget
{
	public:
		keyboard_controller(ros::NodeHandle &node);
		~keyboard_controller(void);
		void keyPressEvent(QKeyEvent* e);

	private:
		geometry_msgs::Twist twist_msg;
		std_msgs::Empty emp_msg;
		ros::Publisher pub_empty_land;
		ros::Publisher pub_twist;
		ros::Publisher pub_empty_takeoff;
		ros::Publisher pub_empty_reset;
};

/**
 *	@brief Structure to handle the KB controller and image display.
*/
struct keyboard_struct {
    keyboard_controller *arg1; /**< Pointer to keyboard_controller class */
    QApplication *arg2; /**< Pointer to QApplication class, for further reference go to http://doc.qt.io/qt-5/qapplication.html*/
};

/* Callback functions */
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void navdataCallback(const ardrone_autonomy::Navdata& data);
void *keyboard_thread(void *arguments);
int keyboard_thread_init(keyboard_controller *control, QApplication *app);



#endif

