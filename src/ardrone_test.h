#ifndef ARDRONE_TEST_H
#define ARDRONE_TEST_H

/* Include Section */

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "ardrone_autonomy/Navdata.h"
#include "ardrone_autonomy/navdata_gps.h"
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
#include <chrono>
#include <QKeyEvent>
#include <QMainWindow>
#include <QMessageBox>
#include <QApplication>
#include <thread>
#include <mutex>
#include <queue>
#include <array>
#include "gps_ang_dist.h"

/*Definitions*/


#define UNKNOW 			0		
#define INITED			1
#define LANDED 			2
#define FLYING 			3
#define HOVERING		4
#define TEST 			5
#define TAKINGOFF		6
#define FLYING2			7
#define LOOPING			8
#define AUTO_PILOT_EN 	true
#define AUTO_PILOT_DIS 	false

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
		void navdata_gps_Callback(const ardrone_autonomy::navdata_gps& data);
		void Takeoff();
		void goToAltitude(double meters);
		void MoveForward();
		void Hover();
		void Rotate(double grades);
		void microAngleComp();
		//void set_dest_coordinates(double dest_latitude, double dest_longitude, double dest_elev);
		void gps_init();
		void gps_auto_pilot();
		void print_info(void);

	private:
		int state;
		float velocity;
		bool auto_pilot;
		double battery;
		double rotX;
		double rotY;
		double rotZ;
		int temp;
		int altd;
		double vx;
		double vy; 
		double vz;
		double ax; 
		double ay; 
		double az;
		double tm;
		double long0;
		double lat0;
		double elevation;
		double dest_long0;
		double dest_lat0;
		double dest_elevation;
		double dist_to_target;
		double comp_ang_to_target; 		//Relative to north
		double rel_comp_ang_to_target;	//Relative to initial position
		std::thread gps_thread;
		std::thread gps_thread2;
		geometry_msgs::Twist twist_msg;
		std_msgs::Empty emp_msg;
		ros::Publisher pub_empty_land;
		ros::Publisher pub_twist;
		ros::Publisher pub_empty_takeoff;
		ros::Publisher pub_empty_reset;
		void keyPressEvent(QKeyEvent *key);
		void keyReleaseEvent(QKeyEvent *key);
};

#endif
