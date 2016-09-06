#include "ardrone_test.h"

bool flag = 0;
keyboard_controller::keyboard_controller(ros::NodeHandle &node){
	pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 
	pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); 
	pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1);
	pub_empty_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1); 
}

void keyboard_controller::keyPressEvent(QKeyEvent * key){
	      pub_twist.publish(twist_msg);
}