#include "ardrone_test.h"
#include "keyboard_controller.moc"

static geometry_msgs::Twist hover;



bool flag = 0;

/**
 *	@brief Constructor
 *
 *	When a keyboard_controlled is constructed, it creates some ros::Publisher classes and
 *	set some messages.

 *	@param node Takes the pointer to a ros::NodeHandle class in which will be advertised 
 *	some topics ("/cmd_vel", "/ardrone/takeoff", "/ardrone/land" and "/ardrone/reset" topics).
*/
keyboard_controller::keyboard_controller(ros::NodeHandle &node){
	status = LANDED;
	pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); 
	pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1);
	pub_empty_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1); 
	twist_msg.linear.x=0.0; 
	twist_msg.linear.y=0.0;
	twist_msg.linear.z=0.0;
	twist_msg.angular.x=0.0; 
	twist_msg.angular.y=0.0;
	twist_msg.angular.z=0.0;
	hover.linear.x=0.0; 
	hover.linear.y=0.0;
	hover.linear.z=0.0;
	hover.angular.x=0.0; 
	hover.angular.y=0.0;
	hover.angular.z=0.0;
}

/**
 *	@brief Destructor
 *
 *	Do nothing.
*/
keyboard_controller::~keyboard_controller(void){}

/**
 *	@brief Handle the Key Press Event
 *
 *	This function maps the key pressed and publish the corresponding message
 *
 *	@param key Take the pointer to QKeyEvent class
 *	@see http://doc.qt.io/qt-5/qkeyevent.html
 *	@todo Documment switch case
 *
*/
void keyboard_controller::keyPressEvent(QKeyEvent *key){
	switch(key->key()){
		case Qt::Key_Z:
	      	pub_empty_takeoff.publish(emp_msg);
	      	status = TAKINGOFF;
	      	break;
	    case Qt::Key_X:
	    	pub_empty_land.publish(emp_msg);
	      	status = LANDING;
	      	break;
	    default: pub_twist.publish(hover);
	}
}

/**
 *	@brief Handle the Key Release Event
 *
 *	@param key Take the pointer to QKeyEvent class
 *	@see http://doc.qt.io/qt-5/qkeyevent.html
 *	@todo Documment switch case
 *
*/
void keyboard_controller::keyReleaseEvent(QKeyEvent *key){
	switch(key->key()){
	    default: pub_twist.publish(hover);
	}
}