#include "ardrone_test.h"
#include "keyboard_controller.moc"

static geometry_msgs::Twist hover;

/**
 *	@brief Constructor
 *
 *	When a keyboard_controlled is constructed, it creates some ros::Publisher classes and
 *	set some messages.

 *	@param node Takes the pointer to a ros::NodeHandle class in which will be advertised 
 *	some topics ("/cmd_vel", "/ardrone/takeoff", "/ardrone/land" and "/ardrone/reset" topics).
*/
keyboard_controller::keyboard_controller(ros::NodeHandle node){
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


void keyboard_controller::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

	cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
  std_msgs::Header h = msg->header;
  std::stringstream ss;
  ss << "Time: " << h.stamp << " Frame: " << h.seq;
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

void keyboard_controller::navdataCallback(const ardrone_autonomy::Navdata& data){
	std::cout << "Battery: " << data.batteryPercent << std::endl;
	std::cout << "Rotacion Izquierta/derecha: " << data.rotX << std::endl;
	std::cout << "Rotacion adelante/atras: " << data.rotY << std::endl;
	std::cout << "Orientacion: " << data.rotZ << std::endl;
	std::cout << "Temperatura: " << data.temp << std::endl;
	std::cout << "Altura estimada: " << data.altd << std::endl;
	std::cout << "Velocidad lineal: vx: " << data.vx << "vy: " << data.ay << "vz: " << data.vz <<std::endl;
	std::cout << "Aceleracion lineal: ax: " << data.ax << "ay: " << data.ay << "az: " << data.az <<std::endl;
	std::cout << "Time stamp: " << data.tm << std::endl;
	std::cout << std::string(25, '\n');
}

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