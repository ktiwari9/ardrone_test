#include "ardrone_test.h"
#include "keyboard_controller.moc"

static geometry_msgs::Twist hover;

/**
 *	@brief Constructor
 *
 *	When a keyboard_controller is constructed, it creates some ros::Publisher classes and
 *	set some messages.

 *	@param node Takes the pointer to a ros::NodeHandle class in which will be advertised 
 *	some topics ("/cmd_vel", "/ardrone/takeoff", "/ardrone/land" and "/ardrone/reset" topics).
*/
keyboard_controller::keyboard_controller(ros::NodeHandle node){
	state = 0;
	velocity = 0.1;
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
 *	@brief Image Callback
 *
 *	This function convert the image to a cv::Mat objetct and then display it.
 *	
 *	@param msg A message of the type sensor_msgs::ImageConstPtr containing the image to display.
 *	@see http://docs.opencv.org/2.4/modules/core/doc/basic_structures.html#mat
*/
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

/**
 *	@brief Navigation Data Callback
 *
 *	This function takes the navigation data message and then diplay it.
 *
 *	@param data A message of the type adrone_autonomy::Navdata containing the data to display.
*/
void keyboard_controller::navdataCallback(const ardrone_autonomy::Navdata& data){
	std::cout << "State: " << data.state << std::endl;
	std::cout << "Battery: " << data.batteryPercent << std::endl;
	std::cout << "Velocity: " << velocity << std::endl;
	std::cout << "Rotacion Izquierta/derecha: " << data.rotX << std::endl;
	std::cout << "Rotacion adelante/atras: " << data.rotY << std::endl;
	std::cout << "Orientacion: " << data.rotZ << std::endl;
	std::cout << "Temperatura: " << data.temp << std::endl;
	std::cout << "Altura estimada: " << data.altd << std::endl;
	std::cout << "Velocidad lineal: vx: " << data.vx << "vy: " << data.ay << "vz: " << data.vz <<std::endl;
	std::cout << "Aceleracion lineal: ax: " << data.ax << "ay: " << data.ay << "az: " << data.az <<std::endl;
	std::cout << "Time stamp: " << data.tm << std::endl;
	std::cout << std::string(25, '\n');
	state = data.state;
}

/* NOT TESTED
void keyboard_controller::navdataGPSCallback(const ardrone_autonomy::Navdata_gps& data){
	std::cout << "Longitude: " << data->longitude << std::endl;
	std::cout << "Latitude: " << data->latitude << std::endl;
	std::cout << "Elevation: " << data->elevation << std::endl;

	std::cout << std::string(25, '\n');
}
*/

/**
 *	@brief Handle the Key Press Event
 *
 *	This function maps the key pressed and publish the corresponding message
 *
 *	@param key Take the pointer to QKeyEvent class
 *	@see http://doc.qt.io/qt-5/qkeyevent.html
*/
void keyboard_controller::keyPressEvent(QKeyEvent *key){
	/// Switch Case key
	switch(key->key()){
		case Qt::Key_Z:			//Take off
			/// - key Z : Take off \n
			if(state == LANDED){
				pub_empty_takeoff.publish(emp_msg);
			}
			break;
		case Qt::Key_X:			//Land
	    	/// - key X : Land \n
			if(state == HOVERING){
				pub_empty_land.publish(emp_msg);
			}
			break;
		case Qt::Key_C:			//Emergency
			/// - key C : Emergency
			pub_empty_reset.publish(emp_msg);
			break;
		case Qt::Key_S:			//Move Backward
			/// - key S : Move Backward
			if(state == FLYING || FLYING2 || HOVERING){
				twist_msg.linear.x = -velocity;
	      		pub_twist.publish(twist_msg);
			}
			break;
		case Qt::Key_W:			//Move Forward
			/// - key W : Move Forward
			if(state == FLYING || FLYING2 || HOVERING){
				twist_msg.linear.x = velocity;
	      		pub_twist.publish(twist_msg);
			}
			break;
		case Qt::Key_D:			//Move Right
			/// - key D : Move Right
			if(state == FLYING || FLYING2 || HOVERING){
				twist_msg.linear.y = -velocity;
	      		pub_twist.publish(twist_msg);
			}
			break;
		case Qt::Key_A:			//Move Left
			/// - key A : Move Left
			if(state == FLYING || FLYING2 || HOVERING){
				twist_msg.linear.y = velocity;
	      		pub_twist.publish(twist_msg);
			}
			break;
		case Qt::Key_Q:			//Move Down
			/// - key Q : Move Down
			if(state == FLYING || FLYING2 || HOVERING){
				twist_msg.linear.z = -velocity;
	      		pub_twist.publish(twist_msg);
			}
			break;
		case Qt::Key_E:			//Move Up
			/// - key E : Move UP
			if(state == FLYING || FLYING2 || HOVERING){
				twist_msg.linear.z = velocity;
	      		pub_twist.publish(twist_msg);
			}
			break;
		case Qt::Key_F:			//Turn Right
			/// - key F : Turn Right
			if(state == FLYING || FLYING2 || HOVERING){
				twist_msg.angular.z = -velocity;
	      		pub_twist.publish(twist_msg);
			}
			break;
		case Qt::Key_R:			//Turn Left
			/// - key R : Turn Left 
			if(state == FLYING || FLYING2 || HOVERING){
				twist_msg.angular.z = velocity;
	      		pub_twist.publish(twist_msg);
			}
			break;
		case Qt::Key_Plus:			//Increase Velocity
			/// - key + : Increase Velocity
			if(velocity < 1){
				velocity += 0.1;
			}
			break;
		case Qt::Key_Minus:			//Decrease Velocity
			/// - key - : Decrease Velocity 
			if(velocity > 0.1){
				velocity -= 0.1;
			}
			break;	
	}
}

/**
 *	@brief Handle the Key Release Event
 *
 *	This function publish a hover message when a key is released.
 *
 *	@param key Take the pointer to QKeyEvent class
 *	@see http://doc.qt.io/qt-5/qkeyevent.html
*/	
void keyboard_controller::keyReleaseEvent(QKeyEvent *key){
	if(state == FLYING || FLYING2 || HOVERING){
		twist_msg.linear.x=0.0; 
		twist_msg.linear.y=0.0;
		twist_msg.linear.z=0.0;
		twist_msg.angular.x=0.0; 
		twist_msg.angular.y=0.0;
		twist_msg.angular.z=0.0;
		pub_twist.publish(hover);
	}
}