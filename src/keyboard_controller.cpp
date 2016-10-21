#include "ardrone_test.h"
//#include "keyboard_controller.moc"

static geometry_msgs::Twist hover;

keyboard_controller::keyboard_controller(ros::NodeHandle node){
	state = 0;
	velocity = 0.1;
	auto_pilot = 0;
	battery = 0;
	rotX 	= 0;
	rotY	= 0;
	rotZ	= 0;
	temp 	= 0;
	altd 	= 0;
	vx 		= 0;
	vy 		= 0; 
	vz		= 0;
	ax 		= 0; 
	ay 		= 0; 
	az		= 0;
	tm 		= 0;
	long0	= 0;
	lat0	= 0;
	elevation 		=	0;
	dest_long0		= 	0;
	dest_lat0		= 	0;
	dest_elevation 	=	0;
	dist_to_target	=	0;
	comp_ang_to_target = 0;
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
	pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); 
	pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1);
	pub_empty_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1); 
}

/**
 *	@brief Destructor
 *
 *	Do nothing.
*/
keyboard_controller::~keyboard_controller(void){}

/**
 *	@brief GPS basic waypoint
 *
 *
*/

void keyboard_controller::set_dest_coordinates(double dest_latitude, double dest_longitude, double dest_elev){
	dest_long0			= dest_longitude;
	dest_lat0			= dest_latitude;
	dest_elevation		= dest_elev;	
	dist_to_target		= CoordinatesToMeters(lat0, long0, dest_lat0, dest_long0);
	comp_ang_to_target	= CoordinatesToAngle(lat0, long0, dest_lat0, dest_long0);
	//this->print_info();
}


void keyboard_controller::gps_init(void){
	std::thread gps_thread(&keyboard_controller::gps_auto_pilot, this);
	gps_thread.detach();
}

void keyboard_controller::gps_auto_pilot(void){
//	std::mutex mtx;

//	mtx.lock();
	this->set_dest_coordinates(-33.0347953, -71.5939068, 40);
	while(auto_pilot){
		this->print_info();
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
//	mtx.unlock();
}



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
	battery = 	data.batteryPercent;
	rotX 	= 	data.rotX;
	rotY	=	data.rotY;
	rotZ	=	data.rotZ;
	temp 	=	data.temp;
	altd 	=	data.altd;
	vx 		=	data.vx;
	vy 		= 	data.vy; 
	vz		=	data.vz;
	ax 		= 	data.ax; 
	ay 		=	data.ay; 
	az		=	data.az;
	tm 		=	data.tm;
	state 	=	data.state;
}

/**
 *	@brief Navigation Data Callback
 *
 *	This function takes the navigation data message and then diplay it.
 *
 *	@param data A message of the type adrone_autonomy::Navdata containing the data to display.
*/

void keyboard_controller::navdata_gps_Callback(const ardrone_autonomy::navdata_gps& data){
	long0 		= 	data.long0;
	lat0		=	data.lat0;
	elevation 	=	data.elevation;
}

/**
 *	@brief Handle the Key Press Event
 *
 *	This function maps the key pressed and publish the corresponding message
 *
 *	@param key Take the pointer to QKeyEvent class
 *	@see http://doc.qt.io/qt-5/qkeyevent.html
*/
void keyboard_controller::keyPressEvent(QKeyEvent *key){
	
	if(key->key() == Qt::Key_O){ //M de manual	
		/// - key Enter: Recover manual control
		auto_pilot = AUTO_PILOT_DIS;
	}

	else if(key->key() == Qt::Key_Space){			//Emergency
		/// - key Space : Emergency
		pub_empty_reset.publish(emp_msg);
	}

	else if(key->key() == Qt::Key_P){			
			/// - key P : Start autopilot
		(* this).gps_init();
		auto_pilot = AUTO_PILOT_EN;
	}	

	if(auto_pilot == AUTO_PILOT_DIS){
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
			if(state == FLYING || state == FLYING2 ||state == HOVERING){
				pub_empty_land.publish(emp_msg);
			}
			break;
		
		case Qt::Key_S:			//Move Backward
			/// - key S : Move Backward
			if(state == FLYING || state == FLYING2 || state == HOVERING){
				twist_msg.linear.x = -velocity;
	      		pub_twist.publish(twist_msg);
			}
			break;
		case Qt::Key_W:			//Move Forward
			/// - key W : Move Forward
			if(state == FLYING || state == FLYING2 || state == HOVERING){
				twist_msg.linear.x = velocity;
	      		pub_twist.publish(twist_msg);
			}
			break;
		case Qt::Key_D:			//Move Right
			/// - key D : Move Right
			if(state == FLYING || state == FLYING2 || state == HOVERING){
				twist_msg.linear.y = -velocity;
	      		pub_twist.publish(twist_msg);
			}
			break;
		case Qt::Key_A:			//Move Left
			/// - key A : Move Left
			if(state == FLYING || state == FLYING2 || state == HOVERING){
				twist_msg.linear.y = velocity;
	      		pub_twist.publish(twist_msg);
			}
			break;
		case Qt::Key_Q:			//Move Down
			/// - key Q : Move Down
			if(state == FLYING || state == FLYING2 || state == HOVERING){
				twist_msg.linear.z = -velocity;
	      		pub_twist.publish(twist_msg);
			}
			break;
		case Qt::Key_E:			//Move Up
			/// - key E : Move UP
			if(state == FLYING || state == FLYING2 || state == HOVERING){
				twist_msg.linear.z = velocity;
	      		pub_twist.publish(twist_msg);
			}
			break;
		case Qt::Key_F:			//Turn Right
			/// - key F : Turn Right
			if(state == FLYING || state == FLYING2 || state == HOVERING){
				twist_msg.angular.z = -velocity;
	      		pub_twist.publish(twist_msg);
			}
			break;
		case Qt::Key_R:			//Turn Left
			/// - key R : Turn Left 
			if(state == FLYING || state == FLYING2 || state == HOVERING){
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
			if(velocity >= 0.2){
				velocity -= 0.1;
			}
			break;	
	}
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
	if(state == FLYING || state == FLYING2 || state == HOVERING){
		twist_msg.linear.x=0.0; 
		twist_msg.linear.y=0.0;
		twist_msg.linear.z=0.0;
		twist_msg.angular.x=0.0; 
		twist_msg.angular.y=0.0;
		twist_msg.angular.z=0.0;
		pub_twist.publish(hover);
	}
}

void keyboard_controller::print_info(void){
	std::cout << "State: " << state << std::endl;
	std::cout << "AutoPilot: " << auto_pilot << std::endl;
	std::cout << "Battery: " << battery << std::endl;
	std::cout << "Velocity: " << velocity << std::endl;
	std::cout << "Roll Angle(X): " << rotX << std::endl;
	std::cout << "Pitch Angle(Y): " << rotY << std::endl;
	std::cout << "Yaw Angle(Z): " << rotZ << std::endl;
	std::cout << "Stimated altitude: " << altd << std::endl;
	std::cout << "Linear velocity: \n\tvx: " << vx << " \n\tvy: " << ay << " \n\tvz: " << vz << std::endl;
	std::cout << "Linear Acceleration: \n\tax: " << ax << " \n\tay: " << ay << " \n\taz: " << az << std::endl;
	std::cout << "Time stamp: " << tm << std::endl;
	std::cout << "Longitude: " << long0 << std::endl;
	std::cout << "Latitude: " << lat0 << std::endl;
	std::cout << "Elevation: " << elevation << std::endl;
	std::cout << "Target Longitude: " << dest_long0 << std::endl;
	std::cout << "Target Latitude: " << dest_lat0 << std::endl;
	std::cout << "Target Elevation: " << dest_elevation << std::endl;	
	std::cout << "Distance to target: " << dist_to_target << std::endl;
	std::cout << "Compensation Angle to target: " << comp_ang_to_target << std::endl;
	std::cout << std::string(14, '\n');
}