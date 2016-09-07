#include "ardrone_test.h"

static geometry_msgs::Twist hover;



bool flag = 0;
keyboard_controller::keyboard_controller(ros::NodeHandle &node){
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

void keyboard_controller::keyPressEvent(QKeyEvent * key){
	switch(key->key()){
		case Qt::Key_Z:
	      pub_empty_takeoff.publish(emp_msg);
	      break;
	    case Qt::Key_X:
	      pub_empty_land.publish(emp_msg);
	      break;
	     default: pub_twist.publish(hover);
	}
}

void *keyboard_thread(void *arguments){
  	struct keyboard_struct *args = (struct keyboard_struct *)arguments;
  	args->arg1->resize(336, 227);
  	args->arg1->show();
  	args->arg1->setWindowTitle(QString::fromUtf8("Press Key with te windows selected"));
  	args->arg2->exec();
  	return NULL;
}

int keyboard_thread_init(keyboard_controller *control, QApplication *app){
	struct keyboard_struct *args = new keyboard_struct();
  	args->arg1 = control;
  	args->arg2 = app;
  	pthread_t thread_pointer;
  	if(pthread_create(&thread_pointer, NULL, keyboard_thread, args)) {
		return 1;
  	}
	
	
  	ros::spin();
  	cv::destroyWindow("view");
	
  	if(pthread_join(thread_pointer, NULL)) {
  	  	return 2;
  	}
}	