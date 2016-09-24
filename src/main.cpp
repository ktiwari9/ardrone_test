#include "ardrone_test.h"
int main(int argc, char **argv)
{
	std::cout << "Starting ARDRONE TEST node" << std::endl;

	ros::init(argc,argv, "main");
	ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("ardrone/image_raw", 1, imageCallback);
  ros::Subscriber nav = n.subscribe("ardrone/navdata", 1, navdataCallback);

  QApplication app(argc, argv);

  keyboard_controller *control = new keyboard_controller(n);
  keyboard_struct *args = new keyboard_struct(); 
  args->arg1 = control;
  args->argc = argc;
  args->argv = argv;
  keyboard_thread_init(args);
  
  ros::spin();

  return 0;
}
