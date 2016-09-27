#include "ardrone_test.h"   
int main(int argc, char **argv)
{
	std::cout << "Starting ARDRONE TEST node" << std::endl;

	ros::init(argc,argv, "main");
  QApplication app(argc,argv);
	ros::NodeHandle n;
  keyboard_controller control(n);
  
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("ardrone/image_raw",1, &keyboard_controller::imageCallback, &control);
  ros::Subscriber navdata = n.subscribe("ardrone/navdata", 1000, &keyboard_controller::navdataCallback, &control);

  control.show();
  
  ros::spin();

  return app.exec();
}
