#include "ardrone_test.h"

extern int flag;
int main(int argc, char **argv)
{
	std::cout << "Starting ARDRONE node" << std::endl;
	ros::init(argc,argv, "main");
	ros::NodeHandle n;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("ardrone/image_raw", 1, imageCallback);
  ros::Subscriber nav = n.subscribe("ardrone/navdata", 1, navdataCallback);
  

  QApplication app(argc, argv);
  keyboard_controller *test = new keyboard_controller(n);
  test->setWindowTitle(QString::fromUtf8("Press Key with window on"));
  test->resize(336, 227);
  test->show();
  app.exec();


  ros::spin();
  cv::destroyWindow("view");
  

}