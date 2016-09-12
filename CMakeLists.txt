cmake_minimum_required(VERSION 2.8.3)
project(ardrone_test)

find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  Qt5Widgets REQUIRED
  Qt5Core REQUIRED
)

ADD_DEFINITIONS(${QT_DEFINITIONS})
link_directories(/usr/include/qt4)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Widgets_LIBRARIES}
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
)



catkin_package()
add_executable(main src/main.cpp src/image_navdata_handler.cpp src/keyboard_controller.cpp /usr/include/qt4)
add_dependencies(main ardrone_test_generate_messages_cpp)
target_link_libraries(
  main ${catkin_LIBRARIES}
)
install(TARGETS main
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})