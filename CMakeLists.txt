cmake_minimum_required(VERSION 2.8.3)
project(ardrone_test)


catkin_package()
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
)

find_package(
  Qt5Widgets REQUIRED
  Qt5Core REQUIRED
)

include_directories(
  ${catkin_INCLUDE_DIRS}
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
)

add_executable(main src/main.cpp src/keyboard_controller.cpp src/gps_ang_dist.cpp)
add_definitions(-std=c++11)

target_link_libraries(
  main ${catkin_LIBRARIES}
)

target_link_libraries(main Qt5::Widgets)
install(TARGETS main
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
