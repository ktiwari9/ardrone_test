cmake_minimum_required(VERSION 2.8.3)
project(ardrone_test)

find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}

   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
)



catkin_package()
add_executable(main src/image_handler.cpp)
add_dependencies(main ardrone_test_generate_messages_cpp)
target_link_libraries(main
   ${catkin_LIBRARIES}
)
install(TARGETS main
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})