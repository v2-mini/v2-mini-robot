cmake_minimum_required(VERSION 2.8.3)
project(v2mini_motion)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")

find_package(catkin REQUIRED COMPONENTS
	roscpp rospy std_msgs geometry_msgs genmsg  roslib)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(motion_control
	src/motion_control.cpp
	src/robot_controller.cpp)

target_link_libraries(motion_control ${catkin_LIBRARIES})
target_link_libraries(motion_control ${SDL2_LIBRARIES} SDL2)
target_link_libraries(motion_control ${SDL2_image_LIBRARIES} SDL2_image)

install(TARGETS motion_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
