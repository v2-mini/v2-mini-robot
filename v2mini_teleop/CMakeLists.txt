cmake_minimum_required(VERSION 2.8.3)
project(v2mini_teleop)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	sensor_msgs
	geometry_msgs
	genmsg
	roslib
	tf
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(teleop
	src/teleop.cpp
	src/teleop_controller.cpp)

target_link_libraries(teleop ${catkin_LIBRARIES})
target_link_libraries(teleop ${SDL2_LIBRARIES} SDL2)
target_link_libraries(teleop ${SDL2_image_LIBRARIES} SDL2_image)

install(TARGETS teleop
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
