cmake_minimum_required(VERSION 2.8.3)
project(v2mini_motion)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(motion_control src/motion_control.cpp)
target_link_libraries(motion_control ${catkin_LIBRARIES})
add_dependencies(motion_control v2mini_motion_generate_messages_cpp)
