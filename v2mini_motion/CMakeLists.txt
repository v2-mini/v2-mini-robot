cmake_minimum_required(VERSION 2.8.3)
project(v2mini_motion)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs genmsg)
find_package(PkgConfig REQUIRED)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

pkg_check_modules(ncurses++ REQUIRED ncurses++)
add_executable(motion_control src/motion_control.cpp)
target_link_libraries(motion_control ${catkin_LIBRARIES})
target_link_libraries(motion_control ${catkin_LIBRARIES} ncurses)
target_link_libraries(motion_control ${ncurses++_LIBRARIES})
add_dependencies(motion_control v2mini_motion_generate_messages_cpp)
add_dependencies(motion_control ${catkin_EXPORTED_TARGETS} motion_control)
