cmake_minimum_required(VERSION 2.8.3)
project(v2mini_controllers)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_controllers
  roscpp
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/dynamixel_joint_state_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${catkin_NODES_DIRS}
)
