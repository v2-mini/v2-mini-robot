cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

generate_arduino_firmware(unoware
  SRCS unoware.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD uno
  PORT /dev/arduino_uno
)

generate_arduino_firmware(megaware
  SRCS megaware.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD mega2560
  PORT /dev/arduino_mega
)
