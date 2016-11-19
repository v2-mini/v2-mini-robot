cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

generate_arduino_firmware(uno_headware
  SRCS uno_headware.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD uno
  PORT /dev/arduino_uno
)

generate_arduino_firmware(mega_baseware
  SRCS mega_baseware.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD mega2560
  PORT /dev/arduino_mega_base
)

generate_arduino_firmware(mega_torsoware
  SRCS mega_torsoware.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD mega
  PORT /dev/arduino_mega_head
)
