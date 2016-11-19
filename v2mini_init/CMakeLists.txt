cmake_minimum_required(VERSION 2.8.3)
project(v2mini_init)

find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
  std_msgs
  geometry_msgs
)

catkin_package()

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware uno_headware ALL)
rosserial_add_client_target(firmware mega_baseware ALL)
rosserial_add_client_target(firmware mega_torsoware ALL)
rosserial_add_client_target(firmware uno_headware-upload)
rosserial_add_client_target(firmware mega_baseware-upload)
rosserial_add_client_target(firmware mega_torsoware-upload)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
