cmake_minimum_required(VERSION 2.8.3)
project(v2mini_description)

find_package(catkin REQUIRED roslaunch)
catkin_package()

roslaunch_add_file_check(launch)

install(DIRECTORY config launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
