# V2-Mini

## ROS packages for the V2-Mini Robot.


## STOMP MOVEIT README

Note you also need to git clone https://github.com/ros-industrial/industrial_moveit 

to your workspace, and compile it before this will work.


To run:

roslaunch v2mini_moveit_config demo.launch


In Rviz turn on:

Allow Ik approximate
Show robot goal state
set planner to "stomp" 
