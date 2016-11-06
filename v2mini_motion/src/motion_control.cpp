#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <stdlib.h>
#include <stdio.h>

#include "v2mini_motion/robot_controller.h"

int main(int argc, char ** argv) {

	ros::init(argc, argv, "motion_control");
	ros::NodeHandle n;

	ros::Publisher base_pub = n.advertise<geometry_msgs::Twist>("base_cmds", 1);

	// Get type of control from param (auto or remote)
	std::string control_type;
	ros::NodeHandle pnh("~");
	pnh.getParam("control", control_type);

	ROS_DEBUG_STREAM("control type:" << control_type);

	v2mini_motion::RobotController controller;

	ros::Rate loop_rate(50);
	int* key_cmds = NULL;

	while(ros::ok() && !controller.checkQuitStatus()) {

		geometry_msgs::Twist base_cmds;

		if (control_type == "remote") {

			key_cmds = controller.getKeyCmds();
			// controller.reRenderImage();

			base_cmds.linear.x = key_cmds[0];
			base_cmds.linear.y = key_cmds[1];
			base_cmds.angular.z = key_cmds[2];
			base_cmds.linear.z = key_cmds[3];

			//publish the movement commands
			base_pub.publish(base_cmds);
			loop_rate.sleep();

		} else {
			// todo --> autonomous
		}

	}

	return 0;
}
