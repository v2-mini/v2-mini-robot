#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <stdlib.h>
#include <stdio.h>

#include "v2mini_motion/robot_controller.h"

enum ROBOT_VEL {BASE_VELX, BASE_VELY, BASE_VELZ, TORSO_VELZ}; // TODO CANT IMPORT from robot_contoller?

int main(int argc, char ** argv) {

	ros::init(argc, argv, "motion_control");
	ros::NodeHandle n;

	ros::Publisher base_pub = n.advertise<geometry_msgs::Twist>("base_cmds", 1);

	// Get type of control from param (auto or remote)
	std::string control_type;
	ros::NodeHandle pnh("~");
	pnh.getParam("control", control_type);

	ROS_DEBUG_STREAM("control type:" << control_type);

	float* gamepad_cmds = NULL;
	bool quit = false;

	v2mini_motion::RobotController controller;

	SDL_Event event;

	ros::Rate loop_rate(50);

	while(ros::ok() && !controller.checkQuitStatus() && !quit) {

		geometry_msgs::Twist base_cmds;

		if (control_type == "remote") {

			while(SDL_PollEvent(&event) != 0)
			{

				if (event.type == SDL_QUIT) {
					quit = true;
				}

				// TODO USE LOGIC AND PARAM TO SELECT BASE CONTROL TYPE (KEY OR GAMECONTROLLER)

				controller.getKeyCmds();
				gamepad_cmds = controller.getGamepadCmds();

			}

			base_cmds.linear.x = gamepad_cmds[BASE_VELX];
			base_cmds.linear.y = gamepad_cmds[BASE_VELY];
			base_cmds.angular.z = gamepad_cmds[BASE_VELZ];
			base_cmds.linear.z = gamepad_cmds[TORSO_VELZ];

			//publish the movement commands
			base_pub.publish(base_cmds);
			loop_rate.sleep();

		} else {
			// todo --> autonomous
		}

	}

	return 0;
}
