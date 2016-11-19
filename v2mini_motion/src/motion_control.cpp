#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <stdlib.h>
#include <stdio.h>

#include "v2mini_motion/robot_controller.h"

using v2mini_motion::ROBOT_VEL;

int main(int argc, char ** argv) {

	ros::init(argc, argv, "motion_control");
	ros::NodeHandle n;

	ros::Publisher base_pub = n.advertise<geometry_msgs::Twist>("base_cmds", 1);
	ros::Publisher torso_pub = n.advertise<geometry_msgs::Twist>("torso_cmds", 1);

	// Get type of control from param (auto or remote)
	std::string control_type;
	std::string controller_type;
	ros::NodeHandle pnh("~");
	pnh.getParam("control", control_type);

	// Use remote control as default (no arg).
	if (control_type == "")
	{
		control_type == "remote";
	}

	printf("Control Type: %s\n", control_type.c_str());

	if (control_type == "remote")
	{
		// Controlled by either keyboard or gamepad
		pnh.getParam("controller", controller_type);

		// Use keyboard controller as default (no arg).
		if (controller_type == "")
		{
			controller_type = "keyboard";
		}

		printf("Controller: %s\n", controller_type.c_str());
	}

	float* cmds = NULL;
	bool quit = false;

	v2mini_motion::RobotController controller;

	SDL_Event event;

	ros::Rate loop_rate(50);

	while(ros::ok() && !controller.checkQuitStatus() && !quit)
	{

		geometry_msgs::Twist base_cmds;
		geometry_msgs::Twist torso_cmds;

		if (control_type == "remote")
		{

			while(SDL_PollEvent(&event) != 0)
			{

				if (event.type == SDL_QUIT)
				{
					quit = true;
				}

				// getKeyCmds by default (also checks 'esc' button)
				cmds = controller.getKeyCmds();

				if (controller_type == "gamepad")
				{
					// overwrite the pointer if gamecontroller is used
					cmds = controller.getGamepadCmds();
				}

			}

			base_cmds.linear.x = cmds[v2mini_motion::BASE_VELX];
			base_cmds.linear.y = cmds[v2mini_motion::BASE_VELY];
			base_cmds.angular.z = cmds[v2mini_motion::BASE_VELZ];

			torso_cmds.linear.x = cmds[v2mini_motion::FACE];

			//publish the movement commands
			base_pub.publish(base_cmds);
			base_pub.publish(torso_cmds);
			loop_rate.sleep();

		}
		else
		{
			// todo --> autonomous
		}

	}

	cmds = NULL;

	return 0;
}
