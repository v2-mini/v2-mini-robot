#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>

#include "v2mini_telop/user_inputs.h"

using v2mini_telop::ROBOT_VEL;

int main(int argc, char ** argv) {

	ros::init(argc, argv, "telop");
	ros::NodeHandle n;

	ros::Publisher base_pub = n.advertise<geometry_msgs::Twist>("base_cmds", 1);
	ros::Publisher torso_pub = n.advertise<geometry_msgs::Twist>("torso_cmds", 1);

	// Get type of control from param (auto or remote)
	std::string controller_type;
	ros::NodeHandle pnh("~");

	// Controlled by either keyboard or gamepad
	pnh.getParam("controller", controller_type);

	// Use keyboard controller as default (no arg).
	if (controller_type == "")
	{
		controller_type = "keyboard";
	}

	printf("Controller: %s\n", controller_type.c_str());

	float* cmds = NULL;
	bool quit = false;

	v2mini_telop::UserInputs controller;

	SDL_Event event;

	ros::Rate loop_rate(50);

	while(ros::ok() && !controller.checkQuitStatus() && !quit)
	{

		geometry_msgs::Twist base_cmds;
		geometry_msgs::Twist torso_cmds;

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

		base_cmds.linear.x = cmds[v2mini_telop::BASE_VELX];
		base_cmds.linear.y = cmds[v2mini_telop::BASE_VELY];
		base_cmds.angular.z = cmds[v2mini_telop::BASE_VELZ];

		torso_cmds.linear.x = cmds[v2mini_telop::FACE];
		torso_cmds.linear.y = cmds[v2mini_telop::TORSO_VEL];

		//publish the movement commands
		base_pub.publish(base_cmds);
		torso_pub.publish(torso_cmds);

		loop_rate.sleep();
	}

	cmds = NULL;

	return 0;
}
