#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>

#include "v2mini_teleop/teleop_controller.h"

using v2mini_teleop::ROBOT_VEL;

int setHeadPan(int curr_headpan, int increment)
{
	int deg60 = M_PI/3;
	int target_angle;

	if (abs(curr_headpan) <= deg60)
	{
		if (curr_headpan == deg60 && increment < 0)
		{
			target_angle = curr_headpan + increment;
		}
		else if (curr_headpan == -deg60 && increment > 0)
		{
			target_angle = curr_headpan + increment;
		}
		else if (abs(curr_headpan) != deg60)
		{
			target_angle = curr_headpan + increment;
		}
	}

	// limit headpan to +/- 60 deg
	if (target_angle > deg60)
	{
		target_angle = deg60;
	}
	else if (target_angle < -deg60)
	{
		target_angle = -deg60;
	}

	return target_angle;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "teleop");
	ros::NodeHandle node;

	ros::Publisher base_pub = node.advertise<geometry_msgs::Twist>("base_cmds", 1);
	ros::Publisher torso_pub = node.advertise<geometry_msgs::Twist>("torso_cmds", 1);
	ros::Publisher headpan_pub = node.advertise<std_msgs::Float64>("headpan_controller/command", 1);
	ros::Publisher joint_pub = node.advertise<sensor_msgs::JointState>("joint_states", 1);
	tf::TransformBroadcaster broadcaster;

	// todo -> publish joint positions for teleoperated parts
	// and use tf broadcaster to drive virtual teleop joints.
	geometry_msgs::TransformStamped odom_trans;
	sensor_msgs::JointState joint_state;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
	const double degree = M_PI/180;
	double angle=0;

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

	int headpan_angle = 0;

	float* cmds = NULL;
	bool quit = false;

	v2mini_teleop::TeleopController controller;

	SDL_Event event;

	ros::Rate loop_rate(50);

	while(ros::ok() && !controller.checkQuitStatus() && !quit)
	{

		geometry_msgs::Twist base_cmds;
		geometry_msgs::Twist torso_cmds;
		std_msgs::Float64 headpan_cmd;

		//update joint_state --> INCOMPLETE
		joint_state.header.stamp = ros::Time::now();
		joint_state.name.resize(2);
		joint_state.position.resize(2);
		joint_state.name[0] ="body_with_head";
		joint_state.position[0] = 0;
		joint_state.name[1] ="body_with_arm";
		joint_state.position[1] = 0;

		// update transform --> INCOMPLETE
		// (moving in a circle with radius=2)
		odom_trans.header.stamp = ros::Time::now();
		odom_trans.transform.translation.x = cos(angle)*2;
		odom_trans.transform.translation.y = sin(angle)*2;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

		angle += degree/4;

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

		base_cmds.linear.x = cmds[v2mini_teleop::BASE_VELX];
		base_cmds.linear.y = cmds[v2mini_teleop::BASE_VELY];
		base_cmds.angular.z = cmds[v2mini_teleop::BASE_VELZ];

		// todo -> replace Twist msg with custom torso msg (update subscribers too)
		torso_cmds.linear.x = cmds[v2mini_teleop::FACE_TOGGLE];
		torso_cmds.linear.y = cmds[v2mini_teleop::TORSO_VEL];
		torso_cmds.linear.z = cmds[v2mini_teleop::HEADTILT_VEL];

		headpan_angle = setHeadPan(headpan_angle, cmds[v2mini_teleop::HEADPAN_VEL]);
		headpan_cmd.data = headpan_angle;

		//publish the movement commands
		base_pub.publish(base_cmds);
		torso_pub.publish(torso_cmds);
		torso_pub.publish(headpan_cmd);
		joint_pub.publish(joint_state);
		broadcaster.sendTransform(odom_trans);

		loop_rate.sleep();
	}

	cmds = NULL;

	return 0;
}
