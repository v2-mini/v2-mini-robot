#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>

#include "v2mini_teleop/teleop_controller.h"

using v2mini_teleop::ROBOT_VEL;

const int TOTAL_ARM_JOINTS = 5;

double incrementPosition(double current_position, double increment, double neg_lim, double pos_lim)
/*
 * Args:
 * 	current_position: current position.
 * 	increment: increment position value by this much.
 * 	neg_lim: limit position (more negative than pos_lim).
 * 	pos_lim: limit position (more positive than neg_lim).
 *
 * Returns:
 * 	target_position: incremeted position within limit range.
 */
{
	double target_position;

	// check that the current position is within limitted range
	if (current_position <= pos_lim && current_position >= neg_lim)
	{
		// only allow decrement if at pos limit
		if (current_position == pos_lim && increment < 0)
		{
			target_position = current_position + increment;
		}
		// only allow increment if at neg limit
		else if (current_position == neg_lim && increment > 0)
		{
			target_position = current_position + increment;
		}
		// if not at limit, increment up or down
		else if (current_position != pos_lim && current_position != neg_lim)
		{
			target_position = current_position + increment;
		}
		// don't increment if attemping to escape limits
		else
		{
			target_position = current_position;
		}

		// ensure target is within range after increment
		if (target_position > pos_lim)
		{
			target_position = pos_lim;
		}
		else if (target_position < neg_lim)
		{
			target_position = neg_lim;
		}

	}
	// if outside the limits, don't increment
	else
		target_position = current_position;

	return target_position;
}

int toggleArmJoint(int current_joint, int TOTAL_ARM_JOINTS, bool reverse_toggle)
{
	int next_joint;
	int joint_indices = TOTAL_ARM_JOINTS - 1;

	if (current_joint >= joint_indices && !reverse_toggle)
	{
		next_joint = 0;
	}
	else if (current_joint <= 0 && reverse_toggle)
	{
		next_joint = joint_indices;
	}
	else if (current_joint >= joint_indices && reverse_toggle)
	{
		next_joint = current_joint - 1;
	}
	else if (!reverse_toggle)
	{
		next_joint = current_joint + 1;
	}
	else
	{
		next_joint = current_joint - 1;
	}

	return next_joint;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "teleop");
	ros::NodeHandle node;

	tf::TransformBroadcaster broadcaster;

	ros::Publisher base_pub = node.advertise<geometry_msgs::Twist>("base_cmds", 1);
	ros::Publisher torso_pub = node.advertise<geometry_msgs::Twist>("torso_cmds", 1);
	ros::Publisher joint_pub = node.advertise<sensor_msgs::JointState>("joint_states", 1);
	ros::Publisher routine_pub = node.advertise<std_msgs::String>("routines", 1);
	ros::Publisher arm_pubs[] = {
			node.advertise<std_msgs::Float64>("arm_joint1_controller/command", 1),
			node.advertise<std_msgs::Float64>("arm_joint2_controller/command", 1),
			node.advertise<std_msgs::Float64>("arm_joint3_controller/command", 1),
			node.advertise<std_msgs::Float64>("arm_joint4_controller/command", 1),
			node.advertise<std_msgs::Float64>("arm_joint5_controller/command", 1),
	};

	// todo -> publish joint positions for teleoperated parts
	// and use tf broadcaster to update state of urdf.
	geometry_msgs::TransformStamped odom_trans;
	sensor_msgs::JointState joint_state;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
	const double radian = M_PI/180;
	double angle;

	int headpan_pos = 511;
	int headpan_max = 665;
	int headpan_min = 357;
	int current_joint = 0;
	double arm_joint_positions[TOTAL_ARM_JOINTS] = {0.11, 1.3, -0.12, -0.25, 0.16};
	double arm_joint_lims[TOTAL_ARM_JOINTS][2] = {
			{-0.2, 2.85},
			{-0.3, 1.6},
			{-3, 3},
			{-1.5, 1.5},
			{-3, 3}};

	// Get type of control from param (auto or remote)
	std::string controller_type;
	std::string arm_control;
	ros::NodeHandle pnh("~");

	// Controlled by either keyboard or gamepad
	pnh.getParam("controller", controller_type);

	// temperary demo script option
	pnh.getParam("arm_control", arm_control);

	// Use keyboard controller as default (no arg).
	if (controller_type == "")
	{
		controller_type = "keyboard";
	}

	printf("Controller: %s\n", controller_type.c_str());
	printf ("Scripted Arm: %s \n", arm_control.c_str());

	float* cmds = NULL;
	bool quit = false;

	v2mini_teleop::TeleopController controller;

	SDL_Event event;

	ros::Rate loop_rate(50);

	while(ros::ok() && !controller.checkQuitStatus() && !quit)
	{
		geometry_msgs::Twist base_cmds;
		geometry_msgs::Twist torso_cmds;
		std_msgs::String routine_cmds;
		std_msgs::Float64 arm_cmds[TOTAL_ARM_JOINTS];

		//update joint_state --> INCOMPLETE
		joint_state.header.stamp = ros::Time::now();
		joint_state.name.resize(2);
		joint_state.position.resize(2);
		joint_state.name[0] ="neckpan";
		joint_state.position[0] = 0;
		joint_state.name[1] ="shoulder";
		joint_state.position[1] = 0;

		// update transform --> INCOMPLETE
		// (moving in a circle with radius=2)
		odom_trans.header.stamp = ros::Time::now();
		odom_trans.transform.translation.x = cos(angle)*2;
		odom_trans.transform.translation.y = sin(angle)*2;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

		angle += radian / 4;

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

		// temporary workaround for ros controller not supporting XL-320 is to use arduino ...
		headpan_pos = incrementPosition(headpan_pos, cmds[v2mini_teleop::HEADPAN_VEL], headpan_min, headpan_max);

		torso_cmds.angular.x = headpan_pos;
		torso_cmds.linear.x = cmds[v2mini_teleop::FACE_TOGGLE];
		torso_cmds.linear.y = cmds[v2mini_teleop::TORSO_VEL];
		torso_cmds.linear.z = cmds[v2mini_teleop::HEADTILT_VEL];
		torso_cmds.angular.y = cmds[v2mini_teleop::WRIST_VEL];
		torso_cmds.angular.z = cmds[v2mini_teleop::GRIPPER_VEL];

		// control arm w/ script
		if (arm_control == "script")
		{
			std::string routine_type = controller.get_routine();

			if (routine_type != "none")
			{
				routine_cmds.data = routine_type;
				routine_pub.publish(routine_cmds);
				controller.set_routine("none");
			}


		}
		// teleop arm
		else
		{
			// toggle controlled joint on button press
			if (controller.armToggled() == 1)
			{
				current_joint = toggleArmJoint(current_joint, TOTAL_ARM_JOINTS, false);
				controller.resetArmToggle();
			}
			else if (controller.armToggled() == -1)
			{
				current_joint = toggleArmJoint(current_joint, TOTAL_ARM_JOINTS, true);
				controller.resetArmToggle();
			}

			// set the target value for each joint dynamixel and publish
			for (int joint = 0; joint < TOTAL_ARM_JOINTS; joint++)
			{
				if (current_joint == joint)
				{
					arm_joint_positions[joint] = incrementPosition(
							arm_joint_positions[joint], cmds[v2mini_teleop::ARM_JOINT_VEL],
							arm_joint_lims[joint][0], arm_joint_lims[joint][1]);

					arm_cmds[joint].data = arm_joint_positions[joint];

				}
				else
				{
					arm_cmds[joint].data = arm_joint_positions[joint];
				}

				arm_pubs[joint].publish(arm_cmds[joint]);
			}
		}

		// publish topics
		base_pub.publish(base_cmds);
		torso_pub.publish(torso_cmds);
		joint_pub.publish(joint_state);
		broadcaster.sendTransform(odom_trans);

		loop_rate.sleep();
	}

	cmds = NULL;

	return 0;
}
