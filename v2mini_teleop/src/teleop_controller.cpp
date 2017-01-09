#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <ros/package.h>
#include <stdexcept>
#include <stdio.h>
#include <cmath>

#include "v2mini_teleop/teleop_controller.h"

namespace v2mini_teleop
{

const Uint8 *keys = SDL_GetKeyboardState(NULL);

TeleopController::TeleopController()
{

	// Initialize SDL
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) == 0)
	{
		//Set texture filtering to linear
		SDL_SetHint( SDL_HINT_RENDER_SCALE_QUALITY, "1");

		window = SDL_CreateWindow("V2Mini Controller", SDL_WINDOWPOS_UNDEFINED,
				SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);

		if (window == NULL)
		{
			printf("SDL failed to create window: %s\n", SDL_GetError());
		}
		else
		{
			// Setup SDL window and load robot icon
			renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

			if (renderer == NULL)
			{
				printf("SDL failed to render: %s\n", SDL_GetError());
			}

			SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, 0xFF);

			// get the path to the src package dir
			std::string ros_path = ros::package::getPath("v2mini_teleop") + "/robo_icon.jpg";

			IMG_Init(IMG_INIT_PNG);
			SDL_Surface* loadedSurface = IMG_Load(ros_path.c_str());
			texture = SDL_CreateTextureFromSurface(renderer, loadedSurface);
			SDL_FreeSurface(loadedSurface);

			reRenderImage();
		}
	}
	else
	{
		printf("SDL failed to initialize: %s\n", SDL_GetError());
	}
}

TeleopController::~TeleopController()
{
	//Deallocate SDL resources
	SDL_DestroyTexture(texture);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);

	if (controller != NULL)
	{
		SDL_GameControllerClose(controller);
		controller = NULL;
	}

	window = NULL;
	renderer = NULL;
	texture = NULL;

	IMG_Quit();
	SDL_Quit();
}

void TeleopController::loadGamepad()
{
	controller = SDL_GameControllerOpen(0);

	if (controller == NULL)
	{
		printf("No Game Controller detected: %s\n", SDL_GetError());
	}
}

float* TeleopController::getKeyCmds()
{

	static float key_cmds[] = {0, 0, 0, 0, 0};
	const int max_value = 1000;

	if(keys[SDL_SCANCODE_ESCAPE])
	{
		quit = true;
	}

	// Base Left & Right
	if (keys[SDL_SCANCODE_LEFT] != keys[SDL_SCANCODE_RIGHT])
	{
		if (keys[SDL_SCANCODE_RIGHT] == 1) {
			key_cmds[BASE_VELX] = max_value;
		}
		else
		{
			key_cmds[BASE_VELX] = -max_value;
		}
	}
	else
	{
		key_cmds[BASE_VELX] = 0;
	}

	// Base Forward & Back
	if (keys[SDL_SCANCODE_UP] != keys[SDL_SCANCODE_DOWN])
	{
		if (keys[SDL_SCANCODE_UP] == 1)
		{
			key_cmds[BASE_VELY] = max_value;
		}
		else
		{
			key_cmds[BASE_VELY] = -max_value;
		}
	}
	else
	{
		key_cmds[BASE_VELY] = 0;
	}

	// Base Angular CW & CCW
	if (keys[SDL_SCANCODE_Z] != keys[SDL_SCANCODE_X])
	{
		if (keys[SDL_SCANCODE_Z] == 1)
		{
			// CCW
			key_cmds[BASE_VELZ] = max_value;
		}
		else
		{
			// CW
			key_cmds[BASE_VELZ] = -max_value;
		}
	}
	else
	{
		key_cmds[BASE_VELZ] = 0;
	}

	// Torso Height Up & Down
	if (keys[SDL_SCANCODE_R] != keys[SDL_SCANCODE_F])
	{

		if (keys[SDL_SCANCODE_R] == 1)
		{
			key_cmds[TORSO_VEL] = 1;
		}
		else
		{
			key_cmds[TORSO_VEL] = -1;
		}
	}
	else
	{
		key_cmds[TORSO_VEL] = 0;
	}

	// Toggle Face
	if (keys[SDL_SCANCODE_E] != 0)
	{
		key_cmds[FACE_TOGGLE] = 1;
	}
	else
	{
		key_cmds[FACE_TOGGLE] = 0;
	}

	// Tilt Head up and down
	if (keys[SDL_SCANCODE_W] != keys[SDL_SCANCODE_S])
	{
		if (keys[SDL_SCANCODE_W] == 1)
		{
			// tilt down
			key_cmds[HEADTILT_VEL] = -max_value;
		}
		else
		{
			// tilt up
			key_cmds[HEADTILT_VEL] = max_value;
		}
	}
	else
	{
		key_cmds[HEADTILT_VEL] = 0;
	}

	// Pan Head CCW & CW
	if (keys[SDL_SCANCODE_A] != keys[SDL_SCANCODE_D])
	{
		if (keys[SDL_SCANCODE_A] == 1)
		{
			// pan CCW
			key_cmds[HEADPAN_VEL] = max_value;
		}
		else
		{
			// pan CW
			key_cmds[HEADPAN_VEL] = -max_value;
		}
	}
	else
	{
		key_cmds[HEADPAN_VEL] = 0;
	}

	// Gripper Open & Close
	if (keys[SDL_SCANCODE_I] != keys[SDL_SCANCODE_O])
	{
		if (keys[SDL_SCANCODE_I] == 1)
		{
			// close claws
			key_cmds[GRIPPER_VEL] = 1;
		}
		else
		{
			// open claws
			key_cmds[GRIPPER_VEL] = -1;
		}
	}
	else
	{
		key_cmds[GRIPPER_VEL] = 0;
	}

	// Wrist Up & Down
	if (keys[SDL_SCANCODE_N] != keys[SDL_SCANCODE_M])
	{
		if (keys[SDL_SCANCODE_N] == 1)
		{
			// open claws
			key_cmds[WRIST_VEL] = 1;
		}
		else
		{
			// close claws
			key_cmds[WRIST_VEL] = -1;
		}
	}
	else
	{
		key_cmds[WRIST_VEL] = 0;
	}

	// Rotate Arm Joint CW or CCW
	if (keys[SDL_SCANCODE_K] != keys[SDL_SCANCODE_L])
	{
		if (keys[SDL_SCANCODE_K] == 1)
		{
			// rotate joint CCW
			key_cmds[ARM_JOINT_VEL] = max_value;
		}
		else
		{
			// rotate joint CW
			key_cmds[ARM_JOINT_VEL] = -max_value;
		}
	}
	else
	{
		key_cmds[ARM_JOINT_VEL] = 0;
	}

	// Toggle the arm joint to control
	if (keys[SDL_SCANCODE_J])
	{
		arm_joint_toggle = 1;
	}
	else if (keys[SDL_SCANCODE_SEMICOLON])
	{
		arm_joint_toggle = -1;
	}

	mapVelocity(key_cmds, max_value);

	return key_cmds;
}

float* TeleopController::getGamepadCmds()
{
	static float cmds[] = {0, 0, 0, 0, 0};
	const int max_value = 32769;
	const int deadzone = 300;

	int axis_leftx;
	int axis_lefty;
	int axis_rightx;
	int axis_righty;

	loadGamepad();

	// LEFT JOY STICK X FOR SIDEWAYS BASE TRANSLATION
	axis_leftx = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX);

	if (std::abs(axis_leftx) > deadzone)
	{
		cmds[BASE_VELX] = axis_leftx;
	}
	else
	{
		cmds[BASE_VELX] = 0;
	}

	// LEFT JOY STICK Y FOR FORWARD BASE TRANSLATION
	axis_lefty = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTY);

	if (std::abs(axis_lefty) > deadzone)
	{
		cmds[BASE_VELY] = -axis_lefty;
	}
	else
	{
		cmds[BASE_VELY] = 0;
	}

	// LEFT & RIGHT BOTTOM TRIGGERS FOR CW ROTATION AND CCW ROTATION
	int cww_rot = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERLEFT);
	int cw_rot = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);

	cmds[BASE_VELZ] = cww_rot - cw_rot;

	// A & Y BUTTONS FOR TORSO UP AND DOWN
	int torso_up = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_Y);
	int torso_down = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_A);

	cmds[TORSO_VEL] = torso_up - torso_down;

	// START BUTTON FOR FACE TOGGLING
	cmds[FACE_TOGGLE] = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_START);

	// RIGHT JOY STICK X FOR HEAD PAN
	axis_rightx = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_RIGHTX);

	if (std::abs(axis_rightx) > deadzone)
	{
		cmds[HEADPAN_VEL] = axis_rightx;
	}
	else
	{
		cmds[HEADPAN_VEL] = 0;
	}

	// RIGHT JOY STICK Y FOR HEAD TILT
	axis_righty = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_RIGHTY);

	if (std::abs(axis_righty) > deadzone)
	{
		cmds[HEADTILT_VEL] = axis_righty;
	}
	else
	{
		cmds[HEADTILT_VEL] = 0;
	}

	// TOP TRIGGERS TO OPEN AND CLOSE GRIPPER
	int grip_close = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_LEFTSHOULDER);
	int grip_open = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER);

	cmds[GRIPPER_VEL] = grip_open - grip_close;

	// Wrist Up & Down
	if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_X))
	{
		cmds[WRIST_VEL] = 1;
	}
	else if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_B))
	{
		cmds[WRIST_VEL] = -1;
	}
	else
	{
		cmds[WRIST_VEL] = 0;
	}

	// Rotate Arm Joint CW or CCW
	if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_DPAD_LEFT))
	{
		cmds[ARM_JOINT_VEL] = max_value;
	}
	else if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_DPAD_RIGHT))
	{
		cmds[ARM_JOINT_VEL] = -max_value;
	}
	else
	{
		cmds[ARM_JOINT_VEL] = 0;
	}

	// Toggle the arm joint to control
	if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_DPAD_UP))
	{
		arm_joint_toggle = 1;
	}
	else if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_DPAD_DOWN))
	{
		arm_joint_toggle = -1;
	}

	mapVelocity(cmds, max_value);

	return cmds;
}

void TeleopController::mapVelocity(float *val_array, int ceiling_value)
{
	int base_max;
	float velx = val_array[BASE_VELX];
	float vely = val_array[BASE_VELY];

	float velr = std::sqrt(velx * velx + vely * vely);

	if (velr > ceiling_value)
	{
		base_max = velr;
	}
	else
	{
		base_max = ceiling_value;
	}

	// Map value range for base x and y
	val_array[BASE_VELX] = velx * MAX_BASE_RADIAL_VEL / base_max;
	val_array[BASE_VELY] = vely * MAX_BASE_RADIAL_VEL / base_max;

	val_array[TORSO_VEL] = val_array[TORSO_VEL] * MAX_TORSO_VEL;

	// Map value range for base angular
	val_array[BASE_VELZ] = val_array[BASE_VELZ] * MAX_BASE_ANGULAR_VEL / ceiling_value;

	// Map value range for head pan & tilt
	val_array[HEADTILT_VEL] = val_array[HEADTILT_VEL] * MAX_HEADTILT_VEL / ceiling_value;
	val_array[HEADPAN_VEL] = val_array[HEADPAN_VEL] * MAX_HEADPAN_VEL / ceiling_value;

	// Map value range for arm & gripper
	val_array[GRIPPER_VEL] = val_array[GRIPPER_VEL] * MAX_GRIPPER_VEL;
	val_array[WRIST_VEL] = val_array[WRIST_VEL] * MAX_WRIST_VEL;
	val_array[ARM_JOINT_VEL] = val_array[ARM_JOINT_VEL] * MAX_ARM_JOINT_VEL / ceiling_value;
}

void TeleopController::reRenderImage()
{
	SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, 0xFF);
	SDL_RenderClear(renderer);
	SDL_RenderCopy(renderer, texture, NULL, NULL);
	SDL_RenderPresent(renderer);
}

bool TeleopController::checkQuitStatus()
{
	return quit;
}

int TeleopController::armToggled()
/*
 * Returns:
 * 	arm_joint_toggle: -1 for reverse toggle, 1 for forward toggle, 0 for not toggled
 */
{
	return arm_joint_toggle;
}

void TeleopController::resetArmToggle()
{
	arm_joint_toggle = 0;
}

}  // namespace v2mini_teleop
