#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <ros/package.h>
#include <stdexcept>
#include <stdio.h>
#include <cmath>

#include "v2mini_telop/user_inputs.h"

namespace v2mini_telop
{

const Uint8 *keys = SDL_GetKeyboardState(NULL);

UserInputs::UserInputs()
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
			std::string ros_path = ros::package::getPath("v2mini_telop") + "/robo_icon.jpg";

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

UserInputs::~UserInputs()
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

void UserInputs::loadGamepad()
{
	controller = SDL_GameControllerOpen(0);

	if (controller == NULL)
	{
		printf("No Game Controller detected: %s\n", SDL_GetError());
	}
}

float* UserInputs::getKeyCmds()
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

	// Angular CW & CCW
	if (keys[SDL_SCANCODE_Q] != keys[SDL_SCANCODE_W])
	{
		if (keys[SDL_SCANCODE_Q] == 1)
		{
			key_cmds[BASE_VELZ] = max_value;
		}
		else
		{
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
		key_cmds[FACE] = 1;
	}
	else
	{
		key_cmds[FACE] = 0;
	}


	mapVelocity(key_cmds, max_value);

	return key_cmds;
}

float* UserInputs::getGamepadCmds()
{
	static float cmds[] = {0, 0, 0, 0, 0};
	const int max_value = 32769;
	const int deadzone = 300;

	int axis_leftx;
	int axis_lefty;
	int axis_righx;
	int axis_righy;

	loadGamepad();

	// LEFT JOY STICK X FOR SIDEWAYS TRANSLATION
	axis_leftx = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX);

	if (std::abs(axis_leftx) > deadzone)
	{
		cmds[BASE_VELX] = axis_leftx;
	}
	else
	{
		cmds[BASE_VELX] = 0;
	}

	// LEFT JOY STICK Y FOR FORWARD TRANSLATION
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

	// B BUTTON FOR FACE TOGGLING
	cmds[FACE] = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_B);

//	// CONTROL ARM MOTION IN THE YZ PLANE WITH DPAD UP KEYBINDING
//	if (SDL_CONTROLLER_BUTTON_DPAD_UP)
//	{
//		// RIGHT JOY STICK X FOR SIDEWAYS TRANSLATION
//		axis_righx. = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX);
//
//		if (std::abs(axis_leftx) > deadzone)
//		{
//			cmds[BASE_VELX] = axis_leftx;
//		}
//		else
//		{
//			cmds[BASE_VELX] = 0;
//		}
//
//		// RIGHT JOY STICK Y FOR FORWARD TRANSLATION
//		axis_righy = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_RIGHTY);
//
//		if (std::abs(axis_lefty) > deadzone)
//		{
//			cmds[ARM_VELYZ] = -axis_lefty;
//		}
//		else
//		{
//			cmds[ARM_VELYZ] = 0;
//		}
//	}
//	// CONTROL ARM MOTION IN THE XZ PLANE WITH DPAD LEFT KEYBINDING
//	else if (SDL_CONTROLLER_BUTTON_DPAD_LEFT)
//	{
//
//	}
//	// CONTROL ARM MOTION IN THE XY PLANE WITH DPAD DOWN KEYBINDING
//	else if (SDL_CONTROLLER_BUTTON_DPAD_DOWN)
//	{
//
//	}

	mapVelocity(cmds, max_value);

	return cmds;
}

void UserInputs::mapVelocity(float *val_array, int ceiling_value)
{
	float velx = val_array[BASE_VELX];
	float vely = val_array[BASE_VELY];

	float velr = std::sqrt(velx * velx + vely * vely);

	if (velr > ceiling_value)
	{
		ceiling_value = velr;
	}

	// Map value range for x and y
	val_array[BASE_VELX] = velx * MAX_BASE_RADIAL_VEL / ceiling_value;
	val_array[BASE_VELY] = vely * MAX_BASE_RADIAL_VEL / ceiling_value;

	// Map value range for angular
	val_array[BASE_VELZ] = val_array[BASE_VELZ] * MAX_BASE_ANGULAR_VEL / ceiling_value;
}

void UserInputs::reRenderImage()
{
	SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, 0xFF);
	SDL_RenderClear(renderer);
	SDL_RenderCopy(renderer, texture, NULL, NULL);
	SDL_RenderPresent(renderer);
}

bool UserInputs::checkQuitStatus()
{
	return quit;
}

} 		// namespace v2mini_telop
