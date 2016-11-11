#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <stdexcept>
#include <stdio.h>
#include <cmath>

#include "v2mini_motion/robot_controller.h"

namespace v2mini_motion {

const Uint8 *keys = SDL_GetKeyboardState(NULL);

RobotController::RobotController() {

	// Initialize SDL
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) == 0) {

		//Set texture filtering to linear
		SDL_SetHint( SDL_HINT_RENDER_SCALE_QUALITY, "1");

		window = SDL_CreateWindow("V2Mini Controller", SDL_WINDOWPOS_UNDEFINED,
				SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);

		if (window == NULL) {
			throw std::runtime_error("SDL failed to create window\n");

		} else {

			controller = SDL_GameControllerOpen(0);

			if (controller == NULL) {
				printf("open failed: %s\n", SDL_GetError());
			}

			// Setup SDL window and load robot icon
			renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

			if (renderer == NULL) {
				throw std::runtime_error("SDL failed to render\n");
			}

			SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, 0xFF);

			// todo remove .png stuff vv
			IMG_Init(IMG_INIT_PNG);
//			std::string ros_path = ros::package::getPath("v2mini_robot")
			SDL_Surface* loadedSurface = IMG_Load("/home/jon/Pictures/robo_icon.jpg"); //todo make rel path
			texture = SDL_CreateTextureFromSurface(renderer, loadedSurface);
			SDL_FreeSurface(loadedSurface);

			reRenderImage();

		}

	} else {
		throw std::runtime_error("SDL failed to initialize\n");
	}
}

RobotController::~RobotController() {
	//Deallocate SDL resources
	SDL_DestroyTexture(texture);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);

	if (controller != NULL) {
		SDL_GameControllerClose(controller);
		controller = NULL;
	}

	window = NULL;
	renderer = NULL;
	texture = NULL;

	IMG_Quit();
	SDL_Quit();
}

int* RobotController::getKeyCmds() {

	static int key_cmds[] = {0, 0, 0, 0};

	if(keys[SDL_SCANCODE_ESCAPE]) {
		quit = true;
	}

	// Base Left & Right
	if (keys[SDL_SCANCODE_LEFT] != keys[SDL_SCANCODE_RIGHT]) {

		if (keys[SDL_SCANCODE_RIGHT] == 1) {
			icon_vel[0] = 10000;

		} else {
			icon_vel[0] = -10000;
		}

	} else {
		icon_vel[0] = 0;
	}

	// Base Forward & Back
	if (keys[SDL_SCANCODE_UP] != keys[SDL_SCANCODE_DOWN]) {

		if (keys[SDL_SCANCODE_UP] == 1) {
			icon_vel[1] = 10000;

		} else {
			icon_vel[1] = -10000;
		}

	} else {
		icon_vel[1] = 0;
	}

	// Angular CW & CCW
	if (keys[SDL_SCANCODE_Q] != keys[SDL_SCANCODE_W]) {

		if (keys[SDL_SCANCODE_Q] == 1) {
			icon_vel[2] = 10000;

		} else {
			icon_vel[2] = -10000;
		}

	} else {
		icon_vel[2] = 0;
	}

	int torso_vel = 0;

	// Torso Height Up & Down
	if (keys[SDL_SCANCODE_R] != keys[SDL_SCANCODE_F]) {

		if (keys[SDL_SCANCODE_R] == 1) {
			torso_vel = 10000;

		} else {
			torso_vel = -10000;
		}
	}

	key_cmds[0] = icon_vel[0];
	key_cmds[1] = icon_vel[1];
	key_cmds[2] = icon_vel[2];
	key_cmds[3] = torso_vel;

	return key_cmds;
}

int* RobotController::getGamepadCmds() {
	static int cmds[] = {0, 0, 0, 0};
	const int deadzone = 1000;

	// LEFT JOY STICK X
	int axis_leftx = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX);

	if (std::abs(axis_leftx) > deadzone) {
		cmds[0] = axis_leftx;
	}
	else {
		cmds[0] = 0;
	}

	// LEFT JOY STICK Y
	int axis_lefty = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTY);

	if (std::abs(axis_lefty) > deadzone) {
		cmds[1] = -axis_lefty;
	}
	else {
		cmds[1] = 0;
	}

	// LEFT & RIGHT BOTTOM TRIGGERS
	int cww_rot = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERLEFT);
	int cw_rot = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);

	cmds[2] = cww_rot - cw_rot;

	// A & Y BUTTONS
	int torso_up = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_Y);
	int torso_down = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_A);

	cmds[3] = torso_up - torso_down;

	return cmds;
}

void RobotController::reRenderImage() {

	SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, 0xFF);
	SDL_RenderClear(renderer);
	SDL_RenderCopy(renderer, texture, NULL, NULL);
	SDL_RenderPresent(renderer);
}

bool RobotController::checkQuitStatus() {
	return quit;
}

} // namespace v2mini_motion
