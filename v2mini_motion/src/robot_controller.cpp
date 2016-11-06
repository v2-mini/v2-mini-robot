#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <stdexcept>
#include <stdio.h>
#include <string>

#include "v2mini_motion/robot_controller.h"

namespace v2mini_motion {

const Uint8 *keys = SDL_GetKeyboardState(NULL);

RobotController::RobotController() {

	// Initialize SDL
	if (SDL_Init(SDL_INIT_VIDEO) == 0) {

		//Set texture filtering to linear
		SDL_SetHint( SDL_HINT_RENDER_SCALE_QUALITY, "1");

		window = SDL_CreateWindow("V2Mini Controller", SDL_WINDOWPOS_UNDEFINED,
				SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);

		if (window == NULL) {
			throw std::runtime_error("SDL failed to create window\n");

		} else {
			// Setup SDL window and load robot icon
			renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

			if (renderer == NULL) {
				throw std::runtime_error("SDL failed to render\n");
			}

			SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, 0xFF);

			// todo remove .png stuff vv

			IMG_Init(IMG_INIT_PNG);
			SDL_Surface* loadedSurface = IMG_Load("robo_icon.jpg");
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
	window = NULL;
	renderer = NULL;
	texture = NULL;

	IMG_Quit();
	SDL_Quit();
}

int* RobotController::getKeyCmds() {
	static int base_cmds[] = {0, 0, 0, 0};

	SDL_PumpEvents();

	if(keys[SDL_SCANCODE_ESCAPE]) {
		quit = true;
	}

	// Base Left & Right
	if (keys[SDL_SCANCODE_LEFT] != keys[SDL_SCANCODE_RIGHT]) {

		if (keys[SDL_SCANCODE_RIGHT] == 1) {
			icon_vel[0] = 1;

		} else {
			icon_vel[0] = -1;
		}

	} else {
		icon_vel[0] = 0;
	}

	// Base Forward & Back
	if (keys[SDL_SCANCODE_UP] != keys[SDL_SCANCODE_DOWN]) {

		if (keys[SDL_SCANCODE_UP] == 1) {
			icon_vel[1] = 1;

		} else {
			icon_vel[1] = -1;
		}

	} else {
		icon_vel[1] = 0;
	}

	// Angular CW & CCW
	if (keys[SDL_SCANCODE_Q] != keys[SDL_SCANCODE_W]) {

		if (keys[SDL_SCANCODE_Q] == 1) {
			icon_vel[2] = 1;

		} else {
			icon_vel[2] = -1;
		}

	} else {
		icon_vel[2] = 0;
	}

	int torso_vel = 0;

	// Torso Height Up & Down
	if (keys[SDL_SCANCODE_R] != keys[SDL_SCANCODE_F]) {

		if (keys[SDL_SCANCODE_R] == 1) {
			torso_vel = 1;

		} else {
			torso_vel = -1;
		}
	}

	base_cmds[0] = icon_vel[0];
	base_cmds[1] = icon_vel[1];
	base_cmds[2] = icon_vel[2];
	base_cmds[3] = torso_vel;

//	printf("x= %i\n", base_cmds[0]);
//	printf("y= %i\n", base_cmds[1]);
//	printf("w= %i\n", base_cmds[2]);
//	printf("t= %i\n", base_cmds[3]);

	return base_cmds;
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
