#ifndef __ROBOT_CONTROLLER_H
#define __ROBOT_CONTROLLER_H

#include <SDL2/SDL.h>

const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;

namespace v2mini_motion {

class RobotController {

	private:
		bool quit = false;
		int icon_vel[3] = {0, 0, 0}; // todo named elements
		SDL_Window* window = NULL;
		SDL_Surface* surface = NULL;
		SDL_Renderer* renderer = NULL;
		SDL_Texture* texture = NULL;
		SDL_GameController *controller = NULL;

	public:
		RobotController();
		~RobotController();
		int* getKeyCmds();
		int* getGamepadCmds();
		bool checkQuitStatus();
		void reRenderImage();
};

}

#endif
