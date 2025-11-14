#include <fstream>
#include <vector>
#include <sstream>
#include <map>
#include <stdexcept>
#include <memory>
#include "DrawingWindow.h"
#include "world.hpp"

#define WIDTH 1920
#define HEIGHT 1080

int main(int argc, char *argv[]) {
    assert(argc >= 2 && "Please provide a .obj file as a command line argument.");

	DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
    World world;
    world.load_file(argv[1]);
    Renderer renderer(window);

	SDL_Event event;
	while (true) {
		while (window.pollForInputEvents(event)) {
            if (event.type == SDL_QUIT || (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)) {
                window.exitCleanly();
                return 0;
            }
            world.handle_event(event);
            renderer.handle_event(event);
        }
        window.clearPixels();
        world.draw(renderer);
		window.renderFrame();
	}
}
