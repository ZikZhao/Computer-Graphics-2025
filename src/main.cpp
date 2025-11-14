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
	DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
    World world;
    world.load_file("../model/cornell-box.obj");
	SDL_Event event;
	while (true) {
		// We MUST poll for events - otherwise the window will freeze !
		if (window.pollForInputEvents(event)) world.handle_event(event, window);
        window.clearPixels();
        world.draw(window);
		// Need to draw the frame at the end, or nothing actually gets shown on the screen !
		window.renderFrame();
	}
}
