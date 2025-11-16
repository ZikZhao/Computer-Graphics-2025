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
    world.load_files(std::vector<std::string>(argv + 1, argv + argc));
    Renderer renderer(window);

	SDL_Event event;
    std::size_t last_print_time = std::chrono::system_clock::now().time_since_epoch().count();
    std::size_t fps = 0;
	while (true) {
		while (window.pollForInputEvents(event)) {
            if (event.type == SDL_QUIT || (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)) {
                window.exitCleanly();
                return 0;
            }
            else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_SPACE) {
                window.savePPM("screenshot.ppm");
                window.saveBMP("screenshot.bmp");
            }
            world.handle_event(event);
            renderer.handle_event(event);
        }
        window.clearPixels();
        world.orbiting();
        renderer.render(world);
		window.renderFrame();
        
        fps++;
        std::size_t now = std::chrono::system_clock::now().time_since_epoch().count();
        if (now - last_print_time >= 1000000000) { // 1 second in nanoseconds
            std::cout << "FPS: " << fps << std::endl;
            fps = 0;
            last_print_time = now;
        }
	}
}

// int main(int args, char* argc[]) {
//     ThreadPool<> pool;
//     for (int i = 0; i < 10; i++)
//     pool.enqueue([]() {
//         std::cout << "Hello from the thread pool!" << std::endl;
//     });
//     pool.wait_idle();
// }
