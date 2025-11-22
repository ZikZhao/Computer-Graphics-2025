#include <fstream>
#include <vector>
#include <sstream>
#include <map>
#include <stdexcept>
#include <memory>
#include "DrawingWindow.h"
#include "window.hpp"
#include "world.hpp"

#define WIDTH 1920
#define HEIGHT 1080

int main(int argc, char *argv[]) {
    assert(argc >= 2 && "Please provide a .obj file as a command line argument.");

	DrawingWindow drawing_window = DrawingWindow(WIDTH, HEIGHT, false);
    Window window(drawing_window);
    
    World world;
    world.load_files(std::vector<std::string>(argv + 1, argv + argc));
    Renderer renderer(drawing_window);

    // Register event handlers
    window.add_event_handler([&](const SDL_Event& event) {
        if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_SPACE) {
            window.save_ppm("screenshot.ppm");
            window.save_bmp("screenshot.bmp");
        }
        world.handle_event(event);
        renderer.handle_event(event);
    });

	SDL_Event event;
    std::size_t last_print_time = std::chrono::system_clock::now().time_since_epoch().count();
    std::size_t fps = 0;
	while (true) {
		while (window.poll_events(event)) {
            // Events are handled by registered handlers
        }
        window.clear_pixels();
        world.orbiting();
        renderer.render(world);
		window.render_frame();
        
        fps++;
        std::size_t now = std::chrono::system_clock::now().time_since_epoch().count();
        if (now - last_print_time >= 1000000000) { // 1 second in nanoseconds
            std::cout << "FPS: " << fps << std::endl;
            fps = 0;
            last_print_time = now;
        }
	}
}
