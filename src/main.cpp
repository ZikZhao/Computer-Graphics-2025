#include <fstream>
#include <vector>
#include <sstream>
#include <map>
#include <stdexcept>
#include <memory>
#include <iostream>
#include "DrawingWindow.h"
#include "world.hpp"

constexpr std::size_t WIDTH = 960;
constexpr std::size_t HEIGHT = 540;

int main(int argc, char *argv[]) {
    assert(argc >= 2 && "Please provide a .obj file as a command line argument.");
    
    // Load world first (fail fast if files are missing/invalid)
    World world;
    world.load_files(std::vector<std::string>(argv + 1, argv + argc));
    
    // Initialize window after successful world loading
    DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
    Renderer renderer(window, world);

    SDL_Event event;
    std::size_t last_print_time = std::chrono::system_clock::now().time_since_epoch().count();
    std::size_t fps = 0;
    
    while (true) {
        // Poll and handle events
        while (SDL_PollEvent(&event)) {
            // Handle window close events
            if (event.type == SDL_QUIT) {
                window.exitCleanly();
                return 0;
            }
            if ((event.type == SDL_KEYDOWN) && (event.key.keysym.sym == SDLK_ESCAPE)) {
                window.exitCleanly();
                return 0;
            }
            if ((event.type == SDL_WINDOWEVENT) && (event.window.event == SDL_WINDOWEVENT_CLOSE)) {
                window.exitCleanly();
                return 0;
            }
            
            // Handle screenshot saving
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_SPACE) {
                window.savePPM("screenshot.ppm");
                window.saveBMP("screenshot.bmp");
            }
            
            // Dispatch to world and renderer
            world.handle_event(event);
            renderer.handle_event(event);
        }
        
        window.clearPixels();
        world.orbiting();
        renderer.render();
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
