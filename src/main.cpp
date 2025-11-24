#include <fstream>
#include <vector>
#include <sstream>
#include <map>
#include <stdexcept>
#include <memory>
#include <iostream>
#include "DrawingWindow.h"
#include "world.hpp"
#include "renderer.hpp"

constexpr std::size_t WIDTH = 960;
constexpr std::size_t HEIGHT = 540;

int main(int argc, char *argv[]) {
    assert(argc >= 2 && "Please provide a .obj file as a command line argument.");
    
    // Load world first
    World world;
    world.load_files(std::vector<std::string>(argv + 1, argv + argc));
    
    // Initialize window after successful world loading
    DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
    Renderer renderer(window, world);

    SDL_Event event;
    std::size_t last_print_time = std::chrono::system_clock::now().time_since_epoch().count();
    std::size_t fps = 0;
    bool ctrl_s_pressed_last_frame = false;
    
    while (true) {
        // Poll and handle events
        while (SDL_PollEvent(&event)) {
            // Handle window close events
            if (event.type == SDL_QUIT ||
                (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE) ||
                (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)) {
                window.exitCleanly();
                return 0;
            }
            // Dispatch to world and renderer
            world.handle_event(event);
            renderer.handle_event(event);
        }
        
        // Check for Ctrl+S combination (poll keyboard state)
        const Uint8* keystate = SDL_GetKeyboardState(nullptr);
        bool ctrl_pressed = keystate[SDL_SCANCODE_LCTRL] || keystate[SDL_SCANCODE_RCTRL];
        bool s_pressed = keystate[SDL_SCANCODE_S];
        bool ctrl_s_pressed = ctrl_pressed && s_pressed;
        
        // Save screenshot on rising edge (when Ctrl+S becomes pressed)
        if (ctrl_s_pressed && !ctrl_s_pressed_last_frame) {
            window.savePPM("screenshot.ppm");
            window.saveBMP("screenshot.bmp");
            std::cout << "Screenshot saved as screenshot.ppm and screenshot.bmp" << std::endl;
        }
        ctrl_s_pressed_last_frame = ctrl_s_pressed;
        
        window.clearPixels();
        world.update();
        world.orbiting();
        renderer.render();
        window.renderFrame();
        
        fps++;
        std::size_t now = std::chrono::system_clock::now().time_since_epoch().count();
        if (now - last_print_time >= 1000000000) { // 1 second in nanoseconds
            std::cout << "FPS: " << std::fixed << std::setprecision(1) << static_cast<double>(fps / ((now - last_print_time) / 1000000000.0)) << std::endl;
            fps = 0;
            last_print_time = now;
        }
    }
}