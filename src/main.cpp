#include <fstream>
#include <vector>
#include <sstream>
#include <map>
#include <stdexcept>
#include <memory>
#include <iostream>
#include "window.hpp"
#include "world.hpp"
#include "renderer.hpp"
#include "raytracer.hpp"
#include "utils.hpp"

constexpr std::size_t WIDTH = 960;
constexpr std::size_t HEIGHT = 540;

int main(int argc, char *argv[]) {
    assert(argc >= 2 && "Please provide a .obj file as a command line argument.");
    
    // Load world first
    World world;
    world.load_files(std::vector<std::string>(argv + 1, argv + argc));
    
    // Initialize window after successful world loading
    Window window(WIDTH, HEIGHT, false);
    Renderer renderer(window, world);
    VideoRecorder video_recorder(window.get_pixel_buffer(), WIDTH, HEIGHT);

    // Centralized input callbacks
    constexpr FloatType move_step = 0.1f;
    window.register_key(
        {SDL_SCANCODE_W, SDL_SCANCODE_S, SDL_SCANCODE_A, SDL_SCANCODE_D, SDL_SCANCODE_SPACE, SDL_SCANCODE_C},
        Window::Trigger::ANY_PRESSED_NO_MODIFIER,
        [&](const Window::KeyState& ks) {
            FloatType fwd = (ks[SDL_SCANCODE_W] ? 1.0f : 0.0f) - (ks[SDL_SCANCODE_S] ? 1.0f : 0.0f);
            FloatType right = (ks[SDL_SCANCODE_D] ? 1.0f : 0.0f) - (ks[SDL_SCANCODE_A] ? 1.0f : 0.0f);
            FloatType up = (ks[SDL_SCANCODE_SPACE] ? 1.0f : 0.0f) - (ks[SDL_SCANCODE_C] ? 1.0f : 0.0f);
            if (fwd != 0.0f || right != 0.0f || up != 0.0f) {
                world.camera_.move(fwd * move_step, right * move_step, up * move_step);
            }
        });

    window.register_key({SDL_SCANCODE_O}, Window::Trigger::ANY_JUST_PRESSED,
        [&](const Window::KeyState&) {
            if (!world.camera_.is_orbiting_) {
                world.camera_.start_orbiting(world.camera_.orbit_target_);
            } else {
                world.camera_.stop_orbiting();
            }
        });

    window.register_key(
        {SDL_SCANCODE_1, SDL_SCANCODE_2, SDL_SCANCODE_3, SDL_SCANCODE_4},
        Window::Trigger::ANY_JUST_PRESSED,
        [&](const Window::KeyState& ks) {
            if (ks[SDL_SCANCODE_1]) renderer.mode_ = Renderer::Wireframe;
            else if (ks[SDL_SCANCODE_2]) renderer.mode_ = Renderer::Rasterized;
            else if (ks[SDL_SCANCODE_3]) renderer.mode_ = Renderer::Raytraced;
            else if (ks[SDL_SCANCODE_4]) renderer.mode_ = Renderer::DepthOfField;
        });

    window.register_key({SDL_SCANCODE_G}, Window::Trigger::ANY_JUST_PRESSED,
        [&](const Window::KeyState&) { renderer.gamma_ = (renderer.gamma_ == 2.2f) ? 1.0f : 2.2f; });
    window.register_key({SDL_SCANCODE_H}, Window::Trigger::ANY_JUST_PRESSED,
        [&](const Window::KeyState&) { renderer.soft_shadows_enabled_ = !renderer.soft_shadows_enabled_; });
    window.register_key({SDL_SCANCODE_P}, Window::Trigger::ANY_JUST_PRESSED,
        [&](const Window::KeyState&) {
            if (renderer.raytracer_->is_photon_map_ready()) {
                renderer.caustics_enabled_ = !renderer.caustics_enabled_;
                std::cout << "Caustics (photon mapping): "
                          << (renderer.caustics_enabled_ ? "ENABLED" : "DISABLED") << std::endl;
            } else {
                std::cout << "Photon map not ready yet, please wait..." << std::endl;
            }
        });
    window.register_key({SDL_SCANCODE_V}, Window::Trigger::ANY_JUST_PRESSED,
        [&](const Window::KeyState&) {
            RayTracer::debug_visualize_caustics_only = !RayTracer::debug_visualize_caustics_only;
            std::cout << "DEBUG Caustics-only mode: "
                      << (RayTracer::debug_visualize_caustics_only ? "ON (verify Beer-Lambert color)" : "OFF")
                      << std::endl;
        });
    
    // Screenshot Save (Ctrl+S)
    window.register_key({SDL_SCANCODE_LCTRL, SDL_SCANCODE_S}, Window::Trigger::ALL_JUST_PRESSED,
        [&](const Window::KeyState& ks) {
            window.save_ppm("screenshot.ppm");
            window.save_bmp("screenshot.bmp");
            std::cout << "Screenshot saved as screenshot.ppm and screenshot.bmp" << std::endl;
        });
    
    window.register_key({SDL_SCANCODE_RCTRL, SDL_SCANCODE_S}, Window::Trigger::ALL_JUST_PRESSED,
        [&](const Window::KeyState& ks) {
            window.save_ppm("screenshot.ppm");
            window.save_bmp("screenshot.bmp");
            std::cout << "Screenshot saved as screenshot.ppm and screenshot.bmp" << std::endl;
        });
    
    // Video Recording Toggle (Ctrl+Shift+S)
    window.register_key({SDL_SCANCODE_LCTRL, SDL_SCANCODE_LSHIFT, SDL_SCANCODE_S}, Window::Trigger::ALL_JUST_PRESSED,
        [&](const auto&) { video_recorder.toggleRecording(); });
    
    window.register_key({SDL_SCANCODE_RCTRL, SDL_SCANCODE_RSHIFT, SDL_SCANCODE_S}, Window::Trigger::ALL_JUST_PRESSED,
        [&](const auto&) { video_recorder.toggleRecording(); });

    window.register_mouse(SDL_BUTTON_LEFT, Window::Trigger::ANY_PRESSED,
        [&](int xrel, int yrel) {
            if (xrel == 0 && yrel == 0) return;
            FloatType dx = -static_cast<FloatType>(xrel) * world.camera_.mouse_sensitivity_;
            FloatType dy = static_cast<FloatType>(yrel) * world.camera_.mouse_sensitivity_;
            world.camera_.rotate(dx, dy);
        });

    std::size_t last_print_time = std::chrono::system_clock::now().time_since_epoch().count();
    std::size_t fps = 0;
    
    while (true) {
        // Process window events (handles quit, escape, etc.)
        if (!window.process_events()) {
            break; // Exit main loop if window should close
        }
        
        // Update window input state and process event bindings
        window.update();
        
        // Render frame
        window.clear_pixels();
        world.update();
        world.orbiting();
        renderer.render();
        window.render();
        
        // Capture frame if recording
        if (video_recorder.isRecording()) {
            video_recorder.captureFrame();
        }
        
        // FPS counter
        fps++;
        std::size_t now = std::chrono::system_clock::now().time_since_epoch().count();
        if (now - last_print_time >= 1000000000) { // 1 second in nanoseconds
            std::cout << "FPS: " << std::fixed << std::setprecision(1) 
                      << static_cast<double>(fps / ((now - last_print_time) / 1000000000.0)) << std::endl;
            fps = 0;
            last_print_time = now;
        }
    }
    
    return 0;
}
