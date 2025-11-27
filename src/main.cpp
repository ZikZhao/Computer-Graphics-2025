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
    window.register_key(
        {SDL_SCANCODE_W, SDL_SCANCODE_S, SDL_SCANCODE_A, SDL_SCANCODE_D, SDL_SCANCODE_SPACE, SDL_SCANCODE_C},
        Window::Trigger::ANY_PRESSED_NO_MODIFIER,
        [&](const Window::KeyState& ks, float dt) {
            constexpr FloatType move_step = 3.0f;
            FloatType fwd = (ks[SDL_SCANCODE_W] ? 1.0f : 0.0f) - (ks[SDL_SCANCODE_S] ? 1.0f : 0.0f);
            FloatType right = (ks[SDL_SCANCODE_D] ? 1.0f : 0.0f) - (ks[SDL_SCANCODE_A] ? 1.0f : 0.0f);
            FloatType up = (ks[SDL_SCANCODE_SPACE] ? 1.0f : 0.0f) - (ks[SDL_SCANCODE_C] ? 1.0f : 0.0f);
            if (fwd != 0.0f || right != 0.0f || up != 0.0f) {
                world.camera_.move(fwd * move_step, right * move_step, up * move_step, dt);
                renderer.reset_accumulation();
            }
        });

    window.register_key({SDL_SCANCODE_O}, Window::Trigger::ANY_JUST_PRESSED,
        [&](const Window::KeyState&, float) {
            if (!world.camera_.is_orbiting_) {
                world.camera_.start_orbiting(world.camera_.orbit_target_);
            } else {
                world.camera_.stop_orbiting();
            }
            renderer.reset_accumulation();
        });

    window.register_key(
        {SDL_SCANCODE_1, SDL_SCANCODE_2, SDL_SCANCODE_3, SDL_SCANCODE_4},
        Window::Trigger::ANY_JUST_PRESSED,
        [&](const Window::KeyState& ks, float) {
            if (ks[SDL_SCANCODE_1]) renderer.mode_ = Renderer::Mode::WIREFRAME;
            else if (ks[SDL_SCANCODE_2]) renderer.mode_ = Renderer::Mode::RASTERIZED;
            else if (ks[SDL_SCANCODE_3]) renderer.mode_ = Renderer::Mode::RAYTRACED;
            else if (ks[SDL_SCANCODE_4]) renderer.mode_ = Renderer::Mode::DEPTH_OF_FIELD;
        });

    window.register_key({SDL_SCANCODE_G}, Window::Trigger::ANY_JUST_PRESSED,
        [&](const Window::KeyState&, float) { renderer.gamma_ = (renderer.gamma_ == 2.2f) ? 1.0f : 2.2f; });
    window.register_key({SDL_SCANCODE_H}, Window::Trigger::ANY_JUST_PRESSED,
        [&](const Window::KeyState&, float) { renderer.soft_shadows_enabled_ = !renderer.soft_shadows_enabled_; });
    window.register_key({SDL_SCANCODE_P}, Window::Trigger::ANY_JUST_PRESSED,
        [&](const Window::KeyState&, float) {
            if (renderer.raytracer_->is_photon_map_ready()) {
                renderer.caustics_enabled_ = !renderer.caustics_enabled_;
                std::cout << "Caustics (photon mapping): "
                          << (renderer.caustics_enabled_ ? "ENABLED" : "DISABLED") << std::endl;
                renderer.reset_accumulation();
            } else {
                std::cout << "Photon map not ready yet, please wait..." << std::endl;
            }
        });
    window.register_key({SDL_SCANCODE_V}, Window::Trigger::ANY_JUST_PRESSED,
        [&](const Window::KeyState&, float) {
            static FloatType prev_intensity = world.light_intensity();
            RayTracer::DebugVisualizeCausticsOnly = !RayTracer::DebugVisualizeCausticsOnly;
            if (RayTracer::DebugVisualizeCausticsOnly) {
                prev_intensity = world.light_intensity();
                world.set_light_intensity(0.0f);
                std::cout << "DEBUG Caustics-only mode: ON (direct light intensity set to 0)" << std::endl;
            } else {
                world.set_light_intensity(prev_intensity);
                std::cout << "DEBUG Caustics-only mode: OFF (direct light intensity restored)" << std::endl;
            }
            renderer.reset_accumulation();
        });
    
    // Screenshot Save (Ctrl+S)
    window.register_key({SDL_SCANCODE_LCTRL, SDL_SCANCODE_S}, Window::Trigger::ALL_JUST_PRESSED,
        [&](const Window::KeyState& ks, float) {
            window.save_ppm("screenshot.ppm");
            window.save_bmp("screenshot.bmp");
            std::cout << "Screenshot saved as screenshot.ppm and screenshot.bmp" << std::endl;
        });
    
    window.register_key({SDL_SCANCODE_RCTRL, SDL_SCANCODE_S}, Window::Trigger::ALL_JUST_PRESSED,
        [&](const Window::KeyState& ks, float) {
            window.save_ppm("screenshot.ppm");
            window.save_bmp("screenshot.bmp");
            std::cout << "Screenshot saved as screenshot.ppm and screenshot.bmp" << std::endl;
        });
    
    // Video Recording Toggle (Ctrl+Shift+S)
    window.register_key({SDL_SCANCODE_LCTRL, SDL_SCANCODE_LSHIFT, SDL_SCANCODE_S}, Window::Trigger::ALL_JUST_PRESSED,
        [&](const Window::KeyState&, float) { video_recorder.toggleRecording(); });
    
    window.register_key({SDL_SCANCODE_RCTRL, SDL_SCANCODE_RSHIFT, SDL_SCANCODE_S}, Window::Trigger::ALL_JUST_PRESSED,
        [&](const Window::KeyState&, float) { video_recorder.toggleRecording(); });

    window.register_mouse(SDL_BUTTON_LEFT, Window::Trigger::ANY_PRESSED,
        [&](int xrel, int yrel, float dt) {
            if (xrel == 0 && yrel == 0) return;
            FloatType dx = -static_cast<FloatType>(xrel) * world.camera_.mouse_sensitivity_;
            FloatType dy = static_cast<FloatType>(yrel) * world.camera_.mouse_sensitivity_;
            world.camera_.rotate(dx, dy);
            renderer.reset_accumulation();
        });

    auto last_print = std::chrono::steady_clock::now();
    std::size_t fps = 0;
    
    while (true) {
        if (!window.process_events()) break;
        
        // Render frame
        world.orbiting();
        renderer.render();
        window.update();
        
        // Capture frame if recording
        if (video_recorder.is_recording()) {
            video_recorder.capture_frame();
        }
        
        // FPS counter
        fps++;
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - last_print;
        if (elapsed >= std::chrono::seconds(1)) {
            std::cout << "FPS: " << std::fixed << std::setprecision(1) 
                      << static_cast<double>(fps / elapsed.count()) << std::endl;
            fps = 0;
            last_print = now;
        }
    }
    
    return 0;
}
