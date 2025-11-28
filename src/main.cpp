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
#include "video_recorder.hpp"

constexpr std::size_t WIDTH = 960;
constexpr std::size_t HEIGHT = 540;

int main(int argc, char *argv[]) {
    assert(argc >= 2 && "Please provide a .obj file as a command line argument.");
    
    // Load world assets from CLI arguments
    World world;
    world.load_files(std::vector<std::string>(argv + 1, argv + argc));
    
    // Initialize output window and rendering engines
    Window window(WIDTH, HEIGHT, false);
    Renderer renderer(window, world);
    VideoRecorder video_recorder(window.get_pixel_buffer(), WIDTH, HEIGHT);

    
    // Centralized input callbacks: movement and UI toggles
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

    window.register_key(
        {SDL_SCANCODE_Q, SDL_SCANCODE_E},
        Window::Trigger::ANY_PRESSED_NO_MODIFIER,
        [&](const Window::KeyState& ks, float dt) {
            constexpr FloatType roll_speed = 2.0f;
            FloatType r = (ks[SDL_SCANCODE_E] ? 1.0f : 0.0f) - (ks[SDL_SCANCODE_Q] ? 1.0f : 0.0f);
            if (r != 0.0f) {
                world.camera_.roll(r * roll_speed * dt);
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
            if (ks[SDL_SCANCODE_1]) renderer.set_mode(Renderer::Mode::WIREFRAME);
            else if (ks[SDL_SCANCODE_2]) renderer.set_mode(Renderer::Mode::RASTERIZED);
            else if (ks[SDL_SCANCODE_3]) renderer.set_mode(Renderer::Mode::RAYTRACED);
            else if (ks[SDL_SCANCODE_4]) renderer.set_mode(Renderer::Mode::DEPTH_OF_FIELD);
        });

    window.register_key({SDL_SCANCODE_G}, Window::Trigger::ANY_JUST_PRESSED,
        [&](const Window::KeyState&, float) { renderer.set_gamma((renderer.gamma() == 2.2f) ? 1.0f : 2.2f); });
    window.register_key({SDL_SCANCODE_P}, Window::Trigger::ANY_JUST_PRESSED,
        [&](const Window::KeyState&, float) {
            if (renderer.is_photon_map_ready()) {
                renderer.set_caustics_enabled(!renderer.caustics_enabled());
                std::cout << "Caustics (photon mapping): "
                          << (renderer.caustics_enabled() ? "ENABLED" : "DISABLED") << std::endl;
                renderer.reset_accumulation();
            } else {
                std::cout << "Photon map not ready yet, please wait..." << std::endl;
            }
        });
    
    // Screenshot save (Ctrl+S)
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
    
    // Video recording toggle (Ctrl+Shift+S)
    window.register_key({SDL_SCANCODE_LCTRL, SDL_SCANCODE_LSHIFT, SDL_SCANCODE_S}, Window::Trigger::ALL_JUST_PRESSED,
        [&](const Window::KeyState&, float) {
            video_recorder.toggle_recording();
            renderer.video_export_mode_ = video_recorder.is_recording();
        });
    
    window.register_key({SDL_SCANCODE_RCTRL, SDL_SCANCODE_RSHIFT, SDL_SCANCODE_S}, Window::Trigger::ALL_JUST_PRESSED,
        [&](const Window::KeyState&, float) {
            video_recorder.toggle_recording();
            renderer.video_export_mode_ = video_recorder.is_recording();
        });

    // Mouse look (left button drag)
    window.register_mouse(SDL_BUTTON_LEFT, Window::Trigger::ANY_PRESSED,
        [&](int xrel, int yrel, float dt) {
            if (xrel == 0 && yrel == 0) return;
            constexpr FloatType MOUSE_SENSITIVITY = 0.002f;
            FloatType dx0 = -static_cast<FloatType>(xrel) * MOUSE_SENSITIVITY;
            FloatType dy0 = static_cast<FloatType>(yrel) * MOUSE_SENSITIVITY;
            FloatType roll = world.camera_.roll_;
            FloatType c = std::cos(roll);
            FloatType s = std::sin(roll);
            FloatType d_yaw = dx0 * c + dy0 * s;
            FloatType d_pitch = -dx0 * s + dy0 * c;
            world.camera_.rotate(d_yaw, d_pitch);
            renderer.reset_accumulation();
        });

    auto last_print = std::chrono::steady_clock::now();
    std::size_t fps = 0;
    
    while (true) {
        
        // Advance any orbit animation and render the frame
        world.camera_.orbiting();
        renderer.video_export_mode_ = video_recorder.is_recording();
        renderer.render();

        // Process input; exit on window close or ESC
        if (!window.process_events()) break;

        // Capture a frame if recording
        if (video_recorder.is_recording()) video_recorder.capture_frame();

        // Present backbuffer
        window.update();
        
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
