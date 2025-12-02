#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "raytracer.hpp"
#include "renderer.hpp"
#include "utils.hpp"
#include "video_recorder.hpp"
#include "window.hpp"
#include "world.hpp"

constexpr std::size_t WindowWidth = 640;
constexpr std::size_t WindowHeight = 480;

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <scene1.obj> [<scene2.obj> ...]" << std::endl;
        return EXIT_FAILURE;
    }

    // Load world assets from CLI arguments
    World world = {std::vector<std::string>(argv + 1, argv + argc)};

    // Initialize output window and rendering engines
    Window window(WindowWidth, WindowHeight, false);
    Renderer renderer(world, window);
    VideoRecorder video_recorder(window.get_pixel_buffer(), WindowWidth, WindowHeight);

    // Centralized input callbacks: movement and UI toggles
    window.register_key(
        {SDL_SCANCODE_W,
         SDL_SCANCODE_S,
         SDL_SCANCODE_A,
         SDL_SCANCODE_D,
         SDL_SCANCODE_SPACE,
         SDL_SCANCODE_C},
        Window::Trigger::ANY_PRESSED_NO_MODIFIER,
        [&](const Window::KeyState& ks, float dt) {
            constexpr FloatType move_step = 3.0f;
            constexpr FloatType video_fixed_dt = 1.0f / 30.0f;  // Fixed step for video recording
            FloatType fwd = (ks[SDL_SCANCODE_W] ? 1.0f : 0.0f) - (ks[SDL_SCANCODE_S] ? 1.0f : 0.0f);
            FloatType right =
                (ks[SDL_SCANCODE_D] ? 1.0f : 0.0f) - (ks[SDL_SCANCODE_A] ? 1.0f : 0.0f);
            FloatType up =
                (ks[SDL_SCANCODE_SPACE] ? 1.0f : 0.0f) - (ks[SDL_SCANCODE_C] ? 1.0f : 0.0f);
            if (fwd != 0.0f || right != 0.0f || up != 0.0f) {
                FloatType effective_dt = video_recorder.is_recording() ? video_fixed_dt : dt;
                world.camera_.move(fwd * move_step, right * move_step, up * move_step, effective_dt);
                renderer.reset_accumulation();
            }
        }
    );

    window.register_key(
        {SDL_SCANCODE_Q, SDL_SCANCODE_E},
        Window::Trigger::ANY_PRESSED_NO_MODIFIER,
        [&](const Window::KeyState& ks, float dt) {
            constexpr FloatType roll_speed = 2.0f;
            constexpr FloatType video_fixed_dt = 1.0f / 30.0f;  // Fixed step for video recording
            FloatType r = (ks[SDL_SCANCODE_E] ? 1.0f : 0.0f) - (ks[SDL_SCANCODE_Q] ? 1.0f : 0.0f);
            if (r != 0.0f) {
                FloatType effective_dt = video_recorder.is_recording() ? video_fixed_dt : dt;
                world.camera_.roll(r * roll_speed * effective_dt);
                renderer.reset_accumulation();
            }
        }
    );

    window.register_key(
        {SDL_SCANCODE_O},
        Window::Trigger::ANY_JUST_PRESSED,
        [&](const Window::KeyState&, float) {
            if (!world.camera_.is_orbiting()) {
                world.camera_.start_orbiting();
            } else {
                world.camera_.stop_orbiting();
            }
            renderer.reset_accumulation();
        }
    );

    window.register_key(
        {SDL_SCANCODE_1, SDL_SCANCODE_2, SDL_SCANCODE_3, SDL_SCANCODE_4, SDL_SCANCODE_0},
        Window::Trigger::ANY_JUST_PRESSED,
        [&](const Window::KeyState& ks, float) {
            std::string_view mode_name;
            if (ks[SDL_SCANCODE_1]) {
                renderer.set_mode(Renderer::Mode::WIREFRAME);
                mode_name = "WIREFRAME";
            } else if (ks[SDL_SCANCODE_2]) {
                renderer.set_mode(Renderer::Mode::RASTERIZED);
                mode_name = "RASTERIZED";
            } else if (ks[SDL_SCANCODE_3]) {
                renderer.set_mode(Renderer::Mode::RAYTRACED);
                mode_name = "RAYTRACED";
            } else if (ks[SDL_SCANCODE_4]) {
                renderer.set_mode(Renderer::Mode::DEPTH_OF_FIELD);
                mode_name = "DEPTH_OF_FIELD";
            } else if (ks[SDL_SCANCODE_0]) {
                renderer.set_mode(Renderer::Mode::PHOTON_VISUALIZATION);
                mode_name = "PHOTON_VISUALIZATION";
            }
            renderer.reset_accumulation();
            std::cout << std::format("[Renderer] Mode set to {}\n", mode_name);
        }
    );

    window.register_key(
        {SDL_SCANCODE_G},
        Window::Trigger::ANY_JUST_PRESSED,
        [&](const Window::KeyState&, float) {
            renderer.set_gamma((renderer.gamma() == 2.2f) ? 1.0f : 2.2f);
        }
    );
    window.register_key(
        {SDL_SCANCODE_P},
        Window::Trigger::ANY_JUST_PRESSED,
        [&](const Window::KeyState&, float) {
            if (renderer.is_photon_map_ready()) {
                renderer.set_caustics_enabled(!renderer.caustics_enabled());
                std::cout << std::format(
                    "[PhotonMap] Caustics {}\n",
                    (renderer.caustics_enabled() ? "ENABLED" : "DISABLED")
                );
                renderer.reset_accumulation();
            } else {
                std::cout << "[PhotonMap] Caustics photon map is not ready yet" << std::endl;
            }
        }
    );

    // Aperture control (+/-)
    window.register_key(
        {SDL_SCANCODE_EQUALS, SDL_SCANCODE_MINUS},
        Window::Trigger::ANY_PRESSED_NO_MODIFIER,
        [&](const Window::KeyState& ks, float) {
            FloatType current_aperture = renderer.aperture_size();
            constexpr FloatType step = 0.01f;
            FloatType new_aperture = current_aperture;

            if (ks[SDL_SCANCODE_EQUALS]) {
                new_aperture += step;
            }
            if (ks[SDL_SCANCODE_MINUS]) {
                new_aperture -= step;
            }

            // Clamp values
            if (new_aperture < 0.0f) new_aperture = 0.0f;
            if (new_aperture > 2.0f) new_aperture = 2.0f;

            if (std::abs(new_aperture - current_aperture) > 0.0001f) {
                renderer.set_aperture_size(new_aperture);
                std::cout << std::format("[DepthOfField] Aperture Size: {:.2f}\n", new_aperture);
                renderer.reset_accumulation();
            }
        }
    );

    // Screenshot save (Ctrl+S)
    window.register_key(
        {SDL_SCANCODE_LCTRL, SDL_SCANCODE_S},
        Window::Trigger::ALL_JUST_PRESSED,
        [&](const Window::KeyState& ks, float) {
            window.save_ppm("screenshot.ppm");
            window.save_bmp("screenshot.bmp");
            std::cout << "[Screenshot] Saved as screenshot.ppm and screenshot.bmp\n";
        }
    );

    // Video recording toggle (Ctrl+R)
    window.register_key(
        {SDL_SCANCODE_LCTRL, SDL_SCANCODE_R},
        Window::Trigger::ALL_JUST_PRESSED,
        [&](const Window::KeyState&, float) {
            video_recorder.toggle_recording();
            renderer.set_video_export_mode(video_recorder.is_recording());
        }
    );

    // Mouse look (left button drag)
    window.register_mouse(
        SDL_BUTTON_LEFT,
        Window::Trigger::ANY_PRESSED,
        [&](int xrel, int yrel, float dt) {
            if (xrel == 0 && yrel == 0) return;
            constexpr FloatType MOUSE_SENSITIVITY = 0.002f;
            FloatType dx0 = -static_cast<FloatType>(xrel) * MOUSE_SENSITIVITY;
            FloatType dy0 = static_cast<FloatType>(yrel) * MOUSE_SENSITIVITY;
            FloatType roll = world.camera_.roll();
            FloatType c = std::cos(roll);
            FloatType s = std::sin(roll);
            FloatType d_yaw = dx0 * c + dy0 * s;
            FloatType d_pitch = -dx0 * s + dy0 * c;
            world.camera_.rotate(d_yaw, d_pitch);
            renderer.reset_accumulation();
        }
    );

    // Keyboard look (arrow keys)
    window.register_key(
        {SDL_SCANCODE_UP, SDL_SCANCODE_DOWN, SDL_SCANCODE_LEFT, SDL_SCANCODE_RIGHT},
        Window::Trigger::ANY_PRESSED_NO_MODIFIER,
        [&](const Window::KeyState& ks, float dt) {
            constexpr FloatType ROTATE_SPEED = 0.2f;
            constexpr FloatType video_fixed_dt = 1.0f / 60.0f;  // Fixed step for video recording
            FloatType dx0 = (ks[SDL_SCANCODE_RIGHT] ? 1.0f : 0.0f) - (ks[SDL_SCANCODE_LEFT] ? 1.0f : 0.0f);
            FloatType dy0 = (ks[SDL_SCANCODE_UP] ? 1.0f : 0.0f) - (ks[SDL_SCANCODE_DOWN] ? 1.0f : 0.0f);
            if (dx0 != 0.0f || dy0 != 0.0f) {
                FloatType roll = world.camera_.roll();
                FloatType c = std::cos(roll);
                FloatType s = std::sin(roll);
                FloatType effective_dt = video_recorder.is_recording() ? video_fixed_dt : dt;
                FloatType d_yaw = (dx0 * c + dy0 * s) * ROTATE_SPEED * effective_dt;
                FloatType d_pitch = (-dx0 * s + dy0 * c) * ROTATE_SPEED * effective_dt;
                world.camera_.rotate(d_yaw, d_pitch);
                renderer.reset_accumulation();
            }
        }
    );

    // Focal distance control via scroll
    window.register_scroll([&](int y_offset) {
        if (y_offset == 0) return;
        FloatType current_fd = renderer.focal_distance();

        // Handle 0 or very small case by forcing a base value
        if (current_fd < 1.0f) current_fd = 1.0f;

        // Logarithmic adjustment: multiply/divide by factor
        constexpr FloatType factor = 1.1f;
        FloatType new_fd = current_fd * std::pow(factor, static_cast<FloatType>(y_offset));

        // Boundary safety
        if (new_fd < 1.0f) new_fd = 1.0f;

        if (std::abs(new_fd - current_fd) > 0.0001f) {
            renderer.set_focal_distance(new_fd);
            std::cout << std::format("[DepthOfField] Focal Distance: {:.2f}\n", new_fd);
            renderer.reset_accumulation();
        }
    });

    while (true) {
        // Advance any orbit animation and render the frame
        world.camera_.orbiting();
        renderer.set_video_export_mode(video_recorder.is_recording());
        renderer.render();

        // Process input; exit on window close or ESC
        if (!window.process_events()) break;

        // Capture a frame if recording
        if (video_recorder.is_recording()) video_recorder.capture_frame();

        // Present backbuffer
        window.update();
    }
}
