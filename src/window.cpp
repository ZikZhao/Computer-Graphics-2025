#include "window.hpp"
#include <algorithm>

Window::Window(int w, int h, bool fullscreen) noexcept 
    : width(w), height(h), pixel_buffer(w * h) {
    
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        printMessageAndQuit("Could not initialise SDL: ", SDL_GetError());
    }
    
    // Create window
    uint32_t flags = SDL_WINDOW_OPENGL;
    if (fullscreen) flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;
    
    int anywhere = SDL_WINDOWPOS_UNDEFINED;
    window = SDL_CreateWindow("COMS30020", anywhere, anywhere, width, height, flags);
    if (!window) {
        printMessageAndQuit("Could not set video mode: ", SDL_GetError());
    }
    
    // Create renderer (software rendering for compatibility)
    flags = SDL_RENDERER_SOFTWARE;
    // Alternative: SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC for hardware acceleration
    renderer = SDL_CreateRenderer(window, -1, flags);
    if (!renderer) {
        printMessageAndQuit("Could not create renderer: ", SDL_GetError());
    }
    
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");
    SDL_RenderSetLogicalSize(renderer, width, height);
    
    // Create texture
    int pixel_format = SDL_PIXELFORMAT_ARGB8888;
    texture = SDL_CreateTexture(renderer, pixel_format, SDL_TEXTUREACCESS_STATIC, width, height);
    if (!texture) {
        printMessageAndQuit("Could not allocate texture: ", SDL_GetError());
    }
}

Window::~Window() {
    if (texture) SDL_DestroyTexture(texture);
    if (renderer) SDL_DestroyRenderer(renderer);
    if (window) SDL_DestroyWindow(window);
    SDL_Quit();
}

void Window::register_key(const std::unordered_set<SDL_Scancode>& keys, Trigger trigger, KeyHandler handler) {
    key_bindings.push_back({keys, trigger, handler, next_event_id++});
}

void Window::register_mouse(Uint8 button, Trigger trigger, MouseHandler handler) {
    mouse_bindings.push_back({button, trigger, handler, true});
}

bool Window::process_events() {
    SDL_Event event;
    keys_last_frame = keys_this_frame;
    keys_updated_this_frame.clear();
    mouse_buttons_last_frame = mouse_buttons_this_frame;
    mouse_buttons_updated_this_frame.clear();
    mouse_motion_this_frame = false;
    mouse_xrel_sum = 0;
    mouse_yrel_sum = 0;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT ||
            (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE) ||
            (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)) {
            return false;
        }
        if (event.type == SDL_KEYDOWN && !event.key.repeat) {
            SDL_Scancode sc = event.key.keysym.scancode;
            if (sc >= 0 && sc < SDL_NUM_SCANCODES) {
                keys_this_frame[sc] = 1;
                keys_updated_this_frame.insert(sc);
            }
        }
        if (event.type == SDL_KEYUP) {
            SDL_Scancode sc = event.key.keysym.scancode;
            if (sc >= 0 && sc < SDL_NUM_SCANCODES) {
                keys_this_frame[sc] = 0;
                keys_updated_this_frame.insert(sc);
            }
        }
        if (event.type == SDL_MOUSEBUTTONDOWN) {
            Uint8 b = event.button.button;
            mouse_buttons_this_frame.insert(b);
            mouse_buttons_updated_this_frame.insert(b);
        }
        if (event.type == SDL_MOUSEBUTTONUP) {
            Uint8 b = event.button.button;
            mouse_buttons_this_frame.erase(b);
            mouse_buttons_updated_this_frame.insert(b);
            for (auto& mb : mouse_bindings) {
                if (mb.button == b) mb.first_motion = true;
            }
        }
        if (event.type == SDL_MOUSEMOTION) {
            mouse_motion_this_frame = true;
            mouse_xrel_sum += event.motion.xrel;
            mouse_yrel_sum += event.motion.yrel;
        }
    }
    return true;
}

void Window::update_keyboard_state() {
    keys_last_frame = keys_this_frame;
    const Uint8* keystate = SDL_GetKeyboardState(nullptr);
    for (int i = 0; i < SDL_NUM_SCANCODES; ++i) {
        keys_this_frame[i] = keystate[i] ? 1 : 0;
    }
}

void Window::update() {
    process_key_bindings();
    process_mouse_bindings();
}

void Window::process_key_bindings() {
    for (const auto& binding : key_bindings) {
        if (check_key_trigger(binding)) {
            binding.handler(keys_this_frame);
        }
    }
}

bool Window::has_modifier_keys() const {
    return keys_this_frame[SDL_SCANCODE_LCTRL] ||
           keys_this_frame[SDL_SCANCODE_RCTRL] ||
           keys_this_frame[SDL_SCANCODE_LSHIFT] ||
           keys_this_frame[SDL_SCANCODE_RSHIFT] ||
           keys_this_frame[SDL_SCANCODE_LALT] ||
           keys_this_frame[SDL_SCANCODE_RALT];
}

bool Window::check_key_trigger(const KeyBinding& binding) const {
    if (binding.keys.empty()) return false;
    
    switch (binding.trigger) {
        case Trigger::ANY_PRESSED: {
            for (auto key : binding.keys) {
                if (keys_updated_this_frame.count(key) && keys_this_frame[key]) return true;
            }
            return false;
        }
        
        case Trigger::ALL_PRESSED: {
            for (auto key : binding.keys) {
                if (!keys_this_frame[key]) return false;
            }
            return true;
        }
        
        case Trigger::ANY_RELEASED: {
            for (auto key : binding.keys) {
                if (keys_updated_this_frame.count(key) && !keys_this_frame[key]) return true;
            }
            return false;
        }
        
        case Trigger::ALL_RELEASED: {
            for (auto key : binding.keys) {
                if (!(keys_updated_this_frame.count(key) && !keys_this_frame[key])) {
                    return false;
                }
            }
            return true;
        }
        
        case Trigger::ANY_DOWN: {
            for (auto key : binding.keys) {
                if (keys_this_frame[key]) return true;
            }
            return false;
        }
        
        case Trigger::ALL_DOWN: {
            for (auto key : binding.keys) {
                if (!keys_this_frame[key]) return false;
            }
            return true;
        }
        
        case Trigger::ANY_JUST_PRESSED: {
            for (auto key : binding.keys) {
                if (keys_updated_this_frame.count(key) && keys_this_frame[key]) return true;
            }
            return false;
        }
        
        case Trigger::ALL_JUST_PRESSED: {
            for (auto key : binding.keys) {
                if (!(keys_updated_this_frame.count(key) && keys_this_frame[key])) return false;
            }
            return true;
        }
        
        case Trigger::ANY_PRESSED_NO_MODIFIER: {
            if (has_modifier_keys()) return false;
            for (auto key : binding.keys) {
                if (keys_this_frame[key]) return true;
            }
            return false;
        }
    }
    
    return false;
}

void Window::process_mouse_bindings() {
    for (auto& binding : mouse_bindings) {
        bool down = mouse_buttons_this_frame.count(binding.button) > 0;
        bool just_pressed = mouse_buttons_updated_this_frame.count(binding.button) > 0 && down;
        switch (binding.trigger) {
            case Trigger::ANY_JUST_PRESSED: {
                if (just_pressed) {
                    binding.handler(0, 0);
                }
                break;
            }
            case Trigger::ANY_PRESSED: {
                if (down) {
                    if (binding.first_motion) {
                        binding.handler(0, 0);
                        binding.first_motion = false;
                    } else if (mouse_motion_this_frame) {
                        binding.handler(mouse_xrel_sum, mouse_yrel_sum);
                    }
                }
                break;
            }
            default: break;
        }
    }
    mouse_motion_this_frame = false;
    mouse_xrel_sum = 0;
    mouse_yrel_sum = 0;
}

void Window::render() {
    SDL_UpdateTexture(texture, nullptr, pixel_buffer.data(), width * sizeof(uint32_t));
    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, nullptr, nullptr);
    SDL_RenderPresent(renderer);
}

void Window::set_pixel_colour(size_t x, size_t y, uint32_t colour) noexcept {
    if (x >= width || y >= height) {
        std::cout << x << "," << y << " not on visible screen area" << std::endl;
    } else {
        pixel_buffer[y * width + x] = colour;
    }
}

uint32_t Window::get_pixel_colour(size_t x, size_t y) const noexcept {
    if (x >= width || y >= height) {
        std::cout << x << "," << y << " not on visible screen area" << std::endl;
        return 0;
    }
    return pixel_buffer[y * width + x];
}

void Window::clear_pixels() noexcept {
    std::fill(pixel_buffer.begin(), pixel_buffer.end(), 0);
}

const std::vector<uint32_t>& Window::get_pixel_buffer() const noexcept {
    return pixel_buffer;
}

void Window::save_ppm(const std::string& filename) const {
    std::ofstream output_stream(filename, std::ofstream::out);
    output_stream << "P6\n";
    output_stream << width << " " << height << "\n";
    output_stream << "255\n";
    
    for (size_t i = 0; i < width * height; i++) {
        std::array<char, 3> rgb {{
            static_cast<char>((pixel_buffer[i] >> 16) & 0xFF),
            static_cast<char>((pixel_buffer[i] >> 8) & 0xFF),
            static_cast<char>((pixel_buffer[i] >> 0) & 0xFF)
        }};
        output_stream.write(rgb.data(), 3);
    }
    output_stream.close();
}

void Window::save_bmp(const std::string& filename) const {
    auto surface = SDL_CreateRGBSurfaceFrom(
        (void*)pixel_buffer.data(), width, height, 32,
        width * sizeof(uint32_t),
        0xFF << 16, 0xFF << 8, 0xFF << 0, 0xFF << 24
    );
    SDL_SaveBMP(surface, filename.c_str());
    SDL_FreeSurface(surface);
}

bool Window::is_key_down(SDL_Scancode key) const {
    return keys_this_frame[key] != 0;
}

bool Window::is_key_just_pressed(SDL_Scancode key) const {
    return keys_updated_this_frame.count(key) && keys_this_frame[key];
}

bool Window::is_key_just_released(SDL_Scancode key) const {
    return keys_updated_this_frame.count(key) && !keys_this_frame[key];
}

void Window::exit_cleanly() noexcept {
    printMessageAndQuit("Exiting");
}

// printMessageAndQuit is provided by libs/sdw/DrawingWindow.cpp
