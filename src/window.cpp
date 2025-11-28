#include "window.hpp"

#include <algorithm>

Window::Window(int w, int h, bool fullscreen) noexcept : width_(w), height_(h), pixel_buffer_(w * h) {
    // Initialize SDL video subsystem
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        std::cerr << "Could not initialise SDL: " << SDL_GetError() << std::endl;
        std::terminate();
    }

    // Create window surface (optional fullscreen)
    uint32_t flags = SDL_WINDOW_OPENGL;
    if (fullscreen) flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;

    int anywhere = SDL_WINDOWPOS_UNDEFINED;
    window_ = SDL_CreateWindow("COMS30020", anywhere, anywhere, width_, height_, flags);
    if (!window_) {
        std::cerr << "Could not set video mode: " << SDL_GetError() << std::endl;
        std::terminate();
    }

    // Renderer bound to window (software for portability)
    flags = SDL_RENDERER_SOFTWARE;
    // Alternative: SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC for hardware acceleration
    renderer_ = SDL_CreateRenderer(window_, -1, flags);
    if (!renderer_) {
        std::cerr << "Could not create renderer: " << SDL_GetError() << std::endl;
        std::terminate();
    }

    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");
    SDL_RenderSetLogicalSize(renderer_, width_, height_);

    // Backbuffer texture receiving ARGB pixels
    int pixel_format = SDL_PIXELFORMAT_ARGB8888;
    texture_ = SDL_CreateTexture(renderer_, pixel_format, SDL_TEXTUREACCESS_STATIC, width_, height_);
    if (!texture_) {
        std::cerr << "Could not allocate texture: " << SDL_GetError() << std::endl;
        std::terminate();
    }
}

Window::~Window() {
    if (texture_) SDL_DestroyTexture(texture_);
    if (renderer_) SDL_DestroyRenderer(renderer_);
    if (window_) SDL_DestroyWindow(window_);
    SDL_Quit();
}

void Window::register_key(const std::unordered_set<SDL_Scancode>& keys, Trigger trigger, KeyHandler handler) noexcept {
    auto now = std::chrono::steady_clock::now();
    key_bindings_.push_back(KeyBinding{
        .keys = keys,
        .trigger = trigger,
        .handler = handler,
        .id = next_event_id_++,
        .last_time = now,
        .time_initialized = false});
}

void Window::register_mouse(Uint8 button, Trigger trigger, MouseHandler handler) noexcept {
    auto now = std::chrono::steady_clock::now();
    mouse_bindings_.push_back(MouseBinding{
        .button = button,
        .trigger = trigger,
        .handler = handler,
        .first_motion = true,
        .last_time = now,
        .time_initialized = false});
}

bool Window::process_events() noexcept {
    // Reset per-frame input state, then drain SDL event queue
    SDL_Event event;
    keys_updated_this_frame_.clear();
    keys_pressed_this_frame_.clear();
    mouse_buttons_last_frame_ = mouse_buttons_this_frame_;
    mouse_buttons_updated_this_frame_.clear();
    mouse_motion_this_frame_ = false;
    mouse_xrel_ = 0;
    mouse_yrel_ = 0;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT || (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE) ||
            (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)) {
            return false;
        }

        // Keyboard transitions
        if (event.type == SDL_KEYDOWN && !event.key.repeat) {
            SDL_Scancode sc = event.key.keysym.scancode;
            if (sc >= 0 && sc < SDL_NUM_SCANCODES) {
                keys_this_frame_[sc] = 1;
                keys_updated_this_frame_.insert(sc);
                keys_pressed_this_frame_.insert(sc);
            }
        }
        if (event.type == SDL_KEYUP) {
            SDL_Scancode sc = event.key.keysym.scancode;
            if (sc >= 0 && sc < SDL_NUM_SCANCODES) {
                keys_this_frame_[sc] = 0;
                keys_updated_this_frame_.insert(sc);
            }
        }

        // Mouse button transitions
        if (event.type == SDL_MOUSEBUTTONDOWN) {
            Uint8 b = event.button.button;
            mouse_buttons_this_frame_.insert(b);
            mouse_buttons_updated_this_frame_.insert(b);
            for (auto& mb : mouse_bindings_) {
                if (mb.button == b) mb.first_motion = true;
            }
        }
        if (event.type == SDL_MOUSEBUTTONUP) {
            Uint8 b = event.button.button;
            mouse_buttons_this_frame_.erase(b);
            mouse_buttons_updated_this_frame_.insert(b);
        }

        // Accumulate relative motion for bound handlers
        if (event.type == SDL_MOUSEMOTION) {
            mouse_motion_this_frame_ = true;
            mouse_xrel_ += event.motion.xrel;
            mouse_yrel_ += event.motion.yrel;
        }
    }

    // Dispatch registered input handlers
    process_key_bindings();
    process_mouse_bindings();
    return true;
}

void Window::update_keyboard_state() noexcept {
    const Uint8* keystate = SDL_GetKeyboardState(nullptr);
    std::copy(keystate, keystate + SDL_NUM_SCANCODES, keys_this_frame_.begin());
}

void Window::process_key_bindings() noexcept {
    auto now = std::chrono::steady_clock::now();
    for (auto& binding : key_bindings_) {
        if (IsPressedMode(binding.trigger)) {
            bool any_down = true;
            if (binding.trigger == Trigger::ANY_DOWN || binding.trigger == Trigger::ANY_PRESSED ||
                binding.trigger == Trigger::ANY_PRESSED_NO_MODIFIER) {
                any_down = false;
                for (auto k : binding.keys) {
                    if (keys_this_frame_[k]) {
                        any_down = true;
                        break;
                    }
                }
            } else if (binding.trigger == Trigger::ALL_DOWN || binding.trigger == Trigger::ALL_PRESSED) {
                any_down = true;
                for (auto k : binding.keys) {
                    if (!keys_this_frame_[k]) {
                        any_down = false;
                        break;
                    }
                }
            }
            if (binding.trigger == Trigger::ANY_PRESSED_NO_MODIFIER && has_modifier_keys()) any_down = false;

            bool just_pressed = false;
            for (auto k : binding.keys) {
                if (keys_pressed_this_frame_.count(k)) {
                    just_pressed = true;
                    break;
                }
            }
            if (any_down) {
                if (just_pressed) {
                    binding.last_time = now;
                    binding.time_initialized = true;
                    continue;  // do not call on initial press
                }
                if (!binding.time_initialized) {
                    binding.last_time = now;
                    binding.time_initialized = true;
                }
                float dt = std::chrono::duration<float>(now - binding.last_time).count();
                binding.handler(keys_this_frame_, dt);
                binding.last_time = now;
            }
        } else {
            if (check_key_trigger(binding)) {
                binding.handler(keys_this_frame_, 0.0f);
            }
        }
    }
}

bool Window::has_modifier_keys() const noexcept {
    return keys_this_frame_[SDL_SCANCODE_LCTRL] || keys_this_frame_[SDL_SCANCODE_RCTRL] ||
           keys_this_frame_[SDL_SCANCODE_LSHIFT] || keys_this_frame_[SDL_SCANCODE_RSHIFT] ||
           keys_this_frame_[SDL_SCANCODE_LALT] || keys_this_frame_[SDL_SCANCODE_RALT];
}

bool Window::check_key_trigger(const KeyBinding& binding) const noexcept {
    if (binding.keys.empty()) return false;

    switch (binding.trigger) {
    case Trigger::ANY_PRESSED: {
        for (auto key : binding.keys) {
            if (keys_updated_this_frame_.count(key) && keys_this_frame_[key]) return true;
        }
        return false;
    }

    case Trigger::ALL_PRESSED: {
        for (auto key : binding.keys) {
            if (!keys_this_frame_[key]) return false;
        }
        return true;
    }

    case Trigger::ANY_RELEASED: {
        for (auto key : binding.keys) {
            if (keys_updated_this_frame_.count(key) && !keys_this_frame_[key]) return true;
        }
        return false;
    }

    case Trigger::ALL_RELEASED: {
        for (auto key : binding.keys) {
            if (!(keys_updated_this_frame_.count(key) && !keys_this_frame_[key])) {
                return false;
            }
        }
        return true;
    }

    case Trigger::ANY_DOWN: {
        for (auto key : binding.keys) {
            if (keys_this_frame_[key]) return true;
        }
        return false;
    }

    case Trigger::ALL_DOWN: {
        for (auto key : binding.keys) {
            if (!keys_this_frame_[key]) return false;
        }
        return true;
    }

    case Trigger::ANY_JUST_PRESSED: {
        for (auto key : binding.keys) {
            if (keys_pressed_this_frame_.count(key)) return true;
        }
        return false;
    }

    case Trigger::ALL_JUST_PRESSED: {
        bool exists_pressed_this_frame = false;
        for (auto key : binding.keys) {
            if (!(keys_this_frame_[key] || keys_pressed_this_frame_.count(key))) return false;
            if (keys_pressed_this_frame_.count(key)) exists_pressed_this_frame = true;
        }
        return exists_pressed_this_frame;
    }

    case Trigger::ANY_PRESSED_NO_MODIFIER: {
        if (has_modifier_keys()) return false;
        for (auto key : binding.keys) {
            if (keys_this_frame_[key]) return true;
        }
        return false;
    }
    }

    return false;
}

void Window::process_mouse_bindings() noexcept {
    auto now = std::chrono::steady_clock::now();
    for (auto& binding : mouse_bindings_) {
        bool down = mouse_buttons_this_frame_.count(binding.button) > 0;
        bool just_pressed = mouse_buttons_updated_this_frame_.count(binding.button) > 0 && down;
        switch (binding.trigger) {
        case Trigger::ANY_JUST_PRESSED: {
            if (just_pressed) binding.handler(0, 0, 0.0f);
            break;
        }
        case Trigger::ANY_PRESSED: {
            if (down) {
                if (just_pressed) {
                    binding.last_time = now;
                    binding.time_initialized = true;
                    binding.first_motion = false;
                } else if (mouse_motion_this_frame_) {
                    if (!binding.time_initialized) {
                        binding.last_time = now;
                        binding.time_initialized = true;
                    }
                    float dt = std::chrono::duration<float>(now - binding.last_time).count();
                    binding.handler(mouse_xrel_, mouse_yrel_, dt);
                    binding.last_time = now;
                }
            }
            break;
        }
        default:
            break;
        }
    }
    mouse_motion_this_frame_ = false;
    mouse_xrel_ = 0;
    mouse_yrel_ = 0;
}

void Window::update() noexcept {
    // Upload ARGB backbuffer to texture and present; then clear for next frame
    SDL_UpdateTexture(texture_, nullptr, pixel_buffer_.data(), width_ * sizeof(uint32_t));
    SDL_RenderClear(renderer_);
    SDL_RenderCopy(renderer_, texture_, nullptr, nullptr);
    SDL_RenderPresent(renderer_);
    clear_pixels();
}

std::uint32_t& Window::operator[](const std::pair<int, int>& xy) noexcept {
    std::size_t x = static_cast<std::size_t>(xy.first);
    std::size_t y = static_cast<std::size_t>(xy.second);
    return pixel_buffer_[y * width_ + x];
}

std::uint32_t Window::operator[](const std::pair<int, int>& xy) const noexcept {
    std::size_t x = static_cast<std::size_t>(xy.first);
    std::size_t y = static_cast<std::size_t>(xy.second);
    return pixel_buffer_[y * width_ + x];
}

void Window::clear_pixels() noexcept { std::fill(pixel_buffer_.begin(), pixel_buffer_.end(), 0); }

const std::vector<uint32_t>& Window::get_pixel_buffer() const noexcept { return pixel_buffer_; }

void Window::save_ppm(const std::string& filename) const {
    std::ofstream output_stream(filename, std::ofstream::out);
    output_stream << "P6\n";
    output_stream << width_ << " " << height_ << "\n";
    output_stream << "255\n";

    for (size_t i = 0; i < width_ * height_; i++) {
        std::array<char, 3> rgb{
            {static_cast<char>((pixel_buffer_[i] >> 16) & 0xFF),
             static_cast<char>((pixel_buffer_[i] >> 8) & 0xFF),
             static_cast<char>((pixel_buffer_[i] >> 0) & 0xFF)}};
        output_stream.write(rgb.data(), 3);
    }
    output_stream.close();
}

void Window::save_bmp(const std::string& filename) const {
    auto surface = SDL_CreateRGBSurfaceFrom(
        (void*)pixel_buffer_.data(),
        width_,
        height_,
        32,
        width_ * sizeof(uint32_t),
        0xFF << 16,
        0xFF << 8,
        0xFF << 0,
        0xFF << 24);
    SDL_SaveBMP(surface, filename.c_str());
    SDL_FreeSurface(surface);
}

bool Window::is_key_pressed(SDL_Scancode key) const noexcept { return keys_this_frame_[key] != 0; }

bool Window::is_key_just_pressed(SDL_Scancode key) const noexcept { return keys_pressed_this_frame_.count(key); }

bool Window::is_key_just_released(SDL_Scancode key) const noexcept {
    return keys_updated_this_frame_.count(key) && !keys_this_frame_[key];
}
