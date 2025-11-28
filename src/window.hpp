#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
#include "SDL.h"
#include "world.hpp"

class Window {
public:
    // Event trigger types
    enum class Trigger {
        ANY_PRESSED,
        ALL_PRESSED,
        ANY_RELEASED,
        ALL_RELEASED,
        ANY_DOWN,
        ALL_DOWN,
        ANY_JUST_PRESSED,
        ALL_JUST_PRESSED,
        ANY_PRESSED_NO_MODIFIER
    };
    using KeyState = std::array<bool, SDL_NUM_SCANCODES>;
    using KeyHandler = std::function<void(const KeyState&, float)>;
    using MouseHandler = std::function<void(int, int, float)>;

private:
    // SDL components
    SDL_Window* window_;
    SDL_Renderer* renderer_;
    SDL_Texture* texture_;
    
    // Window properties
    size_t width_;
    size_t height_;
    std::vector<uint32_t> pixel_buffer_;
    
    // Input management
    KeyState keys_this_frame_{};
    std::unordered_set<SDL_Scancode> keys_updated_this_frame_;
    std::unordered_set<SDL_Scancode> keys_pressed_this_frame_;
    std::unordered_set<Uint8> mouse_buttons_this_frame_;
    std::unordered_set<Uint8> mouse_buttons_last_frame_;
    std::unordered_set<Uint8> mouse_buttons_updated_this_frame_;
    int mouse_xrel_ = 0;
    int mouse_yrel_ = 0;
    bool mouse_motion_this_frame_ = false;
    
    // Event handling
    struct KeyBinding {
        std::unordered_set<SDL_Scancode> keys;
        Trigger trigger;
        KeyHandler handler;
        size_t id;
        std::chrono::steady_clock::time_point last_time;
        bool time_initialized = false;
    };
    std::vector<KeyBinding> key_bindings_;
    struct MouseBinding {
        Uint8 button;
        Trigger trigger;
        MouseHandler handler;
        bool first_motion;
        std::chrono::steady_clock::time_point last_time;
        bool time_initialized = false;
    };
    std::vector<MouseBinding> mouse_bindings_;
    size_t next_event_id_ = 0;
    
    // Helper methods
    bool check_key_trigger(const KeyBinding& binding) const noexcept;
    void process_key_bindings() noexcept;
    void process_mouse_bindings() noexcept;
    void update_keyboard_state() noexcept;
    bool has_modifier_keys() const noexcept;
    static constexpr bool IsPressedMode(Trigger t) noexcept {
        return t == Trigger::ANY_PRESSED || t == Trigger::ALL_PRESSED ||
               t == Trigger::ANY_DOWN || t == Trigger::ALL_DOWN ||
               t == Trigger::ANY_PRESSED_NO_MODIFIER;
    }

public:
    Window() noexcept = delete;
    explicit Window(int w, int h, bool fullscreen = false) noexcept;
    ~Window();
    
    // Event registration
    void register_key(const std::unordered_set<SDL_Scancode>& keys, Trigger trigger, KeyHandler handler) noexcept;
    void register_mouse(Uint8 button, Trigger trigger, MouseHandler handler) noexcept;
    
    // Main loop methods
    bool process_events() noexcept;
    void update() noexcept;
    
    // Pixel manipulation
    void clear_pixels() noexcept;
    std::uint32_t& operator[](const std::pair<int, int>& xy) noexcept;
    std::uint32_t operator[](const std::pair<int, int>& xy) const noexcept;
    const std::vector<uint32_t>& get_pixel_buffer() const noexcept;
    
    // File operations
    void save_ppm(const std::string& filename) const;
    void save_bmp(const std::string& filename) const;
    
    // Getters
    size_t get_width() const noexcept { return width_; }
    size_t get_height() const noexcept { return height_; }
    
    // Keyboard state queries
    bool is_key_pressed(SDL_Scancode key) const noexcept;
    bool is_key_just_pressed(SDL_Scancode key) const noexcept;
    bool is_key_just_released(SDL_Scancode key) const noexcept;
};
