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
        ANY_PRESSED,              // At least one key is pressed this frame (fires once per press)
        ALL_PRESSED,              // All keys are pressed this frame
        ANY_RELEASED,             // At least one key was released this frame
        ALL_RELEASED,             // All keys were released this frame
        ANY_DOWN,                 // At least one key is held down (continuous)
        ALL_DOWN,                 // All keys are held down (continuous)
        ANY_JUST_PRESSED,         // At least one key was just pressed (not pressed last frame, pressed this frame)
        ALL_JUST_PRESSED,         // All keys were just pressed
        ANY_PRESSED_NO_MODIFIER   // At least one key is pressed with no modifier keys (Ctrl/Shift/Alt)
    };

    using KeyState = std::array<bool, SDL_NUM_SCANCODES>;
    using KeyHandler = std::function<void(const KeyState&, float)>;
    using MouseHandler = std::function<void(int, int, float)>;

private:
    // SDL components
    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_Texture* texture;
    
    // Window properties
    size_t width;
    size_t height;
    std::vector<uint32_t> pixel_buffer;
    
    // Input management
    KeyState keys_this_frame{};
    KeyState keys_last_frame{};
    std::unordered_set<SDL_Scancode> keys_updated_this_frame;
    std::unordered_set<Uint8> mouse_buttons_this_frame;
    std::unordered_set<Uint8> mouse_buttons_last_frame;
    std::unordered_set<Uint8> mouse_buttons_updated_this_frame;
    int mouse_xrel = 0;
    int mouse_yrel = 0;
    bool mouse_motion_this_frame = false;
    
    // Event handling
    struct KeyBinding {
        std::unordered_set<SDL_Scancode> keys;
        Trigger trigger;
        KeyHandler handler;
        size_t id;
        std::chrono::steady_clock::time_point last_time;
        bool time_initialized = false;
    };
    std::vector<KeyBinding> key_bindings;
    struct MouseBinding {
        Uint8 button;
        Trigger trigger;
        MouseHandler handler;
        bool first_motion;
        std::chrono::steady_clock::time_point last_time;
        bool time_initialized = false;
    };
    std::vector<MouseBinding> mouse_bindings;
    size_t next_event_id = 0;
    
    // Helper methods
    bool check_key_trigger(const KeyBinding& binding) const;
    void process_key_bindings();
    void process_mouse_bindings();
    void update_keyboard_state();
    bool has_modifier_keys() const;
    static bool is_pressed_mode(Trigger t) noexcept {
        return t == Trigger::ANY_PRESSED || t == Trigger::ALL_PRESSED ||
               t == Trigger::ANY_DOWN || t == Trigger::ALL_DOWN ||
               t == Trigger::ANY_PRESSED_NO_MODIFIER;
    }

public:
    Window() noexcept = delete;
    Window(int w, int h, bool fullscreen = false) noexcept;
    ~Window();
    
    // Event registration
    void register_key(const std::unordered_set<SDL_Scancode>& keys, Trigger trigger, KeyHandler handler);
    void register_mouse(Uint8 button, Trigger trigger, MouseHandler handler);
    
    // Main loop methods
    bool process_events(); // Returns false if should quit; also dispatches bindings
    void update();         // Present the frame and clear buffer for next frame (end of loop)
    
    // Pixel manipulation
    void clear_pixels() noexcept;
    std::uint32_t& operator[](const std::pair<int, int>& xy) noexcept;
    std::uint32_t operator[](const std::pair<int, int>& xy) const noexcept;
    const std::vector<uint32_t>& get_pixel_buffer() const noexcept;
    
    // File operations
    void save_ppm(const std::string& filename) const;
    void save_bmp(const std::string& filename) const;
    
    // Getters
    size_t get_width() const noexcept { return width; }
    size_t get_height() const noexcept { return height; }
    
    // Keyboard state queries
    bool is_key_pressed(SDL_Scancode key) const;
    bool is_key_just_pressed(SDL_Scancode key) const;
    bool is_key_just_released(SDL_Scancode key) const;
};
