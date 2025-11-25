#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include "SDL.h"

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

    using KeyState = std::array<Uint8, SDL_NUM_SCANCODES>;
    using KeyHandler = std::function<void(const KeyState&)>;
    using MouseHandler = std::function<void(int, int)>;

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
    std::array<Uint8, SDL_NUM_SCANCODES> keys_this_frame{};
    std::array<Uint8, SDL_NUM_SCANCODES> keys_last_frame{};
    std::unordered_set<SDL_Scancode> keys_updated_this_frame;
    std::unordered_set<Uint8> mouse_buttons_this_frame;
    std::unordered_set<Uint8> mouse_buttons_last_frame;
    std::unordered_set<Uint8> mouse_buttons_updated_this_frame;
    int mouse_xrel_sum = 0;
    int mouse_yrel_sum = 0;
    bool mouse_motion_this_frame = false;
    
    // Event handling
    struct KeyBinding {
        std::unordered_set<SDL_Scancode> keys;
        Trigger trigger;
        KeyHandler handler;
        size_t id;
    };
    std::vector<KeyBinding> key_bindings;
    struct MouseBinding {
        Uint8 button;
        Trigger trigger;
        MouseHandler handler;
        bool first_motion;
    };
    std::vector<MouseBinding> mouse_bindings;
    size_t next_event_id = 0;
    
    // Helper methods
    bool check_key_trigger(const KeyBinding& binding) const;
    void process_key_bindings();
    void process_mouse_bindings();
    void update_keyboard_state();
    bool has_modifier_keys() const;

public:
    Window() noexcept = delete;
    Window(int w, int h, bool fullscreen = false) noexcept;
    ~Window();
    
    // Disable copy, enable move
    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;
    Window(Window&&) noexcept = default;
    Window& operator=(Window&&) noexcept = default;
    
    // Event registration
    void register_key(const std::unordered_set<SDL_Scancode>& keys, Trigger trigger, KeyHandler handler);
    void register_mouse(Uint8 button, Trigger trigger, MouseHandler handler);
    
    // Main loop methods
    bool process_events(); // Returns false if should quit
    void update();         // Process event bindings based on current keyboard state
    void render();
    
    // Pixel manipulation
    void set_pixel_colour(size_t x, size_t y, uint32_t colour) noexcept;
    uint32_t get_pixel_colour(size_t x, size_t y) const noexcept;
    void clear_pixels() noexcept;
    const std::vector<uint32_t>& get_pixel_buffer() const noexcept;
    
    // File operations
    void save_ppm(const std::string& filename) const;
    void save_bmp(const std::string& filename) const;
    
    // Getters
    size_t get_width() const noexcept { return width; }
    size_t get_height() const noexcept { return height; }
    
    // Keyboard state queries
    bool is_key_down(SDL_Scancode key) const;
    bool is_key_just_pressed(SDL_Scancode key) const;
    bool is_key_just_released(SDL_Scancode key) const;
    
    // Clean exit
    void exit_cleanly() noexcept;
};

void printMessageAndQuit(const std::string& message, const char* error = nullptr) noexcept;
