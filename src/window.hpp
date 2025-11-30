#pragma once

#include <array>
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "SDL.h"
#include "world.hpp"

/**
 * @brief SDL-backed window, input, and pixel backbuffer manager.
 */
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
    using ScrollHandler = std::function<void(int)>;

private:
    struct KeyBinding {
        std::unordered_set<SDL_Scancode> keys;
        Trigger trigger;
        KeyHandler handler;
        std::size_t id;
        std::chrono::steady_clock::time_point last_time;
        bool time_initialized = false;
    };

    struct MouseBinding {
        Uint8 button;
        Trigger trigger;
        MouseHandler handler;
        bool first_motion;
        std::chrono::steady_clock::time_point last_time;
        bool time_initialized = false;
    };

private:
    static constexpr bool IsPressedMode(Trigger t) noexcept {
        return t == Trigger::ANY_PRESSED || t == Trigger::ALL_PRESSED || t == Trigger::ANY_DOWN ||
               t == Trigger::ALL_DOWN || t == Trigger::ANY_PRESSED_NO_MODIFIER;
    }

private:
    // SDL components
    SDL_Window* window_;
    SDL_Renderer* renderer_;
    SDL_Texture* texture_;

    // Window properties
    std::size_t width_;
    std::size_t height_;
    std::vector<std::uint32_t> pixel_buffer_;

    // Input management
    KeyState keys_this_frame_{};
    std::unordered_set<SDL_Scancode> keys_updated_this_frame_;
    std::unordered_set<SDL_Scancode> keys_pressed_this_frame_;
    std::unordered_set<Uint8> mouse_buttons_this_frame_;
    std::unordered_set<Uint8> mouse_buttons_last_frame_;
    std::unordered_set<Uint8> mouse_buttons_updated_this_frame_;
    int mouse_xrel_ = 0;
    int mouse_yrel_ = 0;
    int mouse_scroll_ = 0;

    // Event handling
    std::vector<KeyBinding> key_bindings_;
    std::vector<MouseBinding> mouse_bindings_;
    std::vector<ScrollHandler> scroll_handlers_;
    std::size_t next_event_id_ = 0;

public:
    Window(int width, int height, bool fullscreen = false);
    ~Window();

public:
    [[nodiscard]] std::uint32_t& operator[](const std::pair<int, int>& xy) noexcept;
    [[nodiscard]] std::uint32_t operator[](const std::pair<int, int>& xy) const noexcept;
    [[nodiscard]] const std::vector<std::uint32_t>& get_pixel_buffer() const noexcept {
        return pixel_buffer_;
    }

    [[nodiscard]] std::size_t get_width() const noexcept { return width_; }
    [[nodiscard]] std::size_t get_height() const noexcept { return height_; }

    [[nodiscard]] bool is_key_pressed(SDL_Scancode key) const noexcept;
    [[nodiscard]] bool is_key_just_pressed(SDL_Scancode key) const noexcept;
    [[nodiscard]] bool is_key_just_released(SDL_Scancode key) const noexcept;

public:
    /**
     * @brief Registers a key binding with a trigger policy.
     * @param keys Set of scancodes to monitor.
     * @param trigger Trigger mode (pressed/down/released/etc.).
     * @param handler Callback receiving key state and dt.
     */
    void register_key(
        const std::unordered_set<SDL_Scancode>& keys, Trigger trigger, KeyHandler handler
    ) noexcept;

    /**
     * @brief Registers a mouse binding.
     * @param button SDL mouse button.
     * @param trigger Trigger mode.
     * @param handler Callback receiving xrel, yrel, dt.
     */
    void register_mouse(Uint8 button, Trigger trigger, MouseHandler handler) noexcept;

    /**
     * @brief Registers a scroll handler.
     * @param handler Callback receiving y_offset.
     */
    void register_scroll(ScrollHandler handler) noexcept;

    /** @brief Polls SDL events and dispatches registered handlers. */
    bool process_events() noexcept;
    /** @brief Uploads backbuffer to SDL texture, presents and clears buffer. */
    void update() noexcept;
    /** @brief Clears the ARGB backbuffer to black. */
    void clear() noexcept;

    void save_ppm(const std::string& filename) const;
    void save_bmp(const std::string& filename) const;

private:
    bool check_key_trigger(const KeyBinding& binding) const noexcept;
    void process_key_bindings() noexcept;
    void process_mouse_bindings() noexcept;
    void update_keyboard_state() noexcept;
    bool has_modifier_keys() const noexcept;
};
