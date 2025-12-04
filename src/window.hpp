#pragma once

#include <array>
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <optional>
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
        ANY_PRESSED,       // Non-modifier keys: blocked when modifiers pressed
        ANY_JUST_PRESSED,  // Non-modifier keys: blocked when modifiers pressed
        ALL_JUST_PRESSED   // Modifier combo: triggers on just-pressed when ALL keys held
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
        std::optional<std::chrono::steady_clock::time_point> last_time;
    };

    struct MouseBinding {
        Uint8 button;
        Trigger trigger;
        MouseHandler handler;
        bool first_motion;
        std::optional<std::chrono::steady_clock::time_point> last_time;
    };

public:
    // Window properties
    const std::size_t width_;
    const std::size_t height_;

private:
    // SDL components
    SDL_Window* window_;
    SDL_Renderer* renderer_;
    SDL_Texture* texture_;

    // Backbuffer
    std::vector<std::uint32_t> pixel_buffer_;

    // Input management
    KeyState keys_this_frame_{};
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
    Window(int width, int height);
    ~Window();

public:
    [[nodiscard]] std::uint32_t& operator[](std::pair<int, int> xy) noexcept;
    [[nodiscard]] std::uint32_t operator[](std::pair<int, int> xy) const noexcept;
    [[nodiscard]] const std::vector<std::uint32_t>& get_pixel_buffer() const noexcept {
        return pixel_buffer_;
    }

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
    void register_mouse(std::uint8_t button, Trigger trigger, MouseHandler handler) noexcept;

    /**
     * @brief Registers a scroll handler.
     * @param handler Callback receiving y_offset.
     */
    void register_scroll(ScrollHandler handler) noexcept;

    /// @brief Polls SDL events and dispatches registered handlers.
    bool process_events() noexcept;
    /// @brief Uploads backbuffer to SDL texture, presents and clears buffer.
    void update() noexcept;
    /// @brief Clears the ARGB backbuffer to black.
    void clear() noexcept;

    void save_ppm(const std::string& filename) const;
    void save_bmp(const std::string& filename) const;

private:
    /// @brief Processes registered key bindings based on current state.
    void process_key_bindings() noexcept;

    /// @brief Processes registered mouse bindings based on current state.
    void process_mouse_bindings() noexcept;

    /**
     * @brief Checks if any modifier key is currently pressed.
     * @return True if Shift, Ctrl, or Alt is pressed.
     */
    bool is_modifier_key_pressed() const noexcept;

    /**
     * @brief Checks if a key binding's trigger condition is met.
     * @param binding Key binding to check.
     * @return True if the binding should trigger.
     */
    bool check_key_trigger(const KeyBinding& binding) const noexcept;
};
