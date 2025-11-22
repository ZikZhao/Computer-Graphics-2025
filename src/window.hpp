#pragma once

#include "DrawingWindow.h"
#include <vector>
#include <functional>

class Window {
public:
    DrawingWindow& drawing_window;
    
private:
    // Event callbacks
    std::vector<std::function<void(const SDL_Event&)>> event_handlers_;
    
public:
    Window(DrawingWindow& dw) : drawing_window(dw) {}
    
    // Add event handler
    void add_event_handler(std::function<void(const SDL_Event&)> handler) {
        event_handlers_.push_back(handler);
    }
    
    // Poll for events
    bool poll_events(SDL_Event& event) {
        if (!SDL_PollEvent(&event)) {
            return false;
        }
        
        // Handle window close events
        if (event.type == SDL_QUIT) {
            drawing_window.exitCleanly();
            return false;
        }
        if ((event.type == SDL_KEYDOWN) && (event.key.keysym.sym == SDLK_ESCAPE)) {
            drawing_window.exitCleanly();
            return false;
        }
        if ((event.type == SDL_WINDOWEVENT) && (event.window.event == SDL_WINDOWEVENT_CLOSE)) {
            drawing_window.exitCleanly();
            return false;
        }
        
        // Dispatch to all registered handlers
        for (auto& handler : event_handlers_) {
            handler(event);
        }
        
        return true;
    }
    
    // Forward rendering functions
    void render_frame() { drawing_window.renderFrame(); }
    void clear_pixels() { drawing_window.clearPixels(); }
    void set_pixel_colour(size_t x, size_t y, uint32_t colour) {
        drawing_window.setPixelColour(x, y, colour);
    }
    uint32_t get_pixel_colour(size_t x, size_t y) {
        return drawing_window.getPixelColour(x, y);
    }
    void save_ppm(const std::string& filename) const {
        drawing_window.savePPM(filename);
    }
    void save_bmp(const std::string& filename) const {
        drawing_window.saveBMP(filename);
    }
    
    // Accessors
    size_t width() const { return drawing_window.width; }
    size_t height() const { return drawing_window.height; }
};
