#pragma once
#include "world.hpp"
#include "raytracer.hpp"
#include "rasterizer.hpp"
#include <memory>
#include <vector>
#include <barrier>
#include <atomic>
#include <thread>

// Forward declaration
class DrawingWindow;

class Renderer {
public:
    enum Mode {
        Wireframe,
        Rasterized,
        Raytraced,
        DepthOfField,
    };
    
    Mode mode_ = Raytraced;
    FloatType gamma_ = 2.2f;
    bool soft_shadows_enabled_ = false;
    
    // Depth of field parameters
    FloatType focal_distance_ = 8.0f;
    FloatType aperture_size_ = 0.1f;
    int dof_samples_ = 16;
    
private:
    DrawingWindow& window_;
    const World& world_;
    
    // Sub-engines
    std::unique_ptr<RayTracer> raytracer_;
    std::unique_ptr<Rasterizer> rasterizer_;
    
    // Frame buffer (HDR for ray tracing, tonemapped to LDR for display)
    std::vector<ColourHDR> hdr_buffer_;
    double aspect_ratio_ = 1.0;
    
    // Multi-threading support
    static constexpr int TileHeight = 16;
    std::barrier<> frame_barrier_;
    std::vector<std::jthread> workers_;
    std::atomic<int> tile_counter_ = 0;
    const Camera* current_camera_ = nullptr;
    
public:
    Renderer(DrawingWindow& window, const World& world);
    ~Renderer() = default;
    
    void render() noexcept;
    void handle_event(const SDL_Event& event) noexcept;
    
private:
    void clear() noexcept;
    
    // Mode-specific rendering dispatchers
    void wireframe_render() noexcept;
    void rasterized_render() noexcept;
    void raytraced_render() noexcept;
    void depth_of_field_render() noexcept;
    
    // Worker thread function
    void worker_thread() noexcept;
    void process_rows(int y0, int y1) noexcept;
    
    // Tonemapping and gamma correction
    static Colour tonemap_and_gamma_correct(const ColourHDR& hdr, FloatType gamma) noexcept;
    static FloatType aces_tonemap(FloatType hdr_value) noexcept;
    
    // Coordinate axis visualization
    void draw_coordinate_axes() noexcept;
};
