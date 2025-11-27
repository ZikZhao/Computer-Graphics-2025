#pragma once
#include "world.hpp"
#include "raytracer.hpp"
#include "rasterizer.hpp"
#include <memory>
#include <vector>
#include <barrier>
#include <atomic>
#include <thread>
#include <stop_token>

// Forward declaration
class Window;

class Renderer {
public:
    enum Mode {
        Wireframe,
        Rasterized,
        Raytraced,
        DepthOfField,
    };
    
    Mode mode_ = Rasterized;
    FloatType gamma_ = 2.2f;
    bool soft_shadows_enabled_ = false;
    bool caustics_enabled_ = false;  // Photon mapping for caustics
    
    // Depth of field parameters
    FloatType focal_distance_ = 8.0f;
    FloatType aperture_size_ = 0.1f;
    int dof_samples_ = 16;
    
    // Sub-engines (public for event handlers)
    std::unique_ptr<RayTracer> raytracer_;
    std::unique_ptr<Rasterizer> rasterizer_;
    
private:
    Window& window_;
    const World& world_;
    
    // Frame buffer (HDR for ray tracing, tonemapped to LDR for display)
    std::vector<ColourHDR> hdr_buffer_;
    std::vector<ColourHDR> accumulation_buffer_;
    int frame_count_ = 0;
    int rendering_frame_count_ = 0;
    double aspect_ratio_ = 1.0;
    
    // Multi-threading support
    static constexpr int tile_height = 16;
    std::barrier<> frame_barrier_;
    std::vector<std::jthread> workers_;
    std::atomic<int> tile_counter_ = 0;
    const Camera* current_camera_ = nullptr;
    glm::vec3 last_cam_pos_ = glm::vec3(0.0f);
    FloatType last_cam_yaw_ = 0.0f;
    FloatType last_cam_pitch_ = 0.0f;
    
public:
    Renderer(Window& window, const World& world);
    ~Renderer();
    
    void render() noexcept;
    void ResetAccumulation() noexcept;
    
private:
    void clear() noexcept;
    
    // Mode-specific rendering dispatchers
    void wireframe_render() noexcept;
    void rasterized_render() noexcept;
    void raytraced_render() noexcept;
    void depth_of_field_render() noexcept;
    
    // Worker thread function
    void worker_thread(std::stop_token st) noexcept;
    void process_rows(int y0, int y1) noexcept;
    
    // Tonemapping and gamma correction
    static Colour tonemap_and_gamma_correct(const ColourHDR& hdr, FloatType gamma, uint32_t& seed) noexcept;
    static FloatType aces_tone_mapping(FloatType hdr_value) noexcept;
    
    // Coordinate axis visualization
    void draw_coordinate_axes() noexcept;
};
