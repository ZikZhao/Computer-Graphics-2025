#pragma once
#include <memory>
#include <vector>
#include <barrier>
#include <atomic>
#include <thread>
#include <stop_token>
#include "world.hpp"
#include "raytracer.hpp"
#include "rasterizer.hpp"

// Forward declaration
class Window;

class Renderer {
public:
    enum class Mode {
        WIREFRAME,
        RASTERIZED,
        RAYTRACED,
        DEPTH_OF_FIELD,
    };
    static constexpr int TileHeight = 16;
    static Colour TonemapAndGammaCorrect(const ColourHDR& hdr, FloatType gamma) noexcept;
    static FloatType AcesToneMapping(FloatType hdr_value) noexcept;
    
private:
    Window& window_;
    const World& world_;
    Mode mode_ = Mode::RASTERIZED;
    FloatType gamma_ = 2.2f;
    bool soft_shadows_enabled_ = false;
    bool caustics_enabled_ = false;
    FloatType focal_distance_ = 8.0f;
    FloatType aperture_size_ = 0.1f;
    int dof_samples_ = 16;
    // Sub-engines (public for event handlers)
    std::unique_ptr<RayTracer> raytracer_;
    std::unique_ptr<Rasterizer> rasterizer_;
    // Frame buffer (HDR for ray tracing, tonemapped to LDR for display)
    std::vector<ColourHDR> hdr_buffer_;
    std::vector<ColourHDR> accumulation_buffer_;
    int frame_count_ = 0;
    int rendering_frame_count_ = 0;
    double aspect_ratio_ = 1.0;
    // Multi-threading support
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
    void reset_accumulation() noexcept;
    void set_mode(Mode m) noexcept { mode_ = m; }
    FloatType gamma() const noexcept { return gamma_; }
    void set_gamma(FloatType g) noexcept { gamma_ = g; }
    bool soft_shadows_enabled() const noexcept { return soft_shadows_enabled_; }
    void set_soft_shadows_enabled(bool e) noexcept { soft_shadows_enabled_ = e; }
    bool caustics_enabled() const noexcept { return caustics_enabled_; }
    void set_caustics_enabled(bool e) noexcept { caustics_enabled_ = e; }
    bool is_photon_map_ready() const noexcept { return raytracer_ && raytracer_->is_photon_map_ready(); }
    
private:
    void clear() noexcept;
    void wireframe_render() noexcept;
    void rasterized_render() noexcept;
    void raytraced_render() noexcept;
    void depth_of_field_render() noexcept;
    void worker_thread(std::stop_token st) noexcept;
    void process_rows(int y0, int y1) noexcept;
    void draw_coordinate_axes() noexcept;
};
