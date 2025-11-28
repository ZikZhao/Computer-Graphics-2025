#pragma once
#include <atomic>
#include <barrier>
#include <memory>
#include <stop_token>
#include <thread>
#include <vector>

#include "rasterizer.hpp"
#include "raytracer.hpp"
#include "world.hpp"

// Forward declaration
class Window;

/**
 * @brief Orchestrates rasterization and ray tracing; manages frame tiling.
 */
class Renderer {
public:
    enum class Mode {
        WIREFRAME,
        RASTERIZED,
        RAYTRACED,
        DEPTH_OF_FIELD,
    };
    static constexpr int TileHeight = 16;
    static constexpr int VideoSamples = 64;
    /**
     * @brief Converts HDR to displayable sRGB using ACES and gamma.
     * @param hdr Linear HDR colour.
     * @param gamma Output gamma (1.0 = none, 2.2 typical sRGB).
     * @return 8-bit sRGB colour.
     */
    static Colour TonemapAndGammaCorrect(const ColourHDR& hdr, FloatType gamma) noexcept;

    /**
     * @brief ACES filmic tone mapping curve (approximation).
     * @param hdr_value Input HDR component.
     * @return Mapped component in \[0,1].
     */
    static constexpr FloatType AcesToneMapping(FloatType hdr_value) noexcept;

private:
    Window& window_;
    const World& world_;
    Mode mode_ = Mode::RASTERIZED;
    FloatType gamma_ = 2.2f;
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
    /**
     * @brief Constructs the renderer with shared window/world.
     * @param window Output target and event source.
     * @param world Scene data and camera.
     */
    explicit Renderer(Window& window, const World& world);

    /** @brief Destructor joins worker threads and releases resources. */
    ~Renderer();

    /** @brief Renders a frame in the current mode. */
    void render() noexcept;

    /** @brief Clears progressive accumulation buffers. */
    void reset_accumulation() noexcept;
    bool video_export_mode_ = false;
    /** @brief Sets the rendering mode. */
    void set_mode(Mode m) noexcept { mode_ = m; }
    /** @brief Returns current display gamma. */
    FloatType gamma() const noexcept { return gamma_; }
    /** @brief Sets display gamma. */
    void set_gamma(FloatType g) noexcept { gamma_ = g; }
    /** @brief Returns whether caustics are enabled. */
    bool caustics_enabled() const noexcept { return caustics_enabled_; }
    /** @brief Enables or disables photon-mapped caustics. */
    void set_caustics_enabled(bool e) noexcept { caustics_enabled_ = e; }
    /** @brief Indicates whether the photon map is ready. */
    bool is_photon_map_ready() const noexcept { return raytracer_ && raytracer_->is_photon_map_ready(); }

private:
    void clear() noexcept;
    void wireframe_render() noexcept;
    void rasterized_render() noexcept;
    void raytraced_render() noexcept;
    void depth_of_field_render() noexcept;
    void worker_thread(std::stop_token st) noexcept;
    void process_rows(int y0, int y1) noexcept;
};
