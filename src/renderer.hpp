#pragma once
#include <atomic>
#include <barrier>
#include <memory>
#include <stop_token>
#include <thread>
#include <vector>

#include "rasterizer.hpp"
#include "raytracer.hpp"
#include "window.hpp"
#include "world.hpp"

/**
 * @brief Orchestrates rasterization and ray tracing; manages frame tiling.
 */
class Renderer {
public: // Types
    enum class Mode {
        WIREFRAME,
        RASTERIZED,
        RAYTRACED,
        DEPTH_OF_FIELD,
    };

public: // Static Methods & Constants
    static constexpr int TileHeight = 16;
    static constexpr int VideoSamples = 64;

    /**
     * @brief Converts HDR to displayable sRGB using ACES and gamma.
     * @param hdr Linear HDR colour.
     * @param gamma Output gamma (1.0 = none, 2.2 typical sRGB).
     * @return 8-bit sRGB colour.
     */
    [[nodiscard]] static Colour TonemapAndGammaCorrect(
        const ColourHDR& hdr, FloatType gamma
    ) noexcept;

    /**
     * @brief ACES filmic tone mapping curve (approximation).
     * @param hdr_value Input HDR component.
     * @return Mapped component in [0,1].
     */
    [[nodiscard]] static constexpr FloatType AcesToneMapping(FloatType hdr_value) noexcept;

private: // Data
    const World& world_;
    Window& window_;
    Mode mode_ = Mode::RASTERIZED;
    FloatType gamma_ = 2.2f;
    bool caustics_enabled_ = false;
    FloatType focal_distance_ = 8.0f;
    FloatType aperture_size_ = 0.1f;
    int dof_samples_ = 16;
    bool video_export_mode_ = false;
    std::unique_ptr<RayTracer> raytracer_;
    std::unique_ptr<Rasterizer> rasterizer_;
    std::vector<ColourHDR> hdr_buffer_;
    std::vector<ColourHDR> accumulation_buffer_;
    int frame_count_ = 0;
    int rendering_frame_count_ = 0;
    double aspect_ratio_ = 1.0;
    std::barrier<> frame_barrier_;
    std::vector<std::jthread> workers_;
    std::atomic<int> tile_counter_ = 0;
    const Camera* current_camera_ = nullptr;
    glm::vec3 last_cam_pos_ = glm::vec3(0.0f);
    FloatType last_cam_yaw_ = 0.0f;
    FloatType last_cam_pitch_ = 0.0f;

public: // Lifecycle
    /**
     * @brief Constructs the renderer with shared window/world.
     * @param world Scene data and camera.
     * @param window Reference to the output window.
     */
    explicit Renderer(const World& world, Window& window);
    ~Renderer();

public: // Accessors & Data Binding
    void set_mode(Mode m) noexcept { mode_ = m; }

    [[nodiscard]] FloatType gamma() const noexcept { return gamma_; }
    void set_gamma(FloatType g) noexcept { gamma_ = g; }

    [[nodiscard]] bool caustics_enabled() const noexcept { return caustics_enabled_; }
    void set_caustics_enabled(bool e) noexcept { caustics_enabled_ = e; }

    [[nodiscard]] bool is_photon_map_ready() const noexcept {
        return raytracer_ && raytracer_->is_photon_map_ready();
    }

    [[nodiscard]] FloatType focal_distance() const noexcept { return focal_distance_; }
    void set_focal_distance(FloatType d) noexcept { focal_distance_ = d; }

    [[nodiscard]] FloatType aperture_size() const noexcept { return aperture_size_; }
    void set_aperture_size(FloatType size) noexcept { aperture_size_ = size; }

    [[nodiscard]] int get_width() const noexcept { return rasterizer_->get_width(); }
    [[nodiscard]] int get_height() const noexcept { return rasterizer_->get_height(); }

    [[nodiscard]] bool video_export_mode() const noexcept { return video_export_mode_; }
    void set_video_export_mode(bool mode) noexcept { video_export_mode_ = mode; }

public: // Core Operations
    void render() noexcept;
    void reset_accumulation() noexcept;

private: // Core Operations (Internal)
    void clear() noexcept;
    void wireframe_render() noexcept;
    void rasterized_render() noexcept;
    void raytraced_render() noexcept;
    void depth_of_field_render() noexcept;
    void worker_thread(std::stop_token st) noexcept;
    void process_rows(int y0, int y1) noexcept;
};
