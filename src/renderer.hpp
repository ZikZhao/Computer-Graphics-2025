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
public:
    enum class Mode {
        WIREFRAME,
        RASTERIZED,
        RAYTRACED,
        DEPTH_OF_FIELD,
        PHOTON_VISUALIZATION,
    };

public:
    static constexpr int TileHeight = 16;
    static constexpr int VideoSamples = 64;

    /**
     * @brief ACES filmic tone mapping curve (approximation).
     * @param hdr_value Input HDR component.
     * @return Mapped component in [0,1].
     */
    [[nodiscard]] static constexpr FloatType AcesToneMapping(FloatType hdr_value) noexcept {
        const FloatType a = 2.51f;
        const FloatType b = 0.03f;
        const FloatType c = 2.43f;
        const FloatType d = 0.59f;
        const FloatType e = 0.14f;
        FloatType numerator = hdr_value * (a * hdr_value + b);
        FloatType denominator = hdr_value * (c * hdr_value + d) + e;
        return std::clamp(numerator / denominator, 0.0f, 1.0f);
    }

    /**
     * @brief Converts HDR to displayable sRGB using ACES and gamma.
     * @param hdr Linear HDR colour.
     * @param gamma Output gamma (1.0 = none, 2.2 typical sRGB).
     * @return 8-bit sRGB colour.
     */
    [[nodiscard]] static Colour TonemapAndGammaCorrect(
        const ColourHDR& hdr, FloatType gamma
    ) noexcept;

private:
    const World& world_;
    Window& window_;

    Mode mode_ = Mode::RASTERIZED;
    FloatType gamma_ = 2.2f;
    bool caustics_enabled_ = false;
    FloatType focal_distance_ = 8.0f;
    FloatType aperture_size_ = 0.1f;
    int dof_samples_ = 16;
    bool offline_render_mode_ = false;
    int frame_count_ = 0;
    int rendering_frame_count_ = 0;
    double aspect_ratio_ = 1.0;

    std::unique_ptr<RayTracer> raytracer_;
    std::unique_ptr<Rasterizer> rasterizer_;
    std::vector<ColourHDR> hdr_buffer_;
    std::vector<ColourHDR> accumulation_buffer_;

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
     * @param world Scene data and camera.
     * @param window Reference to the output window.
     * 
     * Renderer orchestrates rasterizer/raytracer and multi-threaded tiling.
     */
    Renderer(const World& world, Window& window);
    
    ~Renderer();

public:
    void set_mode(Mode m) noexcept { mode_ = m; }

    [[nodiscard]] FloatType gamma() const noexcept { return gamma_; }
    void set_gamma(FloatType g) noexcept { gamma_ = g; }

    [[nodiscard]] bool caustics_enabled() const noexcept { return caustics_enabled_; }
    void set_caustics_enabled(bool e) noexcept { caustics_enabled_ = e; }

    [[nodiscard]] bool is_photon_map_ready() const noexcept {
        return raytracer_ && raytracer_->is_photon_map_ready();
    }

    /**
     * @brief Returns a pointer to the photon map for debug visualization.
     */
    [[nodiscard]] const PhotonMap* photon_map() const noexcept {
        return raytracer_ ? raytracer_->photon_map() : nullptr;
    }

    [[nodiscard]] FloatType focal_distance() const noexcept { return focal_distance_; }
    void set_focal_distance(FloatType d) noexcept { focal_distance_ = d; }

    [[nodiscard]] FloatType aperture_size() const noexcept { return aperture_size_; }
    void set_aperture_size(FloatType size) noexcept { aperture_size_ = size; }

    [[nodiscard]] int get_width() const noexcept { return rasterizer_->get_width(); }
    [[nodiscard]] int get_height() const noexcept { return rasterizer_->get_height(); }

    [[nodiscard]] bool offline_render_mode() const noexcept { return offline_render_mode_; }
    void toggle_offline_render_mode() noexcept {
        offline_render_mode_ = !offline_render_mode_;
    }

public:
    void render() noexcept;
    void reset_accumulation() noexcept;

private:
    /**
     * @brief Clears z-buffer and colour buffer for new frame.
     */
    void clear() noexcept;

    void render_wireframe() noexcept;
    void render_rasterized() noexcept;
    void render_raytraced() noexcept;
    void render_dof() noexcept;
    void render_photon_cloud() noexcept;

    /**
     * @brief Worker thread function for processing tiles.
     * @param st Stop token to allow cooperative cancellation.
     */
    void worker_thread(std::stop_token st) noexcept;

    /**
     * @brief Processes a range of scanlines (tiles) for the current frame.
     * @param y0 Starting scanline (inclusive).
     * @param y1 Ending scanline (exclusive).
     */
    void process_rows(int y0, int y1) noexcept;
};
