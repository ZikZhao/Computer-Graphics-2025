#include "renderer.hpp"

#include <algorithm>
#include <iostream>

// Filmic tone mapping (ACES approximation): compress HDR to displayable range
constexpr FloatType Renderer::AcesToneMapping(FloatType hdr_value) noexcept {
    const FloatType a = 2.51f;
    const FloatType b = 0.03f;
    const FloatType c = 2.43f;
    const FloatType d = 0.59f;
    const FloatType e = 0.14f;
    FloatType numerator = hdr_value * (a * hdr_value + b);
    FloatType denominator = hdr_value * (c * hdr_value + d) + e;
    return std::clamp(numerator / denominator, 0.0f, 1.0f);
}

// Tonemap then gamma-correct HDR to 8-bit sRGB for the window backbuffer
Colour Renderer::TonemapAndGammaCorrect(const ColourHDR& hdr, FloatType gamma) noexcept {
    FloatType r_ldr = AcesToneMapping(hdr.red * 0.6f);
    FloatType g_ldr = AcesToneMapping(hdr.green * 0.6f);
    FloatType b_ldr = AcesToneMapping(hdr.blue * 0.6f);
    FloatType r_out, g_out, b_out;
    if (gamma == 1.0f) {
        r_out = r_ldr;
        g_out = g_ldr;
        b_out = b_ldr;
    } else {
        r_out = std::pow(r_ldr, 1.0f / gamma);
        g_out = std::pow(g_ldr, 1.0f / gamma);
        b_out = std::pow(b_ldr, 1.0f / gamma);
    }
    r_out = std::clamp(r_out, 0.0f, 1.0f);
    g_out = std::clamp(g_out, 0.0f, 1.0f);
    b_out = std::clamp(b_out, 0.0f, 1.0f);
    return Colour{
        .red = static_cast<std::uint8_t>(std::clamp(r_out * 255.0f, 0.0f, 255.0f)),
        .green = static_cast<std::uint8_t>(std::clamp(g_out * 255.0f, 0.0f, 255.0f)),
        .blue = static_cast<std::uint8_t>(std::clamp(b_out * 255.0f, 0.0f, 255.0f))
    };
}

// Renderer orchestrates rasterizer/raytracer and multi-threaded tiling
Renderer::Renderer(const World& world, Window& window)
    : world_(world),
      window_(window),
      hdr_buffer_(window.get_width() * window.get_height(), ColourHDR{}),
      accumulation_buffer_(window.get_width() * window.get_height(), ColourHDR{}),
      frame_barrier_(std::thread::hardware_concurrency() + 1) {
    // Create sub-engines (ray tracing and rasterization backends)
    raytracer_ = std::make_unique<RayTracer>(world);
    rasterizer_ = std::make_unique<Rasterizer>(window);

    // Launch worker threads for tiled rendering; barrier synchronizes frame boundaries
    for (unsigned int i = 0; i < std::thread::hardware_concurrency(); ++i) {
        workers_.emplace_back([this](std::stop_token st) { this->worker_thread(st); });
    }
}

Renderer::~Renderer() {
    for (auto& w : workers_) {
        w.request_stop();
    }
    frame_barrier_.arrive_and_wait();
}

void Renderer::render() noexcept {
    // In video mode, reset accumulation to produce per-frame averages
    if (video_export_mode_) {
        reset_accumulation();
    }
    aspect_ratio_ = static_cast<double>(rasterizer_->get_width()) / rasterizer_->get_height();

    // Dispatch to selected rendering mode
    switch (mode_) {
    case Mode::WIREFRAME:
        clear();
        wireframe_render();
        break;
    case Mode::RASTERIZED:
        clear();
        rasterized_render();
        break;
    case Mode::RAYTRACED:
        raytraced_render();
        break;
    case Mode::DEPTH_OF_FIELD:
        depth_of_field_render();
        break;
    }

    // FPS counter
    static std::uint32_t fps = 0;
    static auto last_print = std::chrono::steady_clock::now();

    fps++;
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - last_print;
    if (elapsed >= std::chrono::seconds(1)) {
        std::cout << std::format(
            "[Renderer] FPS: {:#.3g} | Avg Frame Time: {:#.4g} ms\n",
            static_cast<double>(fps) / elapsed.count(),
            elapsed.count() / static_cast<double>(fps) * 1000.0
        );
        fps = 0;
        last_print = now;
    }
}

void Renderer::reset_accumulation() noexcept {
    std::fill(accumulation_buffer_.begin(), accumulation_buffer_.end(), ColourHDR{});
    frame_count_ = 0;
}

void Renderer::clear() noexcept {
    rasterizer_->clear();
    hdr_buffer_.assign(get_width() * get_height(), ColourHDR{});
}

void Renderer::wireframe_render() noexcept {
    rasterizer_->draw_model_wireframe(
        world_.camera(), world_.all_faces(), world_.all_vertices(), aspect_ratio_
    );
}

void Renderer::rasterized_render() noexcept {
    rasterizer_->draw_model_rasterized(
        world_.camera(),
        world_.all_faces(),
        world_.all_vertices(),
        world_.all_texcoords(),
        aspect_ratio_
    );
}

void Renderer::raytraced_render() noexcept {
    current_camera_ = &world_.camera();
    tile_counter_.store(0, std::memory_order_relaxed);
    glm::vec3 cur_pos = current_camera_->position();
    FloatType cur_yaw = current_camera_->yaw();
    FloatType cur_pitch = current_camera_->pitch();
    bool cam_changed = (glm::length(cur_pos - last_cam_pos_) > 1e-6f) ||
                       (std::abs(cur_yaw - last_cam_yaw_) > 1e-6f) ||
                       (std::abs(cur_pitch - last_cam_pitch_) > 1e-6f);
    if (cam_changed) {
        // Camera moved — reset progressive accumulation to avoid ghosting
        reset_accumulation();
    }
    rendering_frame_count_ = frame_count_ + 1;

    // Signal workers to start and wait for completion via barrier
    frame_barrier_.arrive_and_wait();  // Start signal
    frame_barrier_.arrive_and_wait();  // Completion wait
    frame_count_ = rendering_frame_count_;
    last_cam_pos_ = cur_pos;
    last_cam_yaw_ = cur_yaw;
    last_cam_pitch_ = cur_pitch;
}

void Renderer::depth_of_field_render() noexcept {
    current_camera_ = &world_.camera();
    tile_counter_.store(0, std::memory_order_relaxed);
    rendering_frame_count_ = frame_count_ + 1;

    // Signal workers to start and wait for completion via barrier
    frame_barrier_.arrive_and_wait();  // Start signal
    frame_barrier_.arrive_and_wait();  // Completion wait
    frame_count_ = rendering_frame_count_;
}

void Renderer::process_rows(int y0, int y1) noexcept {
    const Camera& camera = *current_camera_;
    const bool is_dof = (mode_ == Mode::DEPTH_OF_FIELD);

    // Row-batched loop over pixels; sampling and accumulation
    for (int y = y0; y < y1; ++y) {
        for (int x = 0; x < get_width(); x++) {
            int w = get_width();
            int h = get_height();
            int pixel_index = y * w + x;
            int samples_to_run = video_export_mode_ ? VideoSamples : 1;
            ColourHDR pixel_accum{0.0f, 0.0f, 0.0f};
            for (int s = 0; s < samples_to_run; ++s) {
                if (is_dof) {
                    // Depth-of-field rendering — thin lens (aperture/focal plane)
                    ColourHDR hdr = raytracer_->render_pixel_dof(
                        camera,
                        x,
                        y,
                        w,
                        h,
                        focal_distance_,
                        aperture_size_,
                        dof_samples_,
                        true,
                        caustics_enabled_
                    );
                    pixel_accum = ColourHDR{
                        .red = pixel_accum.red + hdr.red,
                        .green = pixel_accum.green + hdr.green,
                        .blue = pixel_accum.blue + hdr.blue
                    };
                } else {
                    // Path tracing — jittered sampling per frame for progressive refinement
                    int sample_index = rendering_frame_count_ * (w * h) + pixel_index + s;
                    std::uint32_t base_seed =
                        static_cast<std::uint32_t>(pixel_index + rendering_frame_count_ * 123457u) |
                        1u;
                    std::uint32_t sub_seed =
                        base_seed ^ (static_cast<std::uint32_t>(s) * 0x9e3779b9u);
                    ColourHDR hdr = raytracer_->render_pixel(
                        camera, x, y, w, h, true, caustics_enabled_, sample_index, sub_seed
                    );
                    pixel_accum = ColourHDR{
                        .red = pixel_accum.red + hdr.red,
                        .green = pixel_accum.green + hdr.green,
                        .blue = pixel_accum.blue + hdr.blue
                    };
                }
            }
            // Average samples for current pixel
            ColourHDR final_hdr_avg{
                pixel_accum.red / static_cast<FloatType>(samples_to_run),
                pixel_accum.green / static_cast<FloatType>(samples_to_run),
                pixel_accum.blue / static_cast<FloatType>(samples_to_run)
            };
            std::size_t idx =
                static_cast<std::size_t>(y) * get_width() + static_cast<std::size_t>(x);
            // Update accumulation buffer (video mode vs progressive mode)
            if (video_export_mode_) {
                accumulation_buffer_[idx] = final_hdr_avg;
            } else {
                accumulation_buffer_[idx] = ColourHDR{
                    .red = accumulation_buffer_[idx].red + final_hdr_avg.red,
                    .green = accumulation_buffer_[idx].green + final_hdr_avg.green,
                    .blue = accumulation_buffer_[idx].blue + final_hdr_avg.blue
                };
            }
            // Compute display HDR (per-frame or progressive average)
            ColourHDR avg_hdr = video_export_mode_
                                    ? accumulation_buffer_[idx]
                                    : ColourHDR{
                                          .red = accumulation_buffer_[idx].red /
                                                 static_cast<FloatType>(rendering_frame_count_),
                                          .green = accumulation_buffer_[idx].green /
                                                   static_cast<FloatType>(rendering_frame_count_),
                                          .blue = accumulation_buffer_[idx].blue /
                                                  static_cast<FloatType>(rendering_frame_count_)
                                      };
            // Tone map + gamma correct, then write to window backbuffer
            Colour final_colour = TonemapAndGammaCorrect(avg_hdr, gamma_);
            window_[{x, y}] = final_colour;
        }
    }
}

void Renderer::worker_thread(std::stop_token st) noexcept {
    while (true) {
        // Wait for frame start signal
        frame_barrier_.arrive_and_wait();
        if (st.stop_requested()) break;

        // Process tiles until all are done (row-batched for cache locality)
        const int num_tiles = (get_height() + TileHeight - 1) / TileHeight;
        while (true) {
            int tile_idx = tile_counter_.fetch_add(1, std::memory_order_relaxed);
            if (tile_idx >= num_tiles) break;

            int y0 = tile_idx * TileHeight;
            int y1 = std::min(y0 + TileHeight, get_height());
            process_rows(y0, y1);
            if (st.stop_requested()) break;
        }

        // Signal frame completion
        frame_barrier_.arrive_and_wait();
    }
}
