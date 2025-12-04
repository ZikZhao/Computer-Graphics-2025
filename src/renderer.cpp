#include "renderer.hpp"

#include <algorithm>
#include <bit>
#include <iostream>

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

Renderer::Renderer(const World& world, Window& window)
    : world_(world),
      window_(window),
      aspect_ratio_(static_cast<double>(window.width_) / window.height_),
      hdr_buffer_(window.width_ * window.height_, ColourHDR{}),
      accumulation_buffer_(window.width_ * window.height_, ColourHDR{}),
      frame_barrier_(std::thread::hardware_concurrency() + 1) {
    // Create sub-engines (ray tracing and rasterization backends)
    raytracer_ = std::make_unique<RayTracer>(world);
    rasterizer_ = std::make_unique<Rasterizer>(window);

    // Pre-generate Hilbert curve tile ordering for cache-friendly traversal
    const int tiles_x =
        (static_cast<int>(window.width_) + Constant::TileSize - 1) / Constant::TileSize;
    const int tiles_y =
        (static_cast<int>(window.height_) + Constant::TileSize - 1) / Constant::TileSize;
    generate_hilbert_order(tiles_x, tiles_y);

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
    // In offline render mode, reset accumulation to produce per-frame averages
    if (offline_render_mode_) {
        reset_accumulation();
    }

    switch (mode_) {
    case Mode::WIREFRAME:
        clear();
        render_wireframe();
        break;
    case Mode::RASTERIZED:
        clear();
        render_rasterized();
        break;
    case Mode::RAYTRACED:
        render_raytraced();
        break;
    case Mode::DEPTH_OF_FIELD:
        render_dof();
        break;
    case Mode::PHOTON_VISUALIZATION:
        render_photon_cloud();
        break;
    }
}

void Renderer::reset_accumulation() noexcept {
    std::fill(accumulation_buffer_.begin(), accumulation_buffer_.end(), ColourHDR{});
    frame_count_ = 0;
}

void Renderer::clear() noexcept {
    rasterizer_->clear();
    hdr_buffer_.assign(window_.width_ * window_.height_, ColourHDR{});
}

void Renderer::render_wireframe() noexcept {
    rasterizer_->wireframe(world_.camera_, world_.all_faces_, world_.all_vertices_);
}

void Renderer::render_rasterized() noexcept {
    rasterizer_->rasterized(
        world_.camera_, world_.all_faces_, world_.all_vertices_, world_.all_texcoords_
    );
}

void Renderer::render_raytraced() noexcept {
    static glm::vec3 last_cam_pos = glm::vec3(0.0f);
    static FloatType last_cam_yaw = 0.0f;
    static FloatType last_cam_pitch = 0.0f;

    tile_counter_.store(0, std::memory_order_relaxed);
    const Camera& cam = world_.camera_;
    bool cam_changed = (glm::length(cam.position_ - last_cam_pos) > 1e-6f) ||
                       (std::abs(cam.yaw_ - last_cam_yaw) > 1e-6f) ||
                       (std::abs(cam.pitch_ - last_cam_pitch) > 1e-6f);
    if (cam_changed) {
        reset_accumulation();
    }
    rendering_frame_count_ = frame_count_ + 1;

    // Signal workers to start and wait for completion via barrier
    frame_barrier_.arrive_and_wait();  // Start signal
    frame_barrier_.arrive_and_wait();  // Completion wait
    frame_count_ = rendering_frame_count_;
    last_cam_pos = cam.position_;
    last_cam_yaw = cam.yaw_;
    last_cam_pitch = cam.pitch_;
}

void Renderer::render_dof() noexcept {
    tile_counter_.store(0, std::memory_order_relaxed);
    rendering_frame_count_ = frame_count_ + 1;

    // Signal workers to start and wait for completion via barrier
    frame_barrier_.arrive_and_wait();  // Start signal
    frame_barrier_.arrive_and_wait();  // Completion wait
    frame_count_ = rendering_frame_count_;
}

void Renderer::process_tile(int tile_x, int tile_y) noexcept {
    const Camera& camera = world_.camera_;
    const bool is_dof = mode_ == Mode::DEPTH_OF_FIELD;

    const int w = static_cast<int>(window_.width_);
    const int h = static_cast<int>(window_.height_);

    int x0 = tile_x * Constant::TileSize;
    int y0 = tile_y * Constant::TileSize;
    int x1 = std::min(x0 + Constant::TileSize, w);
    int y1 = std::min(y0 + Constant::TileSize, h);

    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            int pixel_index = y * w + x;

            // Normal debug mode: simple single-sample rendering
            if (normal_debug_mode_) {
                ColourHDR hdr = raytracer_->render_pixel_normal(camera, x, y, w, h);
                std::size_t idx = static_cast<std::size_t>(y) * w + static_cast<std::size_t>(x);
                accumulation_buffer_[idx] = hdr;
                Colour c = TonemapAndGammaCorrect(hdr, gamma_);
                window_[{x, y}] = c;
                continue;
            }

            // Use high sample count in offline mode
            int samples_to_run = offline_render_mode_ ? Constant::VideoSamples : 1;
            ColourHDR pixel_accum{0.0f, 0.0f, 0.0f};
            for (int s = 0; s < samples_to_run; ++s) {
                if (is_dof) {
                    // Depth-of-field rendering
                    ColourHDR hdr = raytracer_->render_pixel_dof(
                        camera,
                        x,
                        y,
                        w,
                        h,
                        focal_distance_,
                        aperture_size_,
                        dof_samples_,
                        caustics_enabled_
                    );
                    pixel_accum = ColourHDR{
                        .red = pixel_accum.red + hdr.red,
                        .green = pixel_accum.green + hdr.green,
                        .blue = pixel_accum.blue + hdr.blue
                    };
                } else {
                    // Normal Path tracing -- jittered sampling per frame for progressive refinement
                    std::uint32_t base_seed =
                        static_cast<std::uint32_t>(pixel_index + rendering_frame_count_ * 123457u) |
                        1u;
                    std::uint32_t sub_seed =
                        base_seed ^ (static_cast<std::uint32_t>(s) * 0x9e3779b9u);
                    ColourHDR hdr =
                        raytracer_->render_pixel(camera, x, y, w, h, caustics_enabled_, sub_seed);
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
            std::size_t idx = static_cast<std::size_t>(y) * w + static_cast<std::size_t>(x);

            // Update accumulation buffer
            if (offline_render_mode_) {
                accumulation_buffer_[idx] = final_hdr_avg;
            } else {
                accumulation_buffer_[idx] = ColourHDR{
                    .red = accumulation_buffer_[idx].red + final_hdr_avg.red,
                    .green = accumulation_buffer_[idx].green + final_hdr_avg.green,
                    .blue = accumulation_buffer_[idx].blue + final_hdr_avg.blue
                };
            }
            // Compute display HDR (per-frame or progressive average)
            ColourHDR avg_hdr = offline_render_mode_
                                    ? accumulation_buffer_[idx]
                                    : ColourHDR{
                                          .red = accumulation_buffer_[idx].red /
                                                 static_cast<FloatType>(rendering_frame_count_),
                                          .green = accumulation_buffer_[idx].green /
                                                   static_cast<FloatType>(rendering_frame_count_),
                                          .blue = accumulation_buffer_[idx].blue /
                                                  static_cast<FloatType>(rendering_frame_count_)
                                      };
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

        // Process tiles using Hilbert curve ordering for better cache locality
        const int num_tiles = static_cast<int>(hilbert_tile_order_.size());
        while (true) {
            int tile_idx = tile_counter_.fetch_add(1, std::memory_order_relaxed);
            if (tile_idx >= num_tiles) break;

            auto [tile_x, tile_y] = hilbert_tile_order_[tile_idx];
            process_tile(tile_x, tile_y);
        }

        // Signal frame completion
        frame_barrier_.arrive_and_wait();
    }
}

void Renderer::render_photon_cloud() noexcept {
    const int width = static_cast<int>(window_.width_);
    const int height = static_cast<int>(window_.height_);

    const PhotonMap& pm = raytracer_->photon_map();
    if (!pm.is_ready()) return;

    const auto render = [&](const Photon& photon) {
        glm::vec4 clip = world_.camera_.world_to_clip(photon.position);
        if (clip.w <= 0.0f) return;
        glm::vec3 ndc = world_.camera_.clip_to_ndc(clip);
        if (ndc.x < -1.0f || ndc.x > 1.0f || ndc.y < -1.0f || ndc.y > 1.0f || ndc.z < 0.0f ||
            ndc.z > 1.0f)
            return;

        int screen_x = static_cast<int>((ndc.x + 1.0f) * 0.5f * static_cast<FloatType>(width));
        int screen_y = static_cast<int>((1.0f - ndc.y) * 0.5f * static_cast<FloatType>(height));
        if (screen_x < 0 || screen_x >= width || screen_y < 0 || screen_y >= height) return;

        // Compute brightness from photon power (with exposure boost for visibility)
        FloatType lum = (photon.power.r + photon.power.g + photon.power.b) / 3.0f *
                        Constant::PhotonVisualizationExposure;
        lum = std::clamp(lum, 0.0f, 1.0f);

        // Use white dots for visibility, or tint by photon power
        Colour dot_color = Colour{
            .red = static_cast<std::uint8_t>(std::clamp(
                photon.power.r * Constant::PhotonVisualizationExposure * 255.0f, 0.0f, 255.0f
            )),
            .green = static_cast<std::uint8_t>(std::clamp(
                photon.power.g * Constant::PhotonVisualizationExposure * 255.0f, 0.0f, 255.0f
            )),
            .blue = static_cast<std::uint8_t>(std::clamp(
                photon.power.b * Constant::PhotonVisualizationExposure * 255.0f, 0.0f, 255.0f
            ))
        };

        // Draw a 2x2 block for better visibility
        for (int dy = 0; dy < 2; dy++) {
            for (int dx = 0; dx < 2; dx++) {
                int px = screen_x + dx;
                int py = screen_y + dy;
                if (px >= 0 && px < width && py >= 0 && py < height) {
                    window_[{px, py}] = dot_color;
                }
            }
        }
    };

    for (const auto& cell : pm.grid()) {
        for (const auto& p : cell) {
            render(p);
        }
    }
}

[[nodiscard]] glm::ivec2 hilbert_index_to_coord(int n, int d) noexcept {
    glm::ivec2 pos(0);

    for (int s = 1; s < n; s *= 2) {
        const int rx = 1 & (d / 2);
        const int ry = 1 & (d ^ rx);

        if (ry == 0) {
            if (rx == 1) {
                pos = glm::ivec2(s - 1) - pos;
            }
            std::swap(pos.x, pos.y);
        }

        pos += glm::ivec2(rx, ry) * s;
        d /= 4;
    }
    return pos;
}

void Renderer::generate_hilbert_order(int tiles_x, int tiles_y) noexcept {
    assert(hilbert_tile_order_.empty());
    hilbert_tile_order_.reserve(tiles_x * tiles_y);

    const int max_dim = std::max(tiles_x, tiles_y);
    const int n =
        static_cast<int>(std::bit_ceil(static_cast<unsigned int>(max_dim)));  // Next power of two

    const int total_cells = n * n;

    for (int d = 0; d < total_cells; ++d) {
        const glm::ivec2 pos = hilbert_index_to_coord(n, d);
        if (pos.x < tiles_x && pos.y < tiles_y) {
            hilbert_tile_order_.emplace_back(pos.x, pos.y);
        }
    }

    hilbert_tile_order_.shrink_to_fit();
}
