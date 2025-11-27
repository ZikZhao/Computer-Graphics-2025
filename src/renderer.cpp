#include "renderer.hpp"
#include "window.hpp"
#include <algorithm>
#include <iostream>

Renderer::Renderer(Window& window, const World& world)
    : window_(window),
      world_(world),
      hdr_buffer_(window.get_width() * window.get_height(), ColourHDR()),
      accumulation_buffer_(window.get_width() * window.get_height(), ColourHDR()),
      frame_barrier_(std::thread::hardware_concurrency() + 1) {
    
    // Create sub-engines
    raytracer_ = std::make_unique<RayTracer>(world);
    rasterizer_ = std::make_unique<Rasterizer>(window.get_width(), window.get_height());
    
    // Start worker threads
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

void Renderer::clear() noexcept {
    rasterizer_->clear();
    hdr_buffer_.assign(window_.get_width() * window_.get_height(), ColourHDR());
}

void Renderer::render() noexcept {
    aspect_ratio_ = static_cast<double>(window_.get_width()) / window_.get_height();
    
    switch (mode_) {
    case Wireframe:
        clear();
        wireframe_render();
        break;
    case Rasterized:
        clear();
        rasterized_render();
        break;
    case Raytraced:
        raytraced_render();
        break;
    case DepthOfField:
        depth_of_field_render();
        break;
    }
    
    // Draw coordinate axes overlay
    draw_coordinate_axes();
}

void Renderer::wireframe_render() noexcept {
    rasterizer_->draw_model_wireframe(world_.camera(), world_.all_faces(), window_, aspect_ratio_);
}

void Renderer::rasterized_render() noexcept {
    rasterizer_->draw_model_rasterized(world_.camera(), world_.all_faces(), window_, aspect_ratio_);
}

void Renderer::raytraced_render() noexcept {
    current_camera_ = &world_.camera();
    tile_counter_.store(0, std::memory_order_relaxed);
    glm::vec3 cur_pos = current_camera_->position_;
    FloatType cur_yaw = current_camera_->yaw_;
    FloatType cur_pitch = current_camera_->pitch_;
    bool cam_changed = (glm::length(cur_pos - last_cam_pos_) > 1e-6f) ||
                       (std::abs(cur_yaw - last_cam_yaw_) > 1e-6f) ||
                       (std::abs(cur_pitch - last_cam_pitch_) > 1e-6f);
    if (cam_changed) {
        ResetAccumulation();
    }
    rendering_frame_count_ = frame_count_ + 1;
    
    // Signal workers to start and wait for completion
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
    
    // Signal workers to start and wait for completion
    frame_barrier_.arrive_and_wait();  // Start signal
    frame_barrier_.arrive_and_wait();  // Completion wait
    frame_count_ = rendering_frame_count_;
}

void Renderer::process_rows(int y0, int y1) noexcept {
    const Camera& camera = *current_camera_;
    const bool is_dof = (mode_ == DepthOfField);
    
    for (int y = y0; y < y1; ++y) {
        for (int x = 0; x < static_cast<int>(window_.get_width()); x++) {
            ColourHDR final_hdr;
            
            if (is_dof) {
                final_hdr = raytracer_->render_pixel_dof(
                    camera, x, y, window_.get_width(), window_.get_height(),
                    focal_distance_, aperture_size_, dof_samples_,
                    true, world_.light_intensity(), caustics_enabled_
                );
            } else {
                int w = static_cast<int>(window_.get_width());
                int h = static_cast<int>(window_.get_height());
                int pixel_index = y * w + x;
                int sample_index = rendering_frame_count_ * (w * h) + pixel_index;
                uint32_t seed = static_cast<uint32_t>(pixel_index + rendering_frame_count_ * 123457u) | 1u;
                final_hdr = raytracer_->render_pixel(
                    camera, x, y, w, h,
                    true, world_.light_intensity(), caustics_enabled_, sample_index, seed
                );
            }
            
            std::size_t idx = static_cast<std::size_t>(y) * window_.get_width() + static_cast<std::size_t>(x);
            accumulation_buffer_[idx] = ColourHDR(
                accumulation_buffer_[idx].red + final_hdr.red,
                accumulation_buffer_[idx].green + final_hdr.green,
                accumulation_buffer_[idx].blue + final_hdr.blue
            );
            ColourHDR avg_hdr(
                accumulation_buffer_[idx].red / static_cast<FloatType>(rendering_frame_count_),
                accumulation_buffer_[idx].green / static_cast<FloatType>(rendering_frame_count_),
                accumulation_buffer_[idx].blue / static_cast<FloatType>(rendering_frame_count_)
            );
            Colour final_colour = tonemap_and_gamma_correct(avg_hdr, gamma_);
            window_[{x, y}] = final_colour;
        }
    }
}

void Renderer::worker_thread(std::stop_token st) noexcept {
    while (true) {
        // Wait for frame start signal
        frame_barrier_.arrive_and_wait();
        if (st.stop_requested()) break;
        
        // Process tiles until all are done
        const int num_tiles = (window_.get_height() + tile_height - 1) / tile_height;
        while (true) {
            int tile_idx = tile_counter_.fetch_add(1, std::memory_order_relaxed);
            if (tile_idx >= num_tiles) break;
            
            int y0 = tile_idx * tile_height;
            int y1 = std::min(y0 + tile_height, static_cast<int>(window_.get_height()));
            process_rows(y0, y1);
            if (st.stop_requested()) break;
        }
        
        // Signal frame completion
        frame_barrier_.arrive_and_wait();
        if (st.stop_requested()) break;
    }
}


FloatType Renderer::aces_tone_mapping(FloatType hdr_value) noexcept {
    const FloatType a = 2.51f;
    const FloatType b = 0.03f;
    const FloatType c = 2.43f;
    const FloatType d = 0.59f;
    const FloatType e = 0.14f;
    
    FloatType numerator = hdr_value * (a * hdr_value + b);
    FloatType denominator = hdr_value * (c * hdr_value + d) + e;
    
    return std::clamp(numerator / denominator, 0.0f, 1.0f);
}

Colour Renderer::tonemap_and_gamma_correct(const ColourHDR& hdr, FloatType gamma) noexcept {
    FloatType r_ldr = aces_tone_mapping(hdr.red * 0.6f);
    FloatType g_ldr = aces_tone_mapping(hdr.green * 0.6f);
    FloatType b_ldr = aces_tone_mapping(hdr.blue * 0.6f);
    
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
        static_cast<std::uint8_t>(std::clamp(r_out * 255.0f, 0.0f, 255.0f)),
        static_cast<std::uint8_t>(std::clamp(g_out * 255.0f, 0.0f, 255.0f)),
        static_cast<std::uint8_t>(std::clamp(b_out * 255.0f, 0.0f, 255.0f))
    };
}

void Renderer::draw_coordinate_axes() noexcept {
    // Configuration
    constexpr int origin_x = 60;  // X position of origin in screen space
    constexpr int origin_y = 60;  // Y position of origin in screen space
    constexpr int axis_length = 40;  // Length of each axis in pixels
    constexpr int arrow_size = 8;  // Size of arrow head
    
    // Define axis colors (RGB = XYZ)
    const Colour x_color{255, 0, 0};    // Red for X axis
    const Colour y_color{0, 255, 0};    // Green for Y axis
    const Colour z_color{0, 0, 255};    // Blue for Z axis
    
    // Get camera orientation to transform world axes to screen space
    const Camera& camera = world_.camera();
    glm::mat3 cam_orientation = camera.orientation();
    
    // World space axes
    glm::vec3 world_x{1.0f, 0.0f, 0.0f};
    glm::vec3 world_y{0.0f, 1.0f, 0.0f};
    glm::vec3 world_z{0.0f, 0.0f, 1.0f};
    
    // Transform to camera space (view space)
    glm::vec3 cam_x = glm::transpose(cam_orientation) * world_x;
    glm::vec3 cam_y = glm::transpose(cam_orientation) * world_y;
    glm::vec3 cam_z = glm::transpose(cam_orientation) * world_z;
    
    // Project to 2D screen (using camera right and up vectors for screen alignment)
    // Camera forward is -Z, right is +X, up is +Y in view space
    auto project_to_screen = [&](const glm::vec3& dir) -> std::pair<int, int> {
        // Project direction onto camera's right (x) and up (y) vectors
        // dir.x is already the right component, dir.y is up, -dir.z is forward (depth)
        float screen_x = dir.x * axis_length;
        float screen_y = -dir.y * axis_length;  // Negate Y because screen Y goes down
        return {origin_x + static_cast<int>(screen_x), origin_y + static_cast<int>(screen_y)};
    };
    
    auto [x_end_x, x_end_y] = project_to_screen(cam_x);
    auto [y_end_x, y_end_y] = project_to_screen(cam_y);
    auto [z_end_x, z_end_y] = project_to_screen(cam_z);
    
    // Helper function to draw a line using Bresenham's algorithm
    auto draw_line = [&](int x0, int y0, int x1, int y1, const Colour& color) {
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        
        while (true) {
            if (x0 >= 0 && x0 < static_cast<int>(window_.get_width()) && 
                y0 >= 0 && y0 < static_cast<int>(window_.get_height())) {
                window_[{x0, y0}] = color;
            }
            
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y0 += sy;
            }
        }
    };
    
    // Helper function to draw arrow head
    auto draw_arrow_head = [&](int x0, int y0, int x1, int y1, const Colour& color) {
        // Calculate direction vector
        float dx = static_cast<float>(x1 - x0);
        float dy = static_cast<float>(y1 - y0);
        float length = std::sqrt(dx * dx + dy * dy);
        if (length < 0.001f) return;
        
        // Normalize
        dx /= length;
        dy /= length;
        
        // Perpendicular vector
        float px = -dy;
        float py = dx;
        
        // Arrow head points
        int back_x = x1 - static_cast<int>(dx * arrow_size);
        int back_y = y1 - static_cast<int>(dy * arrow_size);
        int left_x = back_x + static_cast<int>(px * arrow_size * 0.5f);
        int left_y = back_y + static_cast<int>(py * arrow_size * 0.5f);
        int right_x = back_x - static_cast<int>(px * arrow_size * 0.5f);
        int right_y = back_y - static_cast<int>(py * arrow_size * 0.5f);
        
        draw_line(x1, y1, left_x, left_y, color);
        draw_line(x1, y1, right_x, right_y, color);
    };
    
    // Draw axes in order based on depth (back to front)
    // Axes pointing away from camera (negative Z in camera space) should be drawn first
    struct AxisInfo {
        glm::vec3 dir;
        int end_x, end_y;
        Colour color;
        float depth;
    };
    
    std::vector<AxisInfo> axes = {
        {cam_x, x_end_x, x_end_y, x_color, cam_x.z},
        {cam_y, y_end_x, y_end_y, y_color, cam_y.z},
        {cam_z, z_end_x, z_end_y, z_color, cam_z.z}
    };
    
    // Sort by depth (draw back to front)
    std::sort(axes.begin(), axes.end(), [](const AxisInfo& a, const AxisInfo& b) {
        return a.depth < b.depth;  // More negative Z (further away) drawn first
    });
    
    // Draw each axis
    for (const auto& axis : axes) {
        draw_line(origin_x, origin_y, axis.end_x, axis.end_y, axis.color);
        draw_arrow_head(origin_x, origin_y, axis.end_x, axis.end_y, axis.color);
    }
}

void Renderer::ResetAccumulation() noexcept {
    std::fill(accumulation_buffer_.begin(), accumulation_buffer_.end(), ColourHDR());
    frame_count_ = 0;
}
