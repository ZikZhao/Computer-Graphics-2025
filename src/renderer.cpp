#include "renderer.hpp"
#include "DrawingWindow.h"
#include <algorithm>

Renderer::Renderer(DrawingWindow& window, const World& world)
    : window_(window),
      world_(world),
      hdr_buffer_(window.width * window.height, ColourHDR()),
      frame_barrier_(std::thread::hardware_concurrency() + 1) {
    
    // Create sub-engines
    raytracer_ = std::make_unique<RayTracer>(world);
    rasterizer_ = std::make_unique<Rasterizer>(window.width, window.height);
    
    // Start worker threads
    for (unsigned int i = 0; i < std::thread::hardware_concurrency(); ++i) {
        workers_.emplace_back([this](std::stop_token) { this->worker_thread(); });
    }
}

void Renderer::clear() noexcept {
    rasterizer_->clear();
    hdr_buffer_.assign(window_.width * window_.height, ColourHDR());
}

void Renderer::render() noexcept {
    clear();
    aspect_ratio_ = static_cast<double>(window_.width) / window_.height;
    
    switch (mode_) {
    case Wireframe:
        wireframe_render();
        break;
    case Rasterized:
        rasterized_render();
        break;
    case Raytraced:
        raytraced_render();
        break;
    case DepthOfField:
        depth_of_field_render();
        break;
    }
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
    
    // Signal workers to start and wait for completion
    frame_barrier_.arrive_and_wait();  // Start signal
    frame_barrier_.arrive_and_wait();  // Completion wait
}

void Renderer::depth_of_field_render() noexcept {
    current_camera_ = &world_.camera();
    tile_counter_.store(0, std::memory_order_relaxed);
    
    // Signal workers to start and wait for completion
    frame_barrier_.arrive_and_wait();  // Start signal
    frame_barrier_.arrive_and_wait();  // Completion wait
}

void Renderer::process_rows(int y0, int y1) noexcept {
    const Camera& camera = *current_camera_;
    const bool is_dof = (mode_ == DepthOfField);
    
    for (int y = y0; y < y1; ++y) {
        for (int x = 0; x < static_cast<int>(window_.width); x++) {
            ColourHDR final_hdr;
            
            if (is_dof) {
                final_hdr = raytracer_->render_pixel_dof(
                    camera, x, y, window_.width, window_.height,
                    focal_distance_, aperture_size_, dof_samples_,
                    soft_shadows_enabled_, world_.light_intensity()
                );
            } else {
                final_hdr = raytracer_->render_pixel(
                    camera, x, y, window_.width, window_.height,
                    soft_shadows_enabled_, world_.light_intensity()
                );
            }
            
            Colour final_colour = tonemap_and_gamma_correct(final_hdr, gamma_);
            window_.setPixelColour(x, y, final_colour);
        }
    }
}

void Renderer::worker_thread() noexcept {
    while (true) {
        // Wait for frame start signal
        frame_barrier_.arrive_and_wait();
        
        // Process tiles until all are done
        const int num_tiles = (window_.height + TileHeight - 1) / TileHeight;
        while (true) {
            int tile_idx = tile_counter_.fetch_add(1, std::memory_order_relaxed);
            if (tile_idx >= num_tiles) break;
            
            int y0 = tile_idx * TileHeight;
            int y1 = std::min(y0 + TileHeight, static_cast<int>(window_.height));
            process_rows(y0, y1);
        }
        
        // Signal frame completion
        frame_barrier_.arrive_and_wait();
    }
}

void Renderer::handle_event(const SDL_Event& event) noexcept {
    if (event.type == SDL_KEYDOWN) {
        switch (event.key.keysym.sym) {
        case SDLK_1:
            mode_ = Wireframe;
            break;
        case SDLK_2:
            mode_ = Rasterized;
            break;
        case SDLK_3:
            mode_ = Raytraced;
            break;
        case SDLK_4:
            mode_ = DepthOfField;
            break;
        case SDLK_g:
            gamma_ = (gamma_ == 2.2f) ? 1.0f : 2.2f;
            break;
        case SDLK_h:
            soft_shadows_enabled_ = !soft_shadows_enabled_;
            break;
        }
    }
}

FloatType Renderer::aces_tonemap(FloatType hdr_value) noexcept {
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
    FloatType r_ldr = aces_tonemap(hdr.red);
    FloatType g_ldr = aces_tonemap(hdr.green);
    FloatType b_ldr = aces_tonemap(hdr.blue);
    
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
    
    return Colour{
        static_cast<std::uint8_t>(std::clamp(r_out * 255.0f, 0.0f, 255.0f)),
        static_cast<std::uint8_t>(std::clamp(g_out * 255.0f, 0.0f, 255.0f)),
        static_cast<std::uint8_t>(std::clamp(b_out * 255.0f, 0.0f, 255.0f))
    };
}
