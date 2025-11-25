#pragma once
#include "world.hpp"
#include <vector>

class Window;

class Rasterizer {
public:
    Rasterizer(int width, int height);
    
    void clear() noexcept;
    void resize(int w, int h) noexcept;
    
    // Render a single model
    void draw_model_wireframe(const Camera& camera, const std::vector<Face>& faces, 
                              Window& window, double aspect_ratio) noexcept;
    void draw_model_rasterized(const Camera& camera, const std::vector<Face>& faces,
                               Window& window, double aspect_ratio) noexcept;
    
    // Get depth for debugging
    FloatType get_depth(int x, int y) const noexcept;
    
private:
    int width_;
    int height_;
    std::vector<FloatType> z_buffer_;
    
    // Per-face rendering helpers
    void wireframe_render(const Camera& camera, const Face& face, Window& window, double aspect_ratio) noexcept;
    void rasterized_render(const Camera& camera, const Face& face, Window& window, double aspect_ratio) noexcept;
    
    // Clipping logic
    static bool inside_plane(const glm::vec4& v, ClipPlane plane) noexcept;
    static FloatType compute_intersection_t(const glm::vec4& v0, const glm::vec4& v1, ClipPlane plane) noexcept;
    static ClipVertex intersect_plane(const ClipVertex& v0, const ClipVertex& v1, ClipPlane plane) noexcept;
    static InplaceVector<ClipVertex, 9> clip_against_plane(const InplaceVector<ClipVertex, 9>& input, ClipPlane plane) noexcept;
    InplaceVector<ClipVertex, 9> clip_triangle(const Camera& camera, const Face& face, double aspect_ratio) noexcept;
    
    // Texture sampling with perspective correction
    static std::uint32_t sample_texture(const Face& face, const glm::vec3& bary, 
                                        const ScreenNdcCoord& v0, const ScreenNdcCoord& v1, const ScreenNdcCoord& v2) noexcept;
    
    // NDC to screen conversion
    ScreenNdcCoord ndc_to_screen(const glm::vec3& ndc, const glm::vec2& uv, FloatType w) const noexcept;
};
