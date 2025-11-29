#pragma once
#include <vector>

#include "utils.hpp"
#include "window.hpp"
#include "world.hpp"

/**
 * @brief Rasterizer for wireframe and textured triangle rendering.
 *
 * This class handles the CPU-based rendering pipeline, including:
 * - Sutherland-Hodgman clipping against the view frustum.
 * - Perspective projection and division.
 * - Conversion to screen coordinates.
 * - Wireframe rendering using DDA.
 * - Textured triangle rendering with scanline fill and perspective-correct interpolation.
 * - Z-buffering for depth testing.
 */
class Rasterizer {
public: // Static Methods & Constants
    static constexpr bool InsidePlane(const glm::vec4& v, ClipPlane plane) noexcept;

    static constexpr FloatType ComputeIntersectionT(
        const glm::vec4& v0, const glm::vec4& v1, ClipPlane plane
    ) noexcept;

    static ClipVertex IntersectPlane(
        const ClipVertex& v0, const ClipVertex& v1, ClipPlane plane
    ) noexcept;

    static InplaceVector<ClipVertex, 9> ClipAgainstPlane(
        const InplaceVector<ClipVertex, 9>& input, ClipPlane plane
    ) noexcept;

    static Colour sample_texture(
        const Face& face,
        const glm::vec3& bary,
        const ScreenNdcCoord& v0,
        const ScreenNdcCoord& v1,
        const ScreenNdcCoord& v2
    ) noexcept;

private: // Data
    Window& window_;
    std::vector<FloatType> z_buffer_;

public: // Lifecycle
    explicit Rasterizer(Window& window);
    ~Rasterizer() = default;

public: // Accessors & Data Binding
    [[nodiscard]] FloatType get_depth(int x, int y) const noexcept;
    [[nodiscard]] int get_width() const noexcept { return window_.get_width(); }
    [[nodiscard]] int get_height() const noexcept { return window_.get_height(); }

public: // Core Operations
    void clear() noexcept;
    void resize() noexcept;

    void draw_model_wireframe(
        const Camera& camera,
        const std::vector<Face>& faces,
        const std::vector<glm::vec3>& vertices,
        double aspect_ratio
    ) noexcept;

    void draw_model_rasterized(
        const Camera& camera,
        const std::vector<Face>& faces,
        const std::vector<glm::vec3>& vertices,
        const std::vector<glm::vec2>& texcoords,
        double aspect_ratio
    ) noexcept;

private: // Core Operations (Internal)
    void wireframe_render(
        const Camera& camera,
        const Face& face,
        const std::vector<glm::vec3>& vertices,
        const std::vector<glm::vec2>& texcoords,
        double aspect_ratio
    ) noexcept;

    void rasterized_render(
        const Camera& camera,
        const Face& face,
        const std::vector<glm::vec3>& vertices,
        const std::vector<glm::vec2>& texcoords,
        double aspect_ratio
    ) noexcept;

    InplaceVector<ClipVertex, 9> clip_triangle(
        const Camera& camera,
        const Face& face,
        const std::vector<glm::vec3>& vertices,
        const std::vector<glm::vec2>& texcoords,
        double aspect_ratio
    ) noexcept;

    ScreenNdcCoord ndc_to_screen(const glm::vec3& ndc, const glm::vec2& uv, FloatType w)
        const noexcept;
};
