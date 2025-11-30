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
public:
    /// Clip-space half-space test: inside test for a homogeneous vertex versus a frustum plane
    static constexpr bool InsidePlane(const glm::vec4& v, ClipPlane plane) noexcept {
        switch (plane) {
        case ClipPlane::LEFT:
            return v.x >= -v.w;
        case ClipPlane::RIGHT:
            return v.x <= v.w;
        case ClipPlane::BOTTOM:
            return v.y >= -v.w;
        case ClipPlane::TOP:
            return v.y <= v.w;
        case ClipPlane::NEAR:
            return v.z >= 0.0f;
        case ClipPlane::FAR:
            return v.z <= v.w;
        }
        return false;
    }

    /// Edge-plane intersection parameter t in clip space (homogeneous distances)
    static constexpr FloatType ComputeIntersectionT(
        const glm::vec4& v0, const glm::vec4& v1, ClipPlane plane
    ) noexcept {
        FloatType d0, d1;
        switch (plane) {
        case ClipPlane::LEFT:
            d0 = v0.x + v0.w;
            d1 = v1.x + v1.w;
            break;
        case ClipPlane::RIGHT:
            d0 = v0.w - v0.x;
            d1 = v1.w - v1.x;
            break;
        case ClipPlane::BOTTOM:
            d0 = v0.y + v0.w;
            d1 = v1.y + v1.w;
            break;
        case ClipPlane::TOP:
            d0 = v0.w - v0.y;
            d1 = v1.w - v1.y;
            break;
        case ClipPlane::NEAR:
            d0 = v0.z;
            d1 = v1.z;
            break;
        case ClipPlane::FAR:
            d0 = v0.w - v0.z;
            d1 = v1.w - v1.z;
            break;
        default:
            return 0.0f;
        }
        if (std::abs(d1 - d0) < 1e-6f) {
            return 0.0f;
        }
        return d0 / (d0 - d1);
    }

    /// Attribute interpolation at plane intersection (colour, UV linear in clip space)
    static ClipVertex IntersectPlane(
        const ClipVertex& v0, const ClipVertex& v1, ClipPlane plane
    ) noexcept;

    /// Sutherland–Hodgman polygon clipping against a single frustum plane (clip space)
    static InplaceVector<ClipVertex, 9> ClipAgainstPlane(
        const InplaceVector<ClipVertex, 9>& input, ClipPlane plane
    ) noexcept;

    /// Fragment shading: perspective-correct UV interpolation and base color modulation
    static Colour SampleTexture(
        const Face& face,
        const glm::vec3& bary,
        const ScreenNdcCoord& v0,
        const ScreenNdcCoord& v1,
        const ScreenNdcCoord& v2
    ) noexcept;

private:
    Window& window_;
    std::vector<FloatType> z_buffer_;

public:
    explicit Rasterizer(Window& window);

public:
    [[nodiscard]] FloatType get_depth(int x, int y) const noexcept;
    [[nodiscard]] int get_width() const noexcept { return window_.get_width(); }
    [[nodiscard]] int get_height() const noexcept { return window_.get_height(); }

    void clear() noexcept;
    void resize() noexcept;

    void wireframe(
        const Camera& camera,
        const std::vector<Face>& faces,
        const std::vector<glm::vec3>& vertices,
        double aspect_ratio
    ) noexcept;

    void rasterized(
        const Camera& camera,
        const std::vector<Face>& faces,
        const std::vector<glm::vec3>& vertices,
        const std::vector<glm::vec2>& texcoords,
        double aspect_ratio
    ) noexcept;

private:
    void face_wireframe(
        const Camera& camera,
        const Face& face,
        const std::vector<glm::vec3>& vertices,
        const std::vector<glm::vec2>& texcoords,
        double aspect_ratio
    ) noexcept;

    void face_rasterized(
        const Camera& camera,
        const Face& face,
        const std::vector<glm::vec3>& vertices,
        const std::vector<glm::vec2>& texcoords,
        double aspect_ratio
    ) noexcept;

    /**
     * @brief Clips a triangle to the view frustum using Sutherland-Hodgman algorithm.
     * 
     * @param camera The camera for transforming to clip space.
     * @param face The face defining the triangle.
     * @param vertices The list of vertex positions.
     * @param texcoords The list of texture coordinates.
     * @param aspect_ratio The aspect ratio of the viewport.
     * @return The resulting clipped polygon vertices
     */
    InplaceVector<ClipVertex, 9> clip_triangle(
        const Camera& camera,
        const Face& face,
        const std::vector<glm::vec3>& vertices,
        const std::vector<glm::vec2>& texcoords,
        double aspect_ratio
    ) noexcept;

    /// NDC [-1,1] to pixel coordinates; store 1/w for downstream perspective correction
    ScreenNdcCoord ndc_to_screen(const glm::vec3& ndc, const glm::vec2& uv, FloatType w)
        const noexcept;
};
