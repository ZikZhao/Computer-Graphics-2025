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
private:
    /** @brief Reference to the output window. */
    Window& window_;
    /** @brief Z-buffer for depth testing, storing inverse-Z values. */
    std::vector<FloatType> z_buffer_;

public:
    /**
     * @brief Constructs a rasterizer with an internal Z-buffer and a reference to a color buffer.
     * @param window Reference to the target window.
     */
    explicit Rasterizer(Window& window);

    /** @brief Clears the Z-buffer to its default value (0.0f). */
    void clear() noexcept;

    /**
     * @brief Resizes internal buffers.
     */
    void resize() noexcept;

    /**
     * @brief Draws faces in wireframe.
     * @param camera Active camera providing view and projection matrices.
     * @param faces Triangle list to render.
     * @param vertices Shared vertex positions.
     * @param aspect_ratio Display aspect ratio for projection.
     */
    void draw_model_wireframe(
        const Camera& camera,
        const std::vector<Face>& faces,
        const std::vector<glm::vec3>& vertices,
        double aspect_ratio) noexcept;

    /**
     * @brief Rasterizes textured triangles with depth testing.
     * @param camera Active camera providing view and projection matrices.
     * @param faces Triangle list to render.
     * @param vertices Shared vertex positions.
     * @param texcoords Shared texture coordinates.
     * @param aspect_ratio Display aspect ratio for projection.
     */
    void draw_model_rasterized(
        const Camera& camera,
        const std::vector<Face>& faces,
        const std::vector<glm::vec3>& vertices,
        const std::vector<glm::vec2>& texcoords,
        double aspect_ratio) noexcept;

    /**
     * @brief Returns the inverse-Z value stored at a specific pixel.
     * @param x Pixel X coordinate.
     * @param y Pixel Y coordinate.
     * @return Inverse-Z value, or 0 if the coordinates are out of bounds.
     */
    [[nodiscard]] FloatType get_depth(int x, int y) const noexcept;

    [[nodiscard]] int get_width() const noexcept { return window_.get_width(); }
    [[nodiscard]] int get_height() const noexcept { return window_.get_height(); }

private:
    // Per-face rendering helpers
    void wireframe_render(
        const Camera& camera,
        const Face& face,
        const std::vector<glm::vec3>& vertices,
        const std::vector<glm::vec2>& texcoords,
        double aspect_ratio) noexcept;
    void rasterized_render(
        const Camera& camera,
        const Face& face,
        const std::vector<glm::vec3>& vertices,
        const std::vector<glm::vec2>& texcoords,
        double aspect_ratio) noexcept;

    // Clipping logic
    static constexpr bool InsidePlane(const glm::vec4& v, ClipPlane plane) noexcept;
    static constexpr FloatType ComputeIntersectionT(const glm::vec4& v0, const glm::vec4& v1, ClipPlane plane) noexcept;
    static ClipVertex IntersectPlane(const ClipVertex& v0, const ClipVertex& v1, ClipPlane plane) noexcept;
    static InplaceVector<ClipVertex, 9> ClipAgainstPlane(
        const InplaceVector<ClipVertex, 9>& input, ClipPlane plane) noexcept;
    InplaceVector<ClipVertex, 9> clip_triangle(
        const Camera& camera,
        const Face& face,
        const std::vector<glm::vec3>& vertices,
        const std::vector<glm::vec2>& texcoords,
        double aspect_ratio) noexcept;

    // Texture sampling with perspective correction
    static Colour sample_texture(
        const Face& face,
        const glm::vec3& bary,
        const ScreenNdcCoord& v0,
        const ScreenNdcCoord& v1,
        const ScreenNdcCoord& v2) noexcept;

    // NDC to screen conversion
    ScreenNdcCoord ndc_to_screen(const glm::vec3& ndc, const glm::vec2& uv, FloatType w) const noexcept;
};
