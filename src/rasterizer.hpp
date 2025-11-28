#pragma once
#include <vector>

#include "utils.hpp"
#include "world.hpp"

class Window;

/**
 * @brief CPU rasterizer for wireframe and textured triangle rendering.
 */
class Rasterizer {
public:
    /**
     * @brief Constructs a rasterizer with an internal Z-buffer.
     * @param width Backbuffer width in pixels.
     * @param height Backbuffer height in pixels.
     */
    explicit Rasterizer(int width, int height);

    /** @brief Clears the Z-buffer to the default value. */
    void clear() noexcept;

    /**
     * @brief Resizes internal buffers.
     * @param w New width.
     * @param h New height.
     */
    void resize(int w, int h) noexcept;

    // Render a single model
    /**
     * @brief Draws faces in wireframe.
     * @param camera Active camera.
     * @param faces Triangle list to render.
     * @param vertices Shared vertex positions.
     * @param window Target window backbuffer.
     * @param aspect_ratio Display aspect ratio.
     */
    void draw_model_wireframe(
        const Camera& camera,
        const std::vector<Face>& faces,
        const std::vector<glm::vec3>& vertices,
        Window& window,
        double aspect_ratio) noexcept;

    /**
     * @brief Rasterizes textured triangles with depth testing.
     * @param camera Active camera.
     * @param faces Triangle list to render.
     * @param vertices Shared vertex positions.
     * @param texcoords Shared texture coordinates.
     * @param window Target window backbuffer.
     * @param aspect_ratio Display aspect ratio.
     */
    void draw_model_rasterized(
        const Camera& camera,
        const std::vector<Face>& faces,
        const std::vector<glm::vec3>& vertices,
        const std::vector<glm::vec2>& texcoords,
        Window& window,
        double aspect_ratio) noexcept;

    // Get depth for debugging
    /**
     * @brief Returns inverse-Z stored at a pixel.
     * @param x Pixel X.
     * @param y Pixel Y.
     * @return Inverse-Z value or 0 when out of bounds.
     */
    FloatType get_depth(int x, int y) const noexcept;

private:
    int width_;
    int height_;
    std::vector<FloatType> z_buffer_;

    // Per-face rendering helpers
    void wireframe_render(
        const Camera& camera,
        const Face& face,
        const std::vector<glm::vec3>& vertices,
        const std::vector<glm::vec2>& texcoords,
        Window& window,
        double aspect_ratio) noexcept;
    void rasterized_render(
        const Camera& camera,
        const Face& face,
        const std::vector<glm::vec3>& vertices,
        const std::vector<glm::vec2>& texcoords,
        Window& window,
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
