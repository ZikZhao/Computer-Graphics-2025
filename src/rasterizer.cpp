#include "rasterizer.hpp"

#include <algorithm>

#include "utils.hpp"

ClipVertex Rasterizer::IntersectPlane(
    const ClipVertex& v0, const ClipVertex& v1, ClipPlane plane
) noexcept {
    FloatType t = ComputeIntersectionT(v0.position_clip, v1.position_clip, plane);
    return ClipVertex{
        .position_clip = v0.position_clip * (1.0f - t) + v1.position_clip * t,
        .colour =
            Colour{
                .red = static_cast<std::uint8_t>(v0.colour.red * (1.0f - t) + v1.colour.red * t),
                .green =
                    static_cast<std::uint8_t>(v0.colour.green * (1.0f - t) + v1.colour.green * t),
                .blue = static_cast<std::uint8_t>(v0.colour.blue * (1.0f - t) + v1.colour.blue * t)
            },
        .uv = v0.uv * (1.0f - t) + v1.uv * t
    };
}

InplaceVector<ClipVertex, 9> Rasterizer::ClipAgainstPlane(
    const InplaceVector<ClipVertex, 9>& input, ClipPlane plane
) noexcept {
    InplaceVector<ClipVertex, 9> output;

    if (input.size() == 0) return output;

    for (std::size_t i = 0; i < input.size(); i++) {
        const ClipVertex& current = input[i];
        const ClipVertex& next = input[(i + 1) % input.size()];

        bool current_inside = InsidePlane(current.position_clip, plane);
        bool next_inside = InsidePlane(next.position_clip, plane);

        if (current_inside && next_inside) {
            output.push_back(next);
        } else if (current_inside && !next_inside) {
            output.push_back(IntersectPlane(current, next, plane));
        } else if (!current_inside && next_inside) {
            output.push_back(IntersectPlane(current, next, plane));
            output.push_back(next);
        }
    }

    return output;
}

Colour Rasterizer::SampleTexture(
    const Face& face,
    const glm::vec3& bary,
    const ScreenNdcCoord& v0,
    const ScreenNdcCoord& v1,
    const ScreenNdcCoord& v2
) noexcept {
    glm::vec3 base_color;

    // If textured, reconstruct UV with 1/w weighting to avoid affine distortion
    if (face.material.texture) {
        FloatType inv_w = bary.x * v0.inv_w + bary.y * v1.inv_w + bary.z * v2.inv_w;
        FloatType u = (bary.x * v0.uv.x * v0.inv_w + bary.y * v1.uv.x * v1.inv_w +
                       bary.z * v2.uv.x * v2.inv_w) /
                      inv_w;
        FloatType v = (bary.x * v0.uv.y * v0.inv_w + bary.y * v1.uv.y * v1.inv_w +
                       bary.z * v2.uv.y * v2.inv_w) /
                      inv_w;

        // Sample texture and modulate by material albedo
        Colour tex_sample = face.material.texture->sample(u, v);
        base_color = glm::vec3(
            (tex_sample.red / 255.0f) * face.material.base_color.r,
            (tex_sample.green / 255.0f) * face.material.base_color.g,
            (tex_sample.blue / 255.0f) * face.material.base_color.b
        );
    } else {
        // No texture: use flat material base color
        base_color = face.material.base_color;
    }

    // Convert to 8-bit colour for the backbuffer
    return Colour{
        .red = static_cast<std::uint8_t>(std::clamp(base_color.r * 255.0f, 0.0f, 255.0f)),
        .green = static_cast<std::uint8_t>(std::clamp(base_color.g * 255.0f, 0.0f, 255.0f)),
        .blue = static_cast<std::uint8_t>(std::clamp(base_color.b * 255.0f, 0.0f, 255.0f))
    };
}

Rasterizer::Rasterizer(Window& window)
    : window_(window), z_buffer_(window.width_ * window.height_, 0.0f) {}

void Rasterizer::clear() noexcept { z_buffer_.assign(window_.width_ * window_.height_, 0.0f); }

void Rasterizer::resize() noexcept {
    z_buffer_.resize(window_.width_ * window_.height_);
    clear();
}

FloatType Rasterizer::get_depth(int x, int y) const noexcept {
    if (x < 0 || static_cast<std::size_t>(x) >= window_.width_ || y < 0 ||
        static_cast<std::size_t>(y) >= window_.height_)
        return 0.0f;
    return z_buffer_[static_cast<std::size_t>(y) * window_.width_ + static_cast<std::size_t>(x)];
}

void Rasterizer::wireframe(
    const Camera& camera,
    const std::vector<Face>& faces,
    const std::vector<glm::vec3>& vertices,
    double aspect_ratio
) noexcept {
    for (const auto& face : faces) {
        face_wireframe(camera, face, vertices, std::vector<glm::vec2>{}, aspect_ratio);
    }
}

void Rasterizer::rasterized(
    const Camera& camera,
    const std::vector<Face>& faces,
    const std::vector<glm::vec3>& vertices,
    const std::vector<glm::vec2>& texcoords,
    double aspect_ratio
) noexcept {
    for (const auto& face : faces) {
        face_rasterized(camera, face, vertices, texcoords, aspect_ratio);
    }
}

void Rasterizer::face_wireframe(
    const Camera& camera,
    const Face& face,
    const std::vector<glm::vec3>& vertices,
    const std::vector<glm::vec2>& texcoords,
    double aspect_ratio
) noexcept {
    // Clip triangle to the frustum (Sutherland–Hodgman in clip space)
    auto clipped = clip_triangle(camera, face, vertices, texcoords, aspect_ratio);
    if (clipped.size() < 3) return;

    // Project clipped polygon to NDC and convert to screen; carry 1/w for depth
    InplaceVector<ScreenNdcCoord, 9> screen_verts;
    for (std::size_t i = 0; i < clipped.size(); i++) {
        glm::vec3 ndc = camera.clip_to_ndc(clipped[i].position_clip);
        screen_verts.push_back(ndc_to_screen(ndc, clipped[i].uv, clipped[i].position_clip.w));
    }

    // Edge rasterization using DDA with inverse-Z depth testing
    Colour colour = clipped[0].colour;
    int width = window_.width_;
    int height = window_.height_;

    for (std::size_t i = 0; i < screen_verts.size(); i++) {
        ScreenNdcCoord from = screen_verts[i];
        ScreenNdcCoord to = screen_verts[(i + 1) % screen_verts.size()];
        if (std::abs(to.x - from.x) >= std::abs(to.y - from.y)) {
            std::size_t from_x =
                std::max<std::size_t>(static_cast<std::size_t>(std::min(from.x, to.x)), 0);
            std::size_t to_x =
                std::min<std::size_t>(static_cast<std::size_t>(std::max(from.x, to.x)), width - 1);
            for (std::size_t x = from_x; x <= to_x; x++) {
                std::int64_t y = static_cast<std::int64_t>(
                    std::round(from.y + (to.y - from.y) * (x - from.x) / (to.x - from.x))
                );
                if (x >= static_cast<std::size_t>(width) || y < 0 ||
                    y >= static_cast<std::int64_t>(height))
                    continue;
                FloatType progress = (to.x == from.x) ? 0.0f
                                                      : static_cast<FloatType>(x - from.x) /
                                                            static_cast<FloatType>(to.x - from.x);
                FloatType inv_z =
                    ComputeInvZndc(progress, std::array<FloatType, 2>{from.z_ndc, to.z_ndc});
                FloatType& depth = z_buffer_[y * width + x];
                if (inv_z >= depth) {
                    depth = inv_z;
                    window_[{static_cast<int>(x), static_cast<int>(y)}] = colour;
                }
            }
        } else {
            std::size_t from_y =
                std::max<std::size_t>(static_cast<std::size_t>(std::min(from.y, to.y)), 0);
            std::size_t to_y = std::min(
                static_cast<std::size_t>(static_cast<std::size_t>(std::max(from.y, to.y))),
                static_cast<std::size_t>(height - 1)
            );
            for (std::size_t y = from_y; y <= to_y; y++) {
                std::int64_t x = static_cast<std::int64_t>(
                    std::round(from.x + (to.x - from.x) * (y - from.y) / (to.y - from.y))
                );
                if (x < 0 || x >= static_cast<std::int64_t>(width) ||
                    y >= static_cast<std::size_t>(height))
                    continue;
                FloatType progress = (to.y == from.y) ? 0.0f
                                                      : static_cast<FloatType>(y - from.y) /
                                                            static_cast<FloatType>(to.y - from.y);
                FloatType inv_z =
                    ComputeInvZndc(progress, std::array<FloatType, 2>{from.z_ndc, to.z_ndc});
                FloatType& depth = z_buffer_[y * width + x];
                if (inv_z >= depth) {
                    depth = inv_z;
                    window_[{static_cast<int>(x), static_cast<int>(y)}] = colour;
                }
            }
        }
    }
}

void Rasterizer::face_rasterized(
    const Camera& camera,
    const Face& face,
    const std::vector<glm::vec3>& vertices,
    const std::vector<glm::vec2>& texcoords,
    double aspect_ratio
) noexcept {
    // Clip triangle against all frustum planes; early out if culled
    auto clipped = clip_triangle(camera, face, vertices, texcoords, aspect_ratio);
    if (clipped.size() < 3) {
        return;
    }

    // Project to NDC and convert to screen space, tracking 1/w for perspective-correct
    // interpolation
    InplaceVector<ScreenNdcCoord, 9> screen_verts;
    for (std::size_t i = 0; i < clipped.size(); i++) {
        glm::vec3 ndc = camera.clip_to_ndc(clipped[i].position_clip);
        screen_verts.push_back(ndc_to_screen(ndc, clipped[i].uv, clipped[i].position_clip.w));
    }

    // Triangulate convex polygon fan-wise and perform two-pass scanline fill (top/bottom)
    int width = window_.width_;
    int height = window_.height_;
    for (std::size_t i = 1; i + 1 < clipped.size(); i++) {
        ScreenNdcCoord v0 = screen_verts[0];
        ScreenNdcCoord v1 = screen_verts[i];
        ScreenNdcCoord v2 = screen_verts[i + 1];
        if (v0.y > v1.y) std::swap(v0, v1);
        if (v0.y > v2.y) std::swap(v0, v2);
        if (v1.y > v2.y) std::swap(v1, v2);

        // Compute inverse slopes for edge stepping and derive scan ranges
        std::int64_t from_y = std::max<std::int64_t>(static_cast<std::int64_t>(std::ceil(v0.y)), 0);
        std::int64_t mid_y = std::min<std::int64_t>(
            static_cast<std::int64_t>(std::ceil(v1.y)), static_cast<std::int64_t>(height - 1)
        );
        std::int64_t to_y = std::min<std::int64_t>(
            static_cast<std::int64_t>(std::ceil(v2.y)), static_cast<std::int64_t>(height - 1)
        );
        FloatType inv_slope_v0v1 = (v1.y - v0.y) == 0 ? 0 : (v1.x - v0.x) / (v1.y - v0.y);
        FloatType inv_slope_v0v2 = (v2.y - v0.y) == 0 ? 0 : (v2.x - v0.x) / (v2.y - v0.y);
        FloatType inv_slope_v1v2 = (v2.y - v1.y) == 0 ? 0 : (v2.x - v1.x) / (v2.y - v1.y);

        // Upper half scanlines: barycentrics, perspective-correct texture, depth test
        for (std::int64_t y = from_y; y < mid_y; y++) {
            FloatType y_center = static_cast<FloatType>(y) + 0.5f;
            FloatType x01 = inv_slope_v0v1 * (y_center - v0.y) + v0.x;
            FloatType x02 = inv_slope_v0v2 * (y_center - v0.y) + v0.x;
            std::int64_t start_x = std::max<std::int64_t>(
                static_cast<std::int64_t>(std::floor(std::min(x01, x02))), 0
            );
            std::int64_t end_x = std::min<std::int64_t>(
                static_cast<std::int64_t>(std::ceil(std::max(x01, x02))),
                static_cast<std::int64_t>(width - 1)
            );
            for (std::int64_t x = start_x; x <= end_x; x++) {
                FloatType x_center = static_cast<FloatType>(x) + 0.5f;
                glm::vec3 bary =
                    Barycentric({v0.x, v0.y}, {v1.x, v1.y}, {v2.x, v2.y}, {x_center, y_center});
                if (bary.x >= 0.0f && bary.y >= 0.0f && bary.z >= 0.0f) {
                    Colour colour = SampleTexture(face, bary, v0, v1, v2);
                    FloatType inv_z = ComputeInvZndc(
                        std::array<FloatType, 3>{bary.x, bary.y, bary.z},
                        std::array<FloatType, 3>{v0.z_ndc, v1.z_ndc, v2.z_ndc}
                    );
                    FloatType& depth = z_buffer_[y * width + x];
                    if (inv_z > depth) {
                        depth = inv_z;
                        window_[{static_cast<int>(x), static_cast<int>(y)}] = colour;
                    }
                }
            }
        }

        // Lower half scanlines: same process with bottom edges
        for (std::int64_t y = mid_y; y <= to_y; y++) {
            FloatType y_center = static_cast<FloatType>(y) + 0.5f;
            FloatType x12 = inv_slope_v1v2 * (y_center - v1.y) + v1.x;
            FloatType x02 = inv_slope_v0v2 * (y_center - v0.y) + v0.x;
            std::int64_t start_x = std::max<std::int64_t>(
                static_cast<std::int64_t>(std::floor(std::min(x12, x02))), 0
            );
            std::int64_t end_x = std::min<std::int64_t>(
                static_cast<std::int64_t>(std::ceil(std::max(x12, x02))),
                static_cast<std::int64_t>(width - 1)
            );
            for (std::int64_t x = start_x; x <= end_x; x++) {
                FloatType x_center = static_cast<FloatType>(x) + 0.5f;
                glm::vec3 bary =
                    Barycentric({v0.x, v0.y}, {v1.x, v1.y}, {v2.x, v2.y}, {x_center, y_center});
                if (bary.x >= 0.0f && bary.y >= 0.0f && bary.z >= 0.0f) {
                    Colour colour = SampleTexture(face, bary, v0, v1, v2);
                    FloatType inv_z = ComputeInvZndc(
                        std::array<FloatType, 3>{bary.x, bary.y, bary.z},
                        std::array<FloatType, 3>{v0.z_ndc, v1.z_ndc, v2.z_ndc}
                    );
                    FloatType& depth = z_buffer_[y * width + x];
                    if (inv_z > depth) {
                        depth = inv_z;
                        window_[{static_cast<int>(x), static_cast<int>(y)}] = colour;
                    }
                }
            }
        }
    }
}

InplaceVector<ClipVertex, 9> Rasterizer::clip_triangle(
    const Camera& camera,
    const Face& face,
    const std::vector<glm::vec3>& vertices,
    const std::vector<glm::vec2>& texcoords,
    double aspect_ratio
) noexcept {
    Colour vertex_color{
        .red =
            static_cast<std::uint8_t>(std::clamp(face.material.base_color.r * 255.0f, 0.0f, 255.0f)
            ),
        .green =
            static_cast<std::uint8_t>(std::clamp(face.material.base_color.g * 255.0f, 0.0f, 255.0f)
            ),
        .blue =
            static_cast<std::uint8_t>(std::clamp(face.material.base_color.b * 255.0f, 0.0f, 255.0f))
    };

    // Gather vertex positions and optional UVs
    const glm::vec3& v0 = vertices[face.v_indices[0]];
    const glm::vec3& v1 = vertices[face.v_indices[1]];
    const glm::vec3& v2 = vertices[face.v_indices[2]];
    glm::vec2 uv0(0.0f), uv1(0.0f), uv2(0.0f);
    if (!texcoords.empty()) {
        if (face.vt_indices[0] < texcoords.size()) uv0 = texcoords[face.vt_indices[0]];
        if (face.vt_indices[1] < texcoords.size()) uv1 = texcoords[face.vt_indices[1]];
        if (face.vt_indices[2] < texcoords.size()) uv2 = texcoords[face.vt_indices[2]];
    }

    // Transform to clip space (homogeneous); attributes travel with the polygon
    InplaceVector<ClipVertex, 9> polygon = {
        ClipVertex{
            .position_clip = camera.world_to_clip(v0, aspect_ratio),
            .colour = vertex_color,
            .uv = uv0
        },
        ClipVertex{
            .position_clip = camera.world_to_clip(v1, aspect_ratio),
            .colour = vertex_color,
            .uv = uv1
        },
        ClipVertex{
            .position_clip = camera.world_to_clip(v2, aspect_ratio),
            .colour = vertex_color,
            .uv = uv2
        }
    };

    // Sequentially clip against each frustum plane; bail out if polygon fully disappears
    polygon = ClipAgainstPlane(polygon, ClipPlane::LEFT);
    if (polygon.size() < 3) return {};

    polygon = ClipAgainstPlane(polygon, ClipPlane::RIGHT);
    if (polygon.size() < 3) return {};

    polygon = ClipAgainstPlane(polygon, ClipPlane::BOTTOM);
    if (polygon.size() < 3) return {};

    polygon = ClipAgainstPlane(polygon, ClipPlane::TOP);
    if (polygon.size() < 3) return {};

    polygon = ClipAgainstPlane(polygon, ClipPlane::NEAR);
    if (polygon.size() < 3) return {};

    polygon = ClipAgainstPlane(polygon, ClipPlane::FAR);

    // Return final clipped polygon (may have 3–9 vertices)
    return polygon;
}

ScreenNdcCoord Rasterizer::ndc_to_screen(const glm::vec3& ndc, const glm::vec2& uv, FloatType w)
    const noexcept {
    return ScreenNdcCoord{
        .x = (ndc.x + 1.0f) * 0.5f * window_.width_,
        .y = (1.0f - ndc.y) * 0.5f * window_.height_,
        .z_ndc = ndc.z,
        .uv = uv,
        .inv_w = 1.0f / w
    };
}
