#include "world.hpp"

#include <algorithm>
#include <cstdlib>
#include <format>
#include <functional>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include "constants.hpp"
#include "window.hpp"

auto ParseIndexToken(const std::string& t) {
    struct IndexTriple {
        int v;
        int vt;
        int vn;
    };
    IndexTriple idx{-1, -1, -1};
    if (t.find('/') != std::string::npos) {
        std::vector<std::string> parts;
        std::stringstream ss(t);
        std::string p;
        while (std::getline(ss, p, '/')) parts.push_back(p);
        if (!parts.empty() && !parts[0].empty()) idx.v = std::stoi(parts[0]);
        if (parts.size() >= 2 && !parts[1].empty()) idx.vt = std::stoi(parts[1]);
        if (parts.size() >= 3 && !parts[2].empty()) idx.vn = std::stoi(parts[2]);
    } else {
        bool digits_only = !t.empty() && std::all_of(t.begin(), t.end(), [](char c) {
            return c >= '0' && c <= '9';
        });
        if (digits_only) idx.v = std::stoi(t);
    }
    return idx;
}

ColourHDR ColourHDR::FromSRGB(const Colour& srgb, FloatType gamma) noexcept {
    auto to_linear = [gamma](std::uint8_t component) -> FloatType {
        FloatType normalized = component / 255.0f;
        if (gamma == 1.0f) return normalized;
        return std::pow(normalized, gamma);
    };
    return ColourHDR{
        .red = to_linear(srgb.red), .green = to_linear(srgb.green), .blue = to_linear(srgb.blue)
    };
}

EnvironmentMap::EnvironmentMap(
    std::size_t width, std::size_t height, std::vector<ColourHDR> data, FloatType intensity
) noexcept
    : width_(width), height_(height), data_(std::move(data)), intensity_(intensity) {}

ColourHDR EnvironmentMap::sample(const glm::vec3& direction) const noexcept {
    if (!is_loaded()) return ColourHDR{.red = 0.0f, .green = 0.0f, .blue = 0.0f};
    FloatType theta = std::atan2(direction.x, -direction.z);
    FloatType phi = std::asin(std::clamp(direction.y, -1.0f, 1.0f));
    FloatType u = (theta / (2.0f * std::numbers::pi)) + 0.5f;
    FloatType v = (phi / std::numbers::pi) + 0.5f;
    std::size_t x = static_cast<std::size_t>(
        std::clamp(u * static_cast<FloatType>(width_), 0.0f, static_cast<FloatType>(width_ - 1))
    );
    std::size_t y = static_cast<std::size_t>(
        std::clamp(v * static_cast<FloatType>(height_), 0.0f, static_cast<FloatType>(height_ - 1))
    );
    return data_[y * width_ + x] * intensity_;
}

glm::mat3 Camera::orientation() const noexcept {
    // Derive camera basis (right, up, forward) from yaw/pitch; then apply roll about forward
    FloatType cos_pitch = std::cos(pitch_);
    FloatType sin_pitch = std::sin(pitch_);
    FloatType cos_yaw = std::cos(yaw_);
    FloatType sin_yaw = std::sin(yaw_);
    glm::vec3 f(sin_yaw * cos_pitch, sin_pitch, -cos_yaw * cos_pitch);
    glm::vec3 world_up(0.0f, 1.0f, 0.0f);
    glm::vec3 r = glm::normalize(glm::cross(f, world_up));
    glm::vec3 u = glm::normalize(glm::cross(r, f));
    FloatType c = std::cos(roll_);
    FloatType s = std::sin(roll_);
    auto rotate_axis = [&](const glm::vec3& v) -> glm::vec3 {
        return v * c + glm::cross(f, v) * s + f * glm::dot(f, v) * (1.0f - c);
    };
    r = rotate_axis(r);
    u = rotate_axis(u);
    glm::mat3 o;
    o[0] = r;
    o[1] = u;
    o[2] = f;
    return o;
}

glm::vec3 Camera::forward() const noexcept {
    FloatType cos_pitch = std::cos(pitch_);
    FloatType sin_pitch = std::sin(pitch_);
    FloatType cos_yaw = std::cos(yaw_);
    FloatType sin_yaw = std::sin(yaw_);
    return glm::vec3(sin_yaw * cos_pitch, sin_pitch, -cos_yaw * cos_pitch);
}

glm::vec3 Camera::right() const noexcept {
    glm::mat3 o = orientation();
    return glm::normalize(o[0]);
}

glm::vec3 Camera::up() const noexcept {
    glm::mat3 o = orientation();
    return glm::normalize(o[1]);
}

glm::vec4 Camera::world_to_clip(const glm::vec3& vertex) const noexcept {
    // Transform world-space vertex to camera view, then project to homogeneous clip
    glm::vec3 view_vector = vertex - position_;
    glm::mat3 view_rotation = glm::transpose(orientation());
    glm::vec3 view_space = view_rotation * view_vector;
    FloatType w = view_space.z;
    double fov_rad = glm::radians(Constant::FOV);
    double tan_half_fov = std::tan(fov_rad / 2.0);
    FloatType x_ndc = view_space.x / (view_space.z * tan_half_fov * aspect_ratio_);
    FloatType y_ndc = view_space.y / (view_space.z * tan_half_fov);
    FloatType z_ndc = (Constant::FarPlane * (view_space.z - Constant::NearPlane)) /
                      ((view_space.z) * (Constant::FarPlane - Constant::NearPlane));
    return glm::vec4(x_ndc * w, y_ndc * w, z_ndc * w, w);
}

glm::vec3 Camera::clip_to_ndc(const glm::vec4& clip) const noexcept {
    if (std::abs(clip.w) < 1e-6f) {
        return glm::vec3(0.0f, 0.0f, -1.0f);
    }
    // Divide by w to get NDC; guard above avoids division by near-zero
    return glm::vec3(clip) / clip.w;
}

void Camera::start_orbiting() noexcept {
    orbit_radius_ = glm::length(position_);
    is_orbiting_ = true;
}

void Camera::orbiting() noexcept {
    if (is_orbiting_) {
        yaw_ += Constant::OrbitAngleIncrement;
        position_ = -forward() * orbit_radius_;
    }
}

void Camera::stop_orbiting() noexcept { is_orbiting_ = false; }

void Camera::rotate(FloatType delta_yaw, FloatType delta_pitch) noexcept {
    yaw_ += delta_yaw;
    pitch_ += delta_pitch;
    pitch_ = std::clamp(pitch_, -Constant::MaxPitch, Constant::MaxPitch);
}

void Camera::roll(FloatType delta_roll) noexcept { roll_ += delta_roll; }

void Camera::move(
    FloatType forward_delta, FloatType right_delta, FloatType up_delta, FloatType dt
) noexcept {
    position_ += forward() * (forward_delta * dt);
    position_ += right() * (right_delta * dt);
    position_ += up() * (up_delta * dt);
}

std::pair<glm::vec3, glm::vec3> Camera::generate_ray(
    int pixel_x, int pixel_y, int screen_width, int screen_height
) const noexcept {
    FloatType u = (static_cast<FloatType>(pixel_x) + 0.5f) / static_cast<FloatType>(screen_width);
    FloatType v = (static_cast<FloatType>(pixel_y) + 0.5f) / static_cast<FloatType>(screen_height);
    return generate_ray_uv(u, v, screen_width, screen_height);
}

std::pair<glm::vec3, glm::vec3> Camera::generate_ray_uv(
    FloatType u, FloatType v, int screen_width, int screen_height
) const noexcept {
    // Map pixel UV to view-space direction using pinhole; then rotate to world space
    FloatType ndc_x = u * 2.0f - 1.0f;
    FloatType ndc_y = 1.0f - v * 2.0f;
    double fov_rad = glm::radians(Constant::FOV);
    double tan_half_fov = std::tan(fov_rad / 2.0);
    FloatType view_x =
        ndc_x * static_cast<FloatType>(tan_half_fov) * static_cast<FloatType>(aspect_ratio_);
    FloatType view_y = ndc_y * static_cast<FloatType>(tan_half_fov);
    FloatType view_z = 1.0f;
    glm::vec3 ray_dir_view(view_x, view_y, view_z);
    glm::vec3 ray_dir_world = orientation() * ray_dir_view;
    return {position_, glm::normalize(ray_dir_world)};
}

FloatType EnvironmentMap::ComputeAutoExposure(const std::vector<ColourHDR>& hdr_data) noexcept {
    if (hdr_data.empty()) return 1.0f;

    // Collect non-zero luminance values from all pixels
    std::vector<FloatType> luminances;
    luminances.reserve(hdr_data.size());

    for (const auto& pixel : hdr_data) {
        FloatType luma = Luminance(glm::vec3(pixel.red, pixel.green, pixel.blue));
        if (luma > 0.0f) {
            luminances.push_back(luma);
        }
    }

    if (luminances.empty()) return 1.0f;

    // Sort luminances to compute percentiles
    std::sort(luminances.begin(), luminances.end());
    std::size_t percentile_90_idx = static_cast<std::size_t>(luminances.size() * 0.90f);
    FloatType percentile_90 = luminances[percentile_90_idx];

    // Compute log-average luminance (geometric mean)
    FloatType log_lum_sum = 0.0f;
    for (FloatType lum : luminances) {
        log_lum_sum += std::log(lum + 1e-6f);
    }
    FloatType avg_log_lum = std::exp(log_lum_sum / luminances.size());

    // Blend exposure from log-average (70%) and 90th percentile (30%)
    FloatType exposure_from_avg = Constant::TargetMiddleGray / (avg_log_lum + 1e-6f);
    FloatType exposure_from_p90 = Constant::TargetMiddleGray / (percentile_90 + 1e-6f);

    FloatType auto_exposure = 0.7f * exposure_from_avg + 0.3f * exposure_from_p90;
    auto_exposure = std::clamp(auto_exposure, 0.05f, 2.0f);

    return auto_exposure * 0.5f;
}

bool BVHAccelerator::IntersectAABB(
    const glm::vec3& ro, const glm::vec3& rd, const AABB& box, FloatType tmax
) noexcept {
    glm::vec3 inv = glm::vec3(1.0f) / rd;
    glm::vec3 t0 = (box.min - ro) * inv;
    glm::vec3 t1 = (box.max - ro) * inv;
    glm::vec3 tmin = glm::min(t0, t1);
    glm::vec3 tmaxv = glm::max(t0, t1);
    FloatType t_enter = std::max(std::max(tmin.x, tmin.y), tmin.z);
    FloatType t_exit = std::min(std::min(tmaxv.x, tmaxv.y), tmaxv.z);
    return t_enter <= t_exit && t_exit >= 0.0f && t_enter <= tmax;
}

BVHAccelerator BVHAccelerator::Build(
    const std::vector<Face>& faces,
    const std::vector<glm::vec3>& vertices,
    const std::vector<glm::vec3>& normals,
    const std::vector<glm::vec3>& vertex_normals,
    const std::vector<glm::vec2>& texcoords
) {
    BVHAccelerator bvh(vertices, normals, vertex_normals, texcoords);
    if (faces.empty()) return bvh;

    // Initialize triangle index array and precompute per-triangle data
    bvh.tri_indices_.resize(faces.size());
    std::iota(bvh.tri_indices_.begin(), bvh.tri_indices_.end(), 0);

    struct Cent {
        glm::vec3 c;  // Centroid
        AABB b;       // Bounding box
    };
    std::vector<Cent> data(faces.size());
    for (std::size_t i = 0; i < faces.size(); ++i) {
        const Face& f = faces[i];
        const glm::vec3& v0 = vertices[f.v_indices[0]];
        const glm::vec3& v1 = vertices[f.v_indices[1]];
        const glm::vec3& v2 = vertices[f.v_indices[2]];
        glm::vec3 mn = glm::min(glm::min(v0, v1), v2);
        glm::vec3 mx = glm::max(glm::max(v0, v1), v2);
        data[i] = Cent{.c = (v0 + v1 + v2) / 3.0f, .b = AABB{mn, mx}};
    }

    // Define SAH cost model parameters
    bvh.nodes_.clear();
    auto surface_area = [](const AABB& box) -> FloatType {
        glm::vec3 extent = box.max - box.min;
        return 2.0f * (extent.x * extent.y + extent.y * extent.z + extent.z * extent.x);
    };

    // Recursive BVH construction with SAH-based splitting
    std::function<int(int, int)> build_rec = [&](int start, int end) -> int {
        // Compute bounding box for all triangles and centroid bounds
        AABB box{
            glm::vec3(std::numeric_limits<float>::infinity()),
            glm::vec3(-std::numeric_limits<float>::infinity())
        };
        AABB cbox{box.min, box.max};
        for (int i = start; i < end; ++i) {
            box.min = glm::min(box.min, data[bvh.tri_indices_[i]].b.min);
            box.max = glm::max(box.max, data[bvh.tri_indices_[i]].b.max);
            cbox.min = glm::min(cbox.min, data[bvh.tri_indices_[i]].c);
            cbox.max = glm::max(cbox.max, data[bvh.tri_indices_[i]].c);
        }

        // Create leaf node initially
        int count = end - start;
        int node_index = (int)bvh.nodes_.size();
        bvh.nodes_.push_back(
            BVHNode{.box = box, .left = -1, .right = -1, .start = start, .count = count}
        );
        if (count <= Constant::BVHLeafThreshold) {
            return node_index;
        }

        // Find best split using SAH across all 3 axes
        FloatType best_cost = std::numeric_limits<FloatType>::infinity();
        int best_axis = -1;
        int best_split = -1;
        glm::vec3 extent = cbox.max - cbox.min;
        FloatType parent_area = surface_area(box);

        for (int axis = 0; axis < 3; ++axis) {
            if (extent[axis] < 1e-6f) continue;

            // Bin triangles into buckets along this axis
            struct Bucket {
                int count = 0;
                AABB bounds{
                    glm::vec3(std::numeric_limits<float>::infinity()),
                    glm::vec3(-std::numeric_limits<float>::infinity())
                };
            };
            std::array<Bucket, Constant::SAHBuckets> buckets;
            for (int i = start; i < end; ++i) {
                FloatType centroid = data[bvh.tri_indices_[i]].c[axis];
                int bucket_idx = static_cast<int>(
                    Constant::SAHBuckets * ((centroid - cbox.min[axis]) / extent[axis])
                );
                bucket_idx = std::clamp(bucket_idx, 0, Constant::SAHBuckets - 1);
                buckets[bucket_idx].count++;
                buckets[bucket_idx].bounds.min =
                    glm::min(buckets[bucket_idx].bounds.min, data[bvh.tri_indices_[i]].b.min);
                buckets[bucket_idx].bounds.max =
                    glm::max(buckets[bucket_idx].bounds.max, data[bvh.tri_indices_[i]].b.max);
            }

            // Sweep through bucket splits and compute SAH cost
            for (int split = 0; split < Constant::SAHBuckets - 1; ++split) {
                AABB left_box{
                    glm::vec3(std::numeric_limits<float>::infinity()),
                    glm::vec3(-std::numeric_limits<float>::infinity())
                };
                int left_count = 0;
                for (int i = 0; i <= split; ++i) {
                    if (buckets[i].count > 0) {
                        left_box.min = glm::min(left_box.min, buckets[i].bounds.min);
                        left_box.max = glm::max(left_box.max, buckets[i].bounds.max);
                        left_count += buckets[i].count;
                    }
                }
                AABB right_box{
                    glm::vec3(std::numeric_limits<float>::infinity()),
                    glm::vec3(-std::numeric_limits<float>::infinity())
                };
                int right_count = 0;
                for (int i = split + 1; i < Constant::SAHBuckets; ++i) {
                    if (buckets[i].count > 0) {
                        right_box.min = glm::min(right_box.min, buckets[i].bounds.min);
                        right_box.max = glm::max(right_box.max, buckets[i].bounds.max);
                        right_count += buckets[i].count;
                    }
                }
                if (left_count == 0 || right_count == 0) continue;
                FloatType cost = Constant::SAHTraversalCost;
                cost += (surface_area(left_box) / parent_area) * left_count *
                        Constant::SAHIntersectionCost;
                cost += (surface_area(right_box) / parent_area) * right_count *
                        Constant::SAHIntersectionCost;
                if (cost < best_cost) {
                    best_cost = cost;
                    best_axis = axis;
                    best_split = split;
                }
            }
        }

        // If splitting is not beneficial, keep as leaf
        FloatType leaf_cost = count * Constant::SAHIntersectionCost;
        if (best_cost >= leaf_cost || best_axis == -1) {
            return node_index;
        }

        // Partition triangles based on best split
        auto mid_iter = std::partition(
            bvh.tri_indices_.begin() + start,
            bvh.tri_indices_.begin() + end,
            [&](int idx) {
                FloatType centroid = data[idx].c[best_axis];
                int bucket_idx = static_cast<int>(
                    Constant::SAHBuckets * ((centroid - cbox.min[best_axis]) / extent[best_axis])
                );
                bucket_idx = std::clamp(bucket_idx, 0, Constant::SAHBuckets - 1);
                return bucket_idx <= best_split;
            }
        );
        int mid = static_cast<int>(mid_iter - bvh.tri_indices_.begin());
        if (mid == start || mid == end) {
            mid = (start + end) / 2;  // Fallback to median split
        }

        // Recursively build children and convert to interior node
        int left = build_rec(start, mid);
        int right = build_rec(mid, end);
        bvh.nodes_[node_index].left = left;
        bvh.nodes_[node_index].right = right;
        bvh.nodes_[node_index].count = 0;  // Mark as interior node
        return node_index;
    };

    build_rec(0, (int)faces.size());
    return bvh;
}

BVHAccelerator::BVHAccelerator(
    const std::vector<glm::vec3>& vertices,
    const std::vector<glm::vec3>& normals,
    const std::vector<glm::vec3>& vertex_normals,
    const std::vector<glm::vec2>& texcoords
)
    : vertices_(&vertices),
      texcoords_(&texcoords),
      normals_(&normals),
      normals_by_vertex_(&vertex_normals) {}

RayTriangleIntersection BVHAccelerator::intersect(
    const glm::vec3& ro, const glm::vec3& rd, const std::vector<Face>& faces
) const noexcept {
    // Initialize result with no intersection
    RayTriangleIntersection closest;
    closest.distanceFromCamera = std::numeric_limits<FloatType>::infinity();
    closest.triangleIndex = static_cast<std::size_t>(-1);
    if (nodes_.empty()) return closest;

    // Iterative BVH traversal using explicit stack
    std::vector<int> stack;
    stack.reserve(64);
    stack.push_back(0);

    while (!stack.empty()) {
        int ni = stack.back();
        stack.pop_back();
        const BVHNode& n = nodes_[ni];

        // Skip node if ray doesn't intersect its bounding box
        if (!IntersectAABB(ro, rd, n.box, closest.distanceFromCamera)) continue;

        if (n.count == 0) {
            // Interior node: push children onto stack
            stack.push_back(n.left);
            stack.push_back(n.right);
        } else {
            // Leaf node: test all triangles
            for (int i = 0; i < n.count; ++i) {
                int tri_index = tri_indices_[n.start + i];
                const Face& face = faces[tri_index];
                FloatType t, u, v;
                const glm::vec3& v0 = (*vertices_)[face.v_indices[0]];
                const glm::vec3& v1 = (*vertices_)[face.v_indices[1]];
                const glm::vec3& v2 = (*vertices_)[face.v_indices[2]];
                if (IntersectRayTriangle(ro, rd, v0, v1, v2, t, u, v) &&
                    t < closest.distanceFromCamera) {
                    closest.distanceFromCamera = t;
                    closest.intersectionPoint = ro + rd * t;
                    closest.triangleIndex = tri_index;
                    closest.u = u;
                    closest.v = v;
                    FloatType w = 1.0f - u - v;
                    auto fetch_normal = [&](int idx) -> glm::vec3 {
                        std::uint32_t ni = face.vn_indices[idx];
                        if (normals_ && ni != std::numeric_limits<std::uint32_t>::max() &&
                            ni < normals_->size())
                            return (*normals_)[ni];
                        std::uint32_t vi = face.v_indices[idx];
                        if (normals_by_vertex_ && vi < normals_by_vertex_->size() &&
                            glm::length((*normals_by_vertex_)[vi]) > 0.001f)
                            return (*normals_by_vertex_)[vi];
                        return face.face_normal;
                    };
                    glm::vec3 n0 = fetch_normal(0);
                    glm::vec3 n1 = fetch_normal(1);
                    glm::vec3 n2 = fetch_normal(2);
                    glm::vec3 interpolated_normal = glm::normalize(w * n0 + u * n1 + v * n2);
                    if (glm::dot(interpolated_normal, -rd) < 0.0f) {
                        interpolated_normal = -interpolated_normal;
                    }
                    closest.normal = interpolated_normal;

                    // Interpolate texture coordinates
                    glm::vec2 uv_coord(0.0f);
                    if (texcoords_) {
                        glm::vec2 uv0(0.0f), uv1(0.0f), uv2(0.0f);
                        if (face.vt_indices[0] < texcoords_->size())
                            uv0 = (*texcoords_)[face.vt_indices[0]];
                        if (face.vt_indices[1] < texcoords_->size())
                            uv1 = (*texcoords_)[face.vt_indices[1]];
                        if (face.vt_indices[2] < texcoords_->size())
                            uv2 = (*texcoords_)[face.vt_indices[2]];
                        uv_coord = uv0 * w + uv1 * u + uv2 * v;
                    }
                    // Apply normal mapping if a normal map is present
                    if (face.material.normal_map && texcoords_) {
                        // Sample tangent-space normal from the normal map
                        glm::vec3 tangent_normal =
                            face.material.normal_map->sample(uv_coord.x, uv_coord.y);

                        // Compute TBN matrix from triangle edges and UV deltas
                        glm::vec2 uv0(0.0f), uv1(0.0f), uv2(0.0f);
                        if (face.vt_indices[0] < texcoords_->size())
                            uv0 = (*texcoords_)[face.vt_indices[0]];
                        if (face.vt_indices[1] < texcoords_->size())
                            uv1 = (*texcoords_)[face.vt_indices[1]];
                        if (face.vt_indices[2] < texcoords_->size())
                            uv2 = (*texcoords_)[face.vt_indices[2]];

                        glm::vec3 edge1 = v1 - v0;
                        glm::vec3 edge2 = v2 - v0;
                        glm::vec2 deltaUV1 = uv1 - uv0;
                        glm::vec2 deltaUV2 = uv2 - uv0;

                        FloatType det = deltaUV1.x * deltaUV2.y - deltaUV2.x * deltaUV1.y;

                        if (std::abs(det) > 1e-6f) {
                            FloatType inv_det = 1.0f / det;
                            glm::vec3 tangent =
                                glm::normalize((edge1 * deltaUV2.y - edge2 * deltaUV1.y) * inv_det);
                            glm::vec3 bitangent =
                                glm::normalize((edge2 * deltaUV1.x - edge1 * deltaUV2.x) * inv_det);

                            // Gram-Schmidt orthogonalize tangent and bitangent with respect to
                            // normal
                            glm::vec3 N = interpolated_normal;
                            tangent = glm::normalize(tangent - N * glm::dot(N, tangent));
                            bitangent = glm::normalize(
                                bitangent - N * glm::dot(N, bitangent) -
                                tangent * glm::dot(tangent, bitangent)
                            );

                            // TBN matrix transforms from tangent space to world space
                            glm::mat3 TBN(tangent, bitangent, N);

                            // Transform tangent-space normal to world space
                            closest.normal = glm::normalize(TBN * tangent_normal);
                        }
                    }

                    // Sample diffuse texture if present
                    if (face.material.texture) {
                        Colour tex_sample = face.material.texture->sample(uv_coord.x, uv_coord.y);
                        closest.color = glm::vec3(
                            (tex_sample.red / 255.0f) * face.material.base_color.r,
                            (tex_sample.green / 255.0f) * face.material.base_color.g,
                            (tex_sample.blue / 255.0f) * face.material.base_color.b
                        );
                    } else {
                        closest.color = face.material.base_color;
                    }

                    // Store geometric normal for backface detection
                    closest.geom_normal = face.face_normal;
                    closest.front_face = glm::dot(rd, closest.geom_normal) < 0.0f;
                    if (!closest.front_face) {
                        closest.geom_normal = -closest.geom_normal;
                    }
                }
            }
        }
    }
    return closest;
}

glm::vec3 BVHAccelerator::transmittance(
    const glm::vec3& point, const glm::vec3& light_pos, const std::vector<Face>& faces
) const noexcept {
    // Compute shadow ray direction and distance
    glm::vec3 to_light = light_pos - point;
    FloatType light_distance = glm::length(to_light);
    glm::vec3 light_dir = to_light / light_distance;
    if (nodes_.empty()) return glm::vec3(1.0f, 1.0f, 1.0f);

    glm::vec3 trans(1.0f, 1.0f, 1.0f);
    std::vector<int> stack;
    stack.reserve(64);
    stack.push_back(0);

    // Collect all intersections along the shadow ray
    struct Intersection {
        FloatType t;
        const Face* face;
        FloatType u;
        FloatType v;
    };
    std::vector<Intersection> intersections;

    // BVH traversal to find all intersecting surfaces
    while (!stack.empty()) {
        int ni = stack.back();
        stack.pop_back();
        const BVHNode& n = nodes_[ni];
        if (!IntersectAABB(point, light_dir, n.box, light_distance)) continue;
        if (n.count == 0) {
            stack.push_back(n.left);
            stack.push_back(n.right);
        } else {
            for (int i = 0; i < n.count; ++i) {
                int tri_index = tri_indices_[n.start + i];
                const Face& face = faces[tri_index];
                FloatType t, u, v;
                const glm::vec3& v0 = (*vertices_)[face.v_indices[0]];
                const glm::vec3& v1 = (*vertices_)[face.v_indices[1]];
                const glm::vec3& v2 = (*vertices_)[face.v_indices[2]];
                bool hit = IntersectRayTriangle(point, light_dir, v0, v1, v2, t, u, v);
                if (hit && t > Constant::RayEpsilon && t < (light_distance - 1e-4f)) {
                    intersections.push_back(Intersection{.t = t, .face = &face, .u = u, .v = v});
                }
            }
        }
    }

    // Sort intersections by distance for proper volumetric absorption
    std::sort(
        intersections.begin(),
        intersections.end(),
        [](const Intersection& a, const Intersection& b) { return a.t < b.t; }
    );

    // Accumulate transmittance through transparent surfaces
    for (std::size_t i = 0; i < intersections.size(); ++i) {
        const auto& isect = intersections[i];
        const Material& mat = isect.face->material;

        // Opaque surface blocks all light
        if (mat.tw < 0.01f) {
            return glm::vec3(0.0f, 0.0f, 0.0f);
        }

        // Apply Beer-Lambert absorption for distance between entry/exit
        if (i + 1 < intersections.size()) {
            const auto& next_isect = intersections[i + 1];
            FloatType distance_in_medium = next_isect.t - isect.t;
            glm::vec3 sigma_a = EffectiveSigmaA(mat.sigma_a, mat.base_color, mat.td);
            glm::vec3 absorption = BeerLambert(sigma_a, distance_in_medium);
            trans = trans * absorption;
            i++;
        }

        // Early exit if fully absorbed
        if (trans.r < 0.001f && trans.g < 0.001f && trans.b < 0.001f) {
            return glm::vec3(0.0f, 0.0f, 0.0f);
        }
    }
    return trans;
}

World::World(const std::vector<std::string>& filenames) {
    // Scene ingestion: load HDR environment, OBJ models or text scenes; then merge into a single
    // geometry store
    for (const auto& filename : filenames) {
        std::filesystem::path path(filename);
        if (std::filesystem::is_directory(path)) {
            throw std::runtime_error("File does not exist: " + filename);
        }
        std::string ext = path.extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
            return std::tolower(c);
        });

        if (ext == ".hdr") {
            load_hdr_env_map(path);
        } else if (ext == ".txt") {
            // Text scene
            Model model;
            parse_txt(model, path);
            models_.emplace_back(std::move(model));
        } else if (ext == ".obj") {
            // Standard OBJ model
            Model model;
            parse_obj(model, path);
            models_.emplace_back(std::move(model));
        } else {
            throw std::runtime_error("Unsupported file format: " + filename);
        }
    }
    merge_models();
}

void World::parse_obj(Model& model, const std::filesystem::path& path) {
    auto current_obj = model.objects.end();
    std::ifstream file(path);
    if (!file.is_open()) throw std::runtime_error("Could not open file: " + path.string());

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;
        if (type == "mtllib") {
            // Material library -- preload materials referenced by objects
            std::string relative_path;
            iss >> relative_path;
            std::string material_filename =
                (std::filesystem::path(path).parent_path() / relative_path).string();
            parse_mtl(model, std::move(material_filename));
        } else if (type == "o") {
            // Object boundary -- new mesh group with its own material/shading state
            std::string name;
            iss >> name;
            model.objects.emplace_back(name);
            current_obj = std::prev(model.objects.end());
        } else if (type == "usemtl") {
            // Bind material to current object; preserve shading mode
            assert(current_obj != model.objects.end());
            std::string colour_name;
            iss >> colour_name;
            assert(model.materials.find(colour_name) != model.materials.end());
            auto prev_shading = current_obj->material.shading;
            current_obj->material = model.materials[colour_name];
            current_obj->material.shading = prev_shading;
        } else if (type == "shading" || type == "Shading") {
            // Select shading model (Flat/Gouraud/Phong)
            std::string mode;
            iss >> mode;
            if (current_obj != model.objects.end()) {
                if (mode == "Flat") {
                    current_obj->material.shading = Material::Shading::FLAT;
                } else if (mode == "Gouraud") {
                    current_obj->material.shading = Material::Shading::GOURAUD;
                } else if (mode == "Phong") {
                    current_obj->material.shading = Material::Shading::PHONG;
                }
            }
        } else if (type == "v") {
            // Vertex position
            assert(current_obj != model.objects.end());
            FloatType x, y, z;
            iss >> x >> y >> z;
            model.vertices.emplace_back(x, y, z);
        } else if (type == "vt") {
            // Texture coordinate
            FloatType u, v;
            iss >> u >> v;
            model.texture_coords.emplace_back(u, v);
        } else if (type == "vn") {
            // Vertex normal (normalized)
            FloatType x, y, z;
            iss >> x >> y >> z;
            model.vertex_normals.emplace_back(glm::normalize(glm::vec3(x, y, z)));
        } else if (type == "f") {
            // Triangle face -- parse indices (v/vt/vn) and create Face with material
            assert(current_obj != model.objects.end());
            glm::vec3 vertice[3];
            std::uint32_t vt_indices[3];
            int vi_idx[3];
            int normal_indices[3];
            bool has_normals = false;
            for (int i = 0; i < 3; i++) {
                int vertex_index;
                char slash;
                iss >> vertex_index >> slash;
                vertice[i] = model.vertices[vertex_index - 1];
                vi_idx[i] = vertex_index - 1;
                vt_indices[i] = std::numeric_limits<std::uint32_t>::max();
                normal_indices[i] = -1;
                if (int c = iss.peek(); c >= '0' && c <= '9') {
                    int tex_idx;
                    iss >> tex_idx;
                    if (tex_idx > 0) {
                        vt_indices[i] = static_cast<std::uint32_t>(tex_idx - 1);
                    }
                }
                if (iss.peek() == '/') {
                    iss >> slash;
                    if (int c = iss.peek(); c >= '0' && c <= '9') {
                        int normal_idx;
                        iss >> normal_idx;
                        normal_indices[i] = normal_idx - 1;
                        has_normals = true;
                    }
                }
            }
            Face new_face{
                .v_indices =
                    {static_cast<std::uint32_t>(vi_idx[0]),
                     static_cast<std::uint32_t>(vi_idx[1]),
                     static_cast<std::uint32_t>(vi_idx[2])},
                .vt_indices = {vt_indices[0], vt_indices[1], vt_indices[2]},
                .vn_indices =
                    {(has_normals && normal_indices[0] >= 0)
                         ? static_cast<std::uint32_t>(normal_indices[0])
                         : std::numeric_limits<std::uint32_t>::max(),
                     (has_normals && normal_indices[1] >= 0)
                         ? static_cast<std::uint32_t>(normal_indices[1])
                         : std::numeric_limits<std::uint32_t>::max(),
                     (has_normals && normal_indices[2] >= 0)
                         ? static_cast<std::uint32_t>(normal_indices[2])
                         : std::numeric_limits<std::uint32_t>::max()},
                .material = current_obj->material,
                .face_normal = glm::vec3(0.0f)
            };
            current_obj->faces.emplace_back(std::move(new_face));
        }
    }

    compute_all_face_normals(model);
    flatten_model_faces(model);
}

void World::parse_mtl(Model& model, const std::filesystem::path& path) {
    auto current_material = model.materials.end();
    std::ifstream file(path);
    if (!file.is_open()) throw std::runtime_error("Could not open material file: " + path.string());

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;
        if (type == "newmtl") {
            std::string name;
            iss >> name;
            assert(model.materials.find(name) == model.materials.end());
            current_material = model.materials.emplace(name, Material{}).first;
        } else if (type == "Kd") {
            assert(current_material != model.materials.end());
            FloatType r, g, b;
            iss >> r >> g >> b;
            current_material->second.base_color = glm::vec3(r, g, b);
        } else if (type == "Ns") {
            assert(current_material != model.materials.end());
            FloatType ns;
            iss >> ns;
            current_material->second.shininess = ns;
        } else if (type == "Mt") {
            assert(current_material != model.materials.end());
            FloatType metallic_value;
            iss >> metallic_value;
            current_material->second.metallic = std::clamp(metallic_value, 0.0f, 1.0f);
        } else if (type == "ior") {
            assert(current_material != model.materials.end());
            FloatType ior_value;
            iss >> ior_value;
            current_material->second.ior = std::max(1.0f, ior_value);
        } else if (type == "td") {
            assert(current_material != model.materials.end());
            FloatType td_value;
            iss >> td_value;
            current_material->second.td = std::max(0.0f, td_value);
        } else if (type == "tw") {
            assert(current_material != model.materials.end());
            FloatType tw_value;
            iss >> tw_value;
            current_material->second.tw = std::clamp(tw_value, 0.0f, 1.0f);
        } else if (type == "Ke") {
            assert(current_material != model.materials.end());
            FloatType r, g, b;
            iss >> r >> g >> b;
            current_material->second.emission =
                glm::vec3(std::max(0.0f, r), std::max(0.0f, g), std::max(0.0f, b));
        } else if (type == "sigma_a") {
            assert(current_material != model.materials.end());
            FloatType r, g, b;
            iss >> r >> g >> b;
            current_material->second.sigma_a =
                glm::vec3(std::max(0.0f, r), std::max(0.0f, g), std::max(0.0f, b));
        } else if (type == "map_Kd") {
            assert(current_material != model.materials.end());
            std::string texture_filename;
            iss >> texture_filename;
            texture_filename = (path.parent_path() / texture_filename).string();
            current_material->second.texture =
                std::make_shared<Texture>(load_texture(texture_filename));
        } else if (type == "map_Bump") {
            assert(current_material != model.materials.end());
            std::string normal_filename;
            iss >> normal_filename;
            normal_filename = (path.parent_path() / normal_filename).string();
            current_material->second.normal_map =
                std::make_shared<NormalMap>(load_normal_map(normal_filename));
        }
    }
}

void World::parse_txt(Model& model, const std::filesystem::path& path) {
    auto current_obj = model.objects.end();
    std::ifstream file(path);
    if (!file.is_open()) throw std::runtime_error("Could not open file: " + path.string());

    std::string line;
    auto current_material = model.materials.end();
    int vertex_offset = 0;
    int tex_offset = 0;
    int normal_offset = 0;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;
        if (type == "Environ") {
            // Environment map HDR file
            std::string hdr;
            iss >> hdr;
            std::string hdr_path = (std::filesystem::path(path).parent_path() / hdr).string();
            load_hdr_env_map(hdr_path);
        } else if (type == "Include") {
            // Include another scene file
            std::string rel;
            iss >> rel;
            if (!rel.empty()) {
                std::string inc_path = (std::filesystem::path(path).parent_path() / rel).string();
                parse_txt(model, inc_path);
            }
        } else if (type == "Material") {
            // Begin a new material block
            std::string name;
            iss >> name;
            current_material = model.materials.emplace(name, Material{}).first;
        } else if (type == "Colour") {
            // Albedo (diffuse base color)
            if (current_material != model.materials.end()) {
                FloatType r, g, b;
                iss >> r >> g >> b;
                current_material->second.base_color = glm::vec3(r, g, b);
            }
        } else if (type == "Metallic") {
            // Metallic factor for specular reflection weighting [0,1]
            if (current_material != model.materials.end()) {
                FloatType m;
                iss >> m;
                current_material->second.metallic = std::clamp(m, 0.0f, 1.0f);
            }
        } else if (type == "IOR") {
            // Index of refraction for dielectrics (>= 1.0)
            if (current_material != model.materials.end()) {
                FloatType i;
                iss >> i;
                current_material->second.ior = std::max(1.0f, i);
            }
        } else if (type == "AtDistance") {
            // Absorption length td; used to derive sigma_a when not provided
            if (current_material != model.materials.end()) {
                FloatType td;
                iss >> td;
                current_material->second.td = std::max(0.0f, td);
            }
        } else if (type == "TransmissionWeight") {
            // Transmission weight for mixing reflection/refraction
            if (current_material != model.materials.end()) {
                FloatType tw;
                iss >> tw;
                current_material->second.tw = std::clamp(tw, 0.0f, 1.0f);
            }
        } else if (type == "Emission") {
            // Emissive colour
            if (current_material != model.materials.end()) {
                FloatType r, g, b;
                iss >> r >> g >> b;
                current_material->second.emission =
                    glm::vec3(std::max(0.0f, r), std::max(0.0f, g), std::max(0.0f, b));
            }
        } else if (type == "Texture") {
            // Bind a PPM texture to the material
            if (current_material != model.materials.end()) {
                std::string tex_name;
                iss >> tex_name;
                std::string texture_filename =
                    (std::filesystem::path(path).parent_path() / tex_name).string();
                current_material->second.texture =
                    std::make_shared<Texture>(load_texture(texture_filename));
            }
        } else if (type == "NormalMap") {
            // Bind a PPM normal map to the material
            if (current_material != model.materials.end()) {
                std::string normal_name;
                iss >> normal_name;
                std::string normal_filename =
                    (std::filesystem::path(path).parent_path() / normal_name).string();
                current_material->second.normal_map =
                    std::make_shared<NormalMap>(load_normal_map(normal_filename));
            }
        } else if (type == "Object") {
            // Begin an object block; record offsets for local indexing within this file
            std::string rest;
            std::getline(iss, rest);
            std::string name = rest;
            if (!name.empty() && name[0] == '#') {
                name.erase(0, 1);
            }
            while (!name.empty() && (name[0] == ' ' || name[0] == '\t')) name.erase(0, 1);
            model.objects.emplace_back(name);
            current_obj = std::prev(model.objects.end());
            vertex_offset = static_cast<int>(model.vertices.size());
            tex_offset = static_cast<int>(model.texture_coords.size());
            normal_offset = static_cast<int>(model.vertex_normals.size());
        } else if (type == "Use") {
            // Assign current material to the object
            if (current_obj != model.objects.end()) {
                std::string mat_name;
                iss >> mat_name;
                auto it = model.materials.find(mat_name);
                if (it != model.materials.end()) {
                    auto prev_shading = current_obj->material.shading;
                    current_obj->material = it->second;
                    current_obj->material.shading = prev_shading;
                }
            }
        } else if (type == "Shading") {
            // Select shading method (Flat/Gouraud/Phong)
            if (current_obj != model.objects.end()) {
                std::string mode;
                iss >> mode;
                if (mode == "Flat") {
                    current_obj->material.shading = Material::Shading::FLAT;
                } else if (mode == "Gouraud") {
                    current_obj->material.shading = Material::Shading::GOURAUD;
                } else if (mode == "Phong") {
                    current_obj->material.shading = Material::Shading::PHONG;
                }
            }
        } else if (type == "Vertex") {
            // Vertex position with optional inline normal ("x y z / nx ny nz")
            if (current_obj != model.objects.end()) {
                FloatType x, y, z;
                iss >> x >> y >> z;
                model.vertices.emplace_back(x, y, z);
                model.vertex_normals_by_vertex.emplace_back(glm::vec3(0.0f));
                char c;
                if (iss >> c) {
                    if (c == '/') {
                        FloatType xn, yn, zn;
                        iss >> xn >> yn >> zn;
                        model.vertex_normals_by_vertex.back() =
                            glm::normalize(glm::vec3(xn, yn, zn));
                    } else {
                        iss.putback(c);
                    }
                }
            }
        } else if (type == "TextureCoord") {
            // UV coordinate
            FloatType u, v;
            iss >> u >> v;
            model.texture_coords.emplace_back(u, v);
        } else if (type == "Normal") {
            // Vertex normal (normalized)
            FloatType x, y, z;
            iss >> x >> y >> z;
            model.vertex_normals.emplace_back(glm::normalize(glm::vec3(x, y, z)));
        } else if (type == "Face") {
            // Triangle face -- parse indices (v/vt/vn) and create Face with material
            if (current_obj == model.objects.end()) continue;
            std::string tok[3];
            iss >> tok[0] >> tok[1] >> tok[2];
            glm::vec3 vpos[3];
            std::uint32_t vt_out[3] = {
                std::numeric_limits<std::uint32_t>::max(),
                std::numeric_limits<std::uint32_t>::max(),
                std::numeric_limits<std::uint32_t>::max()
            };
            int vi_out[3] = {0, 0, 0};
            int vn_out[3] = {-1, -1, -1};
            bool has_vn = false;
            for (int i = 0; i < 3; ++i) {
                const std::string& t = tok[i];
                const auto& [v_i, vt_i, vn_i] = ParseIndexToken(t);
                if (vn_i >= 0) has_vn = true;
                int v_global = vertex_offset + (v_i >= 0 ? v_i : 0);
                vpos[i] = model.vertices[v_global];
                vi_out[i] = v_global;
                if (vt_i >= 0) {
                    int vt_global = tex_offset + vt_i;
                    vt_out[i] = static_cast<std::uint32_t>(vt_global);
                }
                if (vn_i >= 0) {
                    int vn_global = normal_offset + vn_i;
                    vn_out[i] = vn_global;
                }
            }
            // If normals are absent, defer to per-vertex cached normals when available; mark
            // missing as -1
            if (!has_vn) {
                for (int i = 0; i < 3; ++i) {
                    int vidx = vi_out[i];
                    if (vidx >= 0 &&
                        static_cast<std::size_t>(vidx) < model.vertex_normals_by_vertex.size()) {
                        if (glm::length(model.vertex_normals_by_vertex[vidx]) > 0.001f) {
                            vn_out[i] = -1;
                        }
                    }
                }
                for (int i = 0; i < 3; ++i) {
                    if (vn_out[i] < 0) {
                        vn_out[i] = -1;
                    }
                }
            }
            Face new_face{
                .v_indices =
                    {static_cast<std::uint32_t>(vi_out[0]),
                     static_cast<std::uint32_t>(vi_out[1]),
                     static_cast<std::uint32_t>(vi_out[2])},
                .vt_indices = {vt_out[0], vt_out[1], vt_out[2]},
                .vn_indices =
                    {vn_out[0] >= 0 ? static_cast<std::uint32_t>(vn_out[0])
                                    : std::numeric_limits<std::uint32_t>::max(),
                     vn_out[1] >= 0 ? static_cast<std::uint32_t>(vn_out[1])
                                    : std::numeric_limits<std::uint32_t>::max(),
                     vn_out[2] >= 0 ? static_cast<std::uint32_t>(vn_out[2])
                                    : std::numeric_limits<std::uint32_t>::max()},
                .material = current_obj->material,
                .face_normal = glm::vec3(0.0f)
            };
            model.objects.back().faces.emplace_back(std::move(new_face));
        }
    }

    compute_all_face_normals(model);
    flatten_model_faces(model);
}

void World::load_hdr_env_map(const std::filesystem::path& path) {
    int width, height, channels;
    float* data = stbi_loadf(path.string().c_str(), &width, &height, &channels, 3);

    if (data) {
        std::vector<ColourHDR> hdr_data;
        hdr_data.reserve(width * height);

        for (int i = 0; i < width * height; ++i) {
            hdr_data.emplace_back(data[i * 3 + 0], data[i * 3 + 1], data[i * 3 + 2]);
        }

        stbi_image_free(data);

        FloatType auto_intensity = EnvironmentMap::ComputeAutoExposure(hdr_data);
        env_map_ = EnvironmentMap(width, height, std::move(hdr_data), auto_intensity);
    } else {
        throw std::runtime_error("Failed to load HDR environment map: " + path.string());
    }
}

Texture World::load_texture(const std::filesystem::path& path) {
    auto ppm = ReadPPM(path);
    std::vector<Colour> texture_data;
    texture_data.resize(ppm.width * ppm.height);
    for (std::size_t i = 0; i < ppm.width * ppm.height; i++) {
        int red = ppm.stream.get();
        int green = ppm.stream.get();
        int blue = ppm.stream.get();
        if (red == EOF || green == EOF || blue == EOF)
            throw std::runtime_error(
                "Unexpected end of file while reading texture: " + path.string()
            );
        texture_data[i] = Colour{
            .red = static_cast<std::uint8_t>(red),
            .green = static_cast<std::uint8_t>(green),
            .blue = static_cast<std::uint8_t>(blue)
        };
    }
    return Texture(ppm.width, ppm.height, std::move(texture_data));
}

NormalMap World::load_normal_map(const std::filesystem::path& path) {
    auto ppm = ReadPPM(path);
    std::vector<glm::vec3> normal_data;
    normal_data.resize(ppm.width * ppm.height);
    for (std::size_t i = 0; i < ppm.width * ppm.height; i++) {
        int red = ppm.stream.get();
        int green = ppm.stream.get();
        int blue = ppm.stream.get();
        if (red == EOF || green == EOF || blue == EOF)
            throw std::runtime_error(
                "Unexpected end of file while reading normal map: " + path.string()
            );

        // Apply inverse gamma correction (sRGB to linear) before converting to tangent-space normal
        FloatType r_linear = std::pow(static_cast<FloatType>(red) / 255.0f, Constant::DefaultGamma);
        FloatType g_linear =
            std::pow(static_cast<FloatType>(green) / 255.0f, Constant::DefaultGamma);
        FloatType b_linear =
            std::pow(static_cast<FloatType>(blue) / 255.0f, Constant::DefaultGamma);

        // Convert from [0, 1] to [-1, 1] for tangent-space normal
        FloatType nx = r_linear * 2.0f - 1.0f;
        FloatType ny = g_linear * 2.0f - 1.0f;
        FloatType nz = b_linear * 2.0f - 1.0f;

        // OpenGL convention
        normal_data[i] = glm::normalize(glm::vec3(nx, ny, nz));
    }
    return NormalMap(ppm.width, ppm.height, std::move(normal_data));
}

void World::compute_all_face_normals(Model& model) {
    // Compute geometric normals per face from vertex positions;
    // used for flat shading and backface tests
    for (auto& obj : model.objects) {
        for (auto& f : obj.faces) {
            const glm::vec3& v0 = model.vertices[f.v_indices[0]];
            const glm::vec3& v1 = model.vertices[f.v_indices[1]];
            const glm::vec3& v2 = model.vertices[f.v_indices[2]];
            f.face_normal = FaceNormal(v0, v1, v2);
        }
    }
}

void World::flatten_model_faces(Model& model) {
    model.all_faces.clear();
    model.all_faces.reserve(std::accumulate(
        model.objects.begin(),
        model.objects.end(),
        std::size_t(0),
        [](std::size_t sum, const Object& obj) { return sum + obj.faces.size(); }
    ));

    for (const auto& object : model.objects) {
        model.all_faces.insert(model.all_faces.end(), object.faces.begin(), object.faces.end());
    }
}

void World::merge_models() noexcept {
    reserve_global_buffers();
    for (const auto& model : models_) {
        append_model_data(model);
    }
    collect_emissive_faces();

    std::cout << std::format(
        "[World] All models loaded: Vertex: {} | Vertex Normal: {} | Face: {} | Environment: {}\n",
        all_vertices_.size(),
        all_vertex_normals_.size() + all_vertex_normals_by_vertex_.size(),
        all_faces_.size(),
        env_map_.is_loaded()
    );

    accelerator_ = BVHAccelerator::Build(
        all_faces_,
        all_vertices_,
        all_vertex_normals_,
        all_vertex_normals_by_vertex_,
        all_texcoords_
    );

    models_.clear();
}

void World::reserve_global_buffers() noexcept {
    all_vertices_.clear();
    all_texcoords_.clear();
    all_vertex_normals_.clear();
    all_vertex_normals_by_vertex_.clear();
    all_faces_.clear();

    std::size_t num_v = 0, num_vt = 0, num_vn = 0, num_vnbv = 0, num_f = 0;
    for (const auto& m : models_) {
        num_v += m.vertices.size();
        num_vt += m.texture_coords.size();
        num_vn += m.vertex_normals.size();
        num_vnbv += m.vertex_normals_by_vertex.size();
        num_f += m.all_faces.size();
    }
    all_vertices_.reserve(num_v);
    all_texcoords_.reserve(num_vt);
    all_vertex_normals_.reserve(num_vn);
    all_vertex_normals_by_vertex_.reserve(num_vnbv);
    all_faces_.reserve(num_f);
}

void World::append_model_data(const Model& model) noexcept {
    std::uint32_t v_offset = static_cast<std::uint32_t>(all_vertices_.size());
    std::uint32_t vt_offset = static_cast<std::uint32_t>(all_texcoords_.size());
    std::uint32_t vn_offset = static_cast<std::uint32_t>(all_vertex_normals_.size());
    std::uint32_t vnbv_offset = static_cast<std::uint32_t>(all_vertex_normals_by_vertex_.size());

    all_vertices_.insert(all_vertices_.end(), model.vertices.begin(), model.vertices.end());
    all_texcoords_.insert(
        all_texcoords_.end(), model.texture_coords.begin(), model.texture_coords.end()
    );
    all_vertex_normals_.insert(
        all_vertex_normals_.end(), model.vertex_normals.begin(), model.vertex_normals.end()
    );
    all_vertex_normals_by_vertex_.insert(
        all_vertex_normals_by_vertex_.end(),
        model.vertex_normals_by_vertex.begin(),
        model.vertex_normals_by_vertex.end()
    );

    for (const auto& local_face : model.all_faces) {
        Face global_face = local_face;

        for (int k = 0; k < 3; ++k) {
            global_face.v_indices[k] += v_offset;
            if (global_face.vt_indices[k] != std::numeric_limits<std::uint32_t>::max()) {
                global_face.vt_indices[k] += vt_offset;
            }
            if (global_face.vn_indices[k] != std::numeric_limits<std::uint32_t>::max()) {
                global_face.vn_indices[k] += vn_offset;
            }
        }
        all_faces_.push_back(std::move(global_face));
    }
}

void World::collect_emissive_faces() noexcept {
    for (const auto& f : all_faces_) {
        if (glm::length(f.material.emission) > 1e-6f) {
            emissive_faces_.push_back(&f);
        }
    }
}
