#pragma once
#include <fstream>
#include <vector>
#include <sstream>
#include <algorithm>
#include <functional>
#include <numeric>
#include <map>
#include <stdexcept>
#include <memory>
#include <chrono>
#include <array>
#include <filesystem>
#include <numbers>
#include <optional>
#include <barrier>
#include <atomic>
#include <thread>
#include "utils.hpp"

// Clipping planes for frustum
enum class ClipPlane {
    LEFT,   // x >= -w
    RIGHT,  // x <= w
    BOTTOM, // y >= -w
    TOP,    // y <= w
    NEAR,   // z >= 0
    FAR     // z <= w
};

struct Colour {
    std::uint8_t red;
    std::uint8_t green;
    std::uint8_t blue;
    constexpr operator std::uint32_t () const {
        return (255 << 24) + (red << 16) + (green << 8) + blue;
    }
};

struct ColourHDR {
    FloatType red;
    FloatType green;
    FloatType blue;
    
    static ColourHDR from_srgb(const Colour& srgb, FloatType gamma) noexcept {
        auto to_linear = [gamma](std::uint8_t component) -> FloatType {
            FloatType normalized = component / 255.0f;
            if (gamma == 1.0f) return normalized;  // No gamma correction
            return std::pow(normalized, gamma);
        };
        return ColourHDR{
            .red = to_linear(srgb.red),
            .green = to_linear(srgb.green),
            .blue = to_linear(srgb.blue)
        };
    }
    
    constexpr ColourHDR operator*(FloatType scalar) const noexcept {
        return ColourHDR{ .red = red * scalar, .green = green * scalar, .blue = blue * scalar };
    }
    
    constexpr ColourHDR operator*(const ColourHDR& other) const noexcept {
        return ColourHDR{ .red = red * other.red, .green = green * other.green, .blue = blue * other.blue };
    }
    
    constexpr ColourHDR operator+(const ColourHDR& other) const noexcept {
        return ColourHDR{ .red = red + other.red, .green = green + other.green, .blue = blue + other.blue };
    }
};

struct Face;
struct RayTriangleIntersection;
class BvhAccelerator; // forward declaration

class EnvironmentMap {
public:
    static FloatType ComputeAutoExposure(const std::vector<ColourHDR>& hdr_data) noexcept;

private:
    std::size_t width_;
    std::size_t height_;
    std::vector<ColourHDR> data_;
    FloatType intensity_;

public:
    EnvironmentMap() noexcept : width_(0), height_(0), intensity_(1.0f) {}
    EnvironmentMap(std::size_t w, std::size_t h, std::vector<ColourHDR> data, FloatType intensity = 0.3f) noexcept
        : width_(w), height_(h), data_(std::move(data)), intensity_(intensity) {}
    constexpr bool is_loaded() const noexcept { return width_ > 0 && height_ > 0; }
    ColourHDR sample(const glm::vec3& direction) const noexcept {
        if (!is_loaded()) return ColourHDR{ .red = 0.0f, .green = 0.0f, .blue = 0.0f };
        FloatType theta = std::atan2(direction.x, -direction.z);
        FloatType phi = std::asin(std::clamp(direction.y, -1.0f, 1.0f));
        FloatType u = (theta / (2.0f * std::numbers::pi)) + 0.5f;
        FloatType v = (phi / std::numbers::pi) + 0.5f;
        std::size_t x = static_cast<std::size_t>(std::clamp(u * static_cast<FloatType>(width_), 0.0f, static_cast<FloatType>(width_ - 1)));
        std::size_t y = static_cast<std::size_t>(std::clamp(v * static_cast<FloatType>(height_), 0.0f, static_cast<FloatType>(height_ - 1)));
        return data_[y * width_ + x] * intensity_;
    }
};

struct RayTriangleIntersection {
    glm::vec3 intersectionPoint;
    FloatType distanceFromCamera;
    glm::vec3 color;  // Linear RGB color (0-1 range)
    std::size_t triangleIndex;
    glm::vec3 normal;
    glm::vec3 geom_normal;
    bool front_face;
    FloatType u; // Barycentric coordinate for vertex 1
    FloatType v; // Barycentric coordinate for vertex 2
};

class Texture {
    std::size_t width_;
    std::size_t height_;
    std::vector<Colour> data_;
public:
    constexpr Texture(std::size_t w, std::size_t h, std::vector<Colour> data)
        : width_(w), height_(h), data_(std::move(data)) {}
    constexpr Colour sample(FloatType u, FloatType v) const {
        std::size_t ix = static_cast<std::size_t>(std::clamp(u * static_cast<FloatType>(width_ - 1), 0.0f, static_cast<FloatType>(width_ - 1)));
        std::size_t iy = static_cast<std::size_t>(std::clamp(v * static_cast<FloatType>(height_ - 1), 0.0f, static_cast<FloatType>(height_ - 1)));
        return data_[iy * width_ + ix];
    }
};

struct Material {
    std::shared_ptr<Texture> texture;
    glm::vec3 base_color = glm::vec3(1.0f, 1.0f, 1.0f);  // Color multiplier for texture (or base color if no texture)
    FloatType shininess = 64.0f;
    FloatType metallic = 0.0f;
    enum class Shading { FLAT, GOURAUD, PHONG } shading = Shading::FLAT;
    FloatType ior = 1.0f;           // Index of refraction (1.0 = no refraction, 1.5 = glass)
    FloatType td = 1.0f;            // Transmission Depth: The distance at which the base_color is reached
    FloatType tw = 0.0f;            // Transparency weight (0 = opaque, 1 = fully transparent)
    glm::vec3 sigma_a = glm::vec3(0.0f, 0.0f, 0.0f);  // Absorption coefficient for Beer-Lambert (optional override)
    glm::vec3 emission = glm::vec3(0.0f);             // Emissive radiance (area light)
};

struct ClipVertex {
    glm::vec4 position_clip;
    Colour colour;
    glm::vec2 uv;
};

struct ScreenNdcCoord {
    FloatType x;
    FloatType y;
    FloatType z_ndc;
    glm::vec2 uv;  // Texture coordinates (already divided by w)
    FloatType inv_w;  // 1/w for perspective correction
};

class Camera {
public:
    static constexpr auto OrbitInterval = std::chrono::seconds(1) / 60;
    static constexpr double FOV = 45.0;
    static constexpr double NearPlane = 0.001;
    static constexpr double FarPlane = 100.0;
public:
    glm::vec3 position_ = { 0.0f, 0.0f, 12.0f };
    FloatType yaw_ = 0.0f;      // Horizontal rotation (around world Y axis)
    FloatType pitch_ = 0.0f;    // Vertical rotation (clamped to ±89 degrees)
    FloatType roll_ = 0.0f;
    glm::vec3 orbit_target_ = { 0.0f, 0.0f, 0.0f };
    bool is_orbiting_ = false;
    FloatType orbit_radius_ = 0.0f;
    Camera() noexcept = default;
    glm::vec4 world_to_clip(const glm::vec3& vertex, double aspect_ratio) const noexcept;
    glm::vec3 clip_to_ndc(const glm::vec4& clip) const noexcept;
    glm::mat3 orientation() const noexcept;
    glm::vec3 forward() const noexcept;
    glm::vec3 right() const noexcept;
    glm::vec3 up() const noexcept;
    void start_orbiting(glm::vec3 target) noexcept;
    void orbiting() noexcept;
    void stop_orbiting() noexcept;
    void rotate(FloatType delta_yaw, FloatType delta_pitch) noexcept;
    void roll(FloatType delta_roll) noexcept;
    void move(FloatType forward_delta, FloatType right_delta, FloatType up_delta, FloatType dt) noexcept;
    void set_position(const glm::vec3& pos) noexcept { position_ = pos; }
    void set_yaw(FloatType y) noexcept { yaw_ = y; }
    void set_pitch(FloatType p) noexcept { pitch_ = p; }
    void set_roll(FloatType r) noexcept { roll_ = r; }
    std::pair<glm::vec3, glm::vec3> generate_ray(int pixel_x, int pixel_y, int screen_width, int screen_height, double aspect_ratio) const noexcept;
    std::pair<glm::vec3, glm::vec3> generate_ray_uv(FloatType u, FloatType v, int screen_width, int screen_height, double aspect_ratio) const noexcept;
};

// Optimization: Store per-vertex indices instead of duplicating vertex data.
// Reduces Face footprint (e.g., ~100B → ~40B depending on Material/padding),
// which improves cache residency and memory bandwidth during BVH traversal and
// ray–triangle intersection. The triangles stream indices; actual positions,
// uvs and normals are fetched from shared arrays on demand.
struct Face {
    std::array<std::uint32_t, 3> v_indices;
    std::array<std::uint32_t, 3> vt_indices;
    std::array<std::uint32_t, 3> vn_indices;
    Material material;
    glm::vec3 face_normal;
};

struct Object {
    std::string name;
    Material material;
    std::vector<Face> faces;
};

class Model {
private:
    std::vector<Object> objects_;
    std::vector<Face> all_faces_;  // Flattened faces for efficient iteration (cached)
    std::map<std::string, Material> materials_;
    std::vector<glm::vec3> vertices_;
    std::vector<glm::vec2> texture_coords_;  // vt coordinates from OBJ
    std::vector<glm::vec3> vertex_normals_;  // vn normals from OBJ
    std::vector<glm::vec3> vertex_normals_by_vertex_;
public:
    Model() noexcept = default;
    void load_file(std::string filename);
    void load_scene_txt(std::string filename);
    friend class SceneLoader;
    const std::vector<Face>& all_faces() const noexcept { return all_faces_; }
    const std::vector<glm::vec3>& vertices() const noexcept { return vertices_; }
    const std::vector<glm::vec2>& texture_coords() const noexcept { return texture_coords_; }
    const std::vector<glm::vec3>& vertex_normals() const noexcept { return vertex_normals_; }
    const std::vector<glm::vec3>& vertex_normals_by_vertex() const noexcept { return vertex_normals_by_vertex_; }
    private:
    void load_materials(std::string filename);
    Texture load_texture(std::string filename);
    void cache_faces() noexcept;
    void compute_face_normals() noexcept;
};

class BvhAccelerator {
public:
    struct AABB {
        glm::vec3 min;
        glm::vec3 max;
    };
    struct BVHNode {
        AABB box;
        int left;
        int right;
        int start;
        int count;
    };
private:
    std::vector<int> tri_indices_;
    std::vector<BVHNode> nodes_;
    const std::vector<glm::vec3>* vertices_ = nullptr;
    const std::vector<glm::vec2>* texcoords_ = nullptr;
    const std::vector<glm::vec3>* normals_ = nullptr;
    const std::vector<glm::vec3>* normals_by_vertex_ = nullptr;
public:
    BvhAccelerator() noexcept = default;
    bool empty() const noexcept { return nodes_.empty(); }
    void set_vertices(const std::vector<glm::vec3>& verts) noexcept { vertices_ = &verts; }
    void set_texcoords(const std::vector<glm::vec2>& uvs) noexcept { texcoords_ = &uvs; }
    void set_normals(const std::vector<glm::vec3>& norms) noexcept { normals_ = &norms; }
    void set_normals_by_vertex(const std::vector<glm::vec3>& norms) noexcept { normals_by_vertex_ = &norms; }
    void build(const std::vector<Face>& faces) noexcept {
        if (!vertices_) return;
        tri_indices_.resize(faces.size());
        std::iota(tri_indices_.begin(), tri_indices_.end(), 0);
        struct Cent { glm::vec3 c; AABB b; };
        std::vector<Cent> data(faces.size());
        for (std::size_t i = 0; i < faces.size(); ++i) {
            const Face& f = faces[i];
            const glm::vec3& v0 = (*vertices_)[f.v_indices[0]];
            const glm::vec3& v1 = (*vertices_)[f.v_indices[1]];
            const glm::vec3& v2 = (*vertices_)[f.v_indices[2]];
            glm::vec3 mn = glm::min(glm::min(v0, v1), v2);
            glm::vec3 mx = glm::max(glm::max(v0, v1), v2);
            AABB b{mn, mx};
            glm::vec3 c = (v0 + v1 + v2) / 3.0f;
            data[i] = Cent{ .c = c, .b = b };
        }
        nodes_.clear();
        auto surface_area = [](const AABB& box) -> FloatType {
            glm::vec3 extent = box.max - box.min;
            return 2.0f * (extent.x * extent.y + extent.y * extent.z + extent.z * extent.x);
        };
        constexpr FloatType traversal_cost = 1.0f;
        constexpr FloatType intersection_cost = 1.0f;
        constexpr int sah_buckets = 12;
        constexpr int leaf_threshold = 4;
        std::function<int(int, int)> build_rec = [&](int start, int end) -> int {
            AABB box{glm::vec3(std::numeric_limits<float>::infinity()), glm::vec3(-std::numeric_limits<float>::infinity())};
            AABB cbox{box.min, box.max};
            for (int i = start; i < end; ++i) {
                box.min = glm::min(box.min, data[tri_indices_[i]].b.min);
                box.max = glm::max(box.max, data[tri_indices_[i]].b.max);
                cbox.min = glm::min(cbox.min, data[tri_indices_[i]].c);
                cbox.max = glm::max(cbox.max, data[tri_indices_[i]].c);
            }
            int count = end - start;
            int node_index = (int)nodes_.size();
            nodes_.push_back(BVHNode{ .box   = box, .left  = -1, .right = -1, .start = start, .count = count });
            if (count <= leaf_threshold) {
                return node_index;
            }
            FloatType best_cost = std::numeric_limits<FloatType>::infinity();
            int best_axis = -1;
            int best_split = -1;
            glm::vec3 extent = cbox.max - cbox.min;
            FloatType parent_area = surface_area(box);
            for (int axis = 0; axis < 3; ++axis) {
                if (extent[axis] < 1e-6f) continue;
                struct Bucket { int count = 0; AABB bounds{glm::vec3(std::numeric_limits<float>::infinity()), glm::vec3(-std::numeric_limits<float>::infinity())}; };
                std::array<Bucket, sah_buckets> buckets;
                for (int i = start; i < end; ++i) {
                    FloatType centroid = data[tri_indices_[i]].c[axis];
                    int bucket_idx = static_cast<int>(sah_buckets * ((centroid - cbox.min[axis]) / extent[axis]));
                    bucket_idx = std::clamp(bucket_idx, 0, sah_buckets - 1);
                    buckets[bucket_idx].count++;
                    buckets[bucket_idx].bounds.min = glm::min(buckets[bucket_idx].bounds.min, data[tri_indices_[i]].b.min);
                    buckets[bucket_idx].bounds.max = glm::max(buckets[bucket_idx].bounds.max, data[tri_indices_[i]].b.max);
                }
                for (int split = 0; split < sah_buckets - 1; ++split) {
                    AABB left_box{glm::vec3(std::numeric_limits<float>::infinity()), glm::vec3(-std::numeric_limits<float>::infinity())};
                    int left_count = 0;
                    for (int i = 0; i <= split; ++i) {
                        if (buckets[i].count > 0) {
                            left_box.min = glm::min(left_box.min, buckets[i].bounds.min);
                            left_box.max = glm::max(left_box.max, buckets[i].bounds.max);
                            left_count += buckets[i].count;
                        }
                    }
                    AABB right_box{glm::vec3(std::numeric_limits<float>::infinity()), glm::vec3(-std::numeric_limits<float>::infinity())};
                    int right_count = 0;
                    for (int i = split + 1; i < sah_buckets; ++i) {
                        if (buckets[i].count > 0) {
                            right_box.min = glm::min(right_box.min, buckets[i].bounds.min);
                            right_box.max = glm::max(right_box.max, buckets[i].bounds.max);
                            right_count += buckets[i].count;
                        }
                    }
                    if (left_count == 0 || right_count == 0) continue;
                    FloatType cost = traversal_cost;
                    cost += (surface_area(left_box) / parent_area) * left_count * intersection_cost;
                    cost += (surface_area(right_box) / parent_area) * right_count * intersection_cost;
                    if (cost < best_cost) {
                        best_cost = cost;
                        best_axis = axis;
                        best_split = split;
                    }
                }
            }
            FloatType leaf_cost = count * intersection_cost;
            if (best_cost >= leaf_cost || best_axis == -1) {
                return node_index;
            }
            auto mid_iter = std::partition(tri_indices_.begin() + start, tri_indices_.begin() + end, [&](int idx) {
                FloatType centroid = data[idx].c[best_axis];
                int bucket_idx = static_cast<int>(sah_buckets * ((centroid - cbox.min[best_axis]) / extent[best_axis]));
                bucket_idx = std::clamp(bucket_idx, 0, sah_buckets - 1);
                return bucket_idx <= best_split;
            });
            int mid = static_cast<int>(mid_iter - tri_indices_.begin());
            if (mid == start || mid == end) {
                mid = (start + end) / 2;
            }
            int left = build_rec(start, mid);
            int right = build_rec(mid, end);
            nodes_[node_index].left = left;
            nodes_[node_index].right = right;
            nodes_[node_index].count = 0;
            return node_index;
        };
        build_rec(0, (int)faces.size());
    }
    static bool IntersectAabb(const glm::vec3& ro, const glm::vec3& rd, const AABB& box, FloatType tmax) noexcept {
        glm::vec3 inv = glm::vec3(1.0f) / rd;
        glm::vec3 t0 = (box.min - ro) * inv;
        glm::vec3 t1 = (box.max - ro) * inv;
        glm::vec3 tmin = glm::min(t0, t1);
        glm::vec3 tmaxv = glm::max(t0, t1);
        FloatType t_enter = std::max(std::max(tmin.x, tmin.y), tmin.z);
        FloatType t_exit = std::min(std::min(tmaxv.x, tmaxv.y), tmaxv.z);
        return t_enter <= t_exit && t_exit >= 0.0f && t_enter <= tmax;
    }
    RayTriangleIntersection intersect(const glm::vec3& ro, const glm::vec3& rd, const std::vector<Face>& faces) const noexcept {
        RayTriangleIntersection closest;
        closest.distanceFromCamera = std::numeric_limits<FloatType>::infinity();
        closest.triangleIndex = static_cast<std::size_t>(-1);
        if (nodes_.empty()) return closest;
        std::vector<int> stack;
        stack.reserve(64);
        stack.push_back(0);
        while (!stack.empty()) {
            int ni = stack.back();
            stack.pop_back();
            const BVHNode& n = nodes_[ni];
            if (!IntersectAabb(ro, rd, n.box, closest.distanceFromCamera)) continue;
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
                    if (IntersectRayTriangle(ro, rd, v0, v1, v2, t, u, v) && t < closest.distanceFromCamera) {
                        closest.distanceFromCamera = t;
                        closest.intersectionPoint = ro + rd * t;
                        closest.triangleIndex = tri_index;
                        closest.u = u;
                        closest.v = v;
                        FloatType w = 1.0f - u - v;
                        auto fetch_normal = [&](int idx) -> glm::vec3 {
                            std::uint32_t ni = face.vn_indices[idx];
                            if (normals_ && ni != std::numeric_limits<std::uint32_t>::max() && ni < normals_->size()) return (*normals_)[ni];
                            std::uint32_t vi = face.v_indices[idx];
                            if (normals_by_vertex_ && vi < normals_by_vertex_->size() && glm::length((*normals_by_vertex_)[vi]) > 0.001f) return (*normals_by_vertex_)[vi];
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
                        glm::vec2 uv_coord(0.0f);
                        if (texcoords_) {
                            glm::vec2 uv0(0.0f), uv1(0.0f), uv2(0.0f);
                            if (face.vt_indices[0] < texcoords_->size()) uv0 = (*texcoords_)[face.vt_indices[0]];
                            if (face.vt_indices[1] < texcoords_->size()) uv1 = (*texcoords_)[face.vt_indices[1]];
                            if (face.vt_indices[2] < texcoords_->size()) uv2 = (*texcoords_)[face.vt_indices[2]];
                            uv_coord = uv0 * w + uv1 * u + uv2 * v;
                        }
                        if (face.material.texture) {
                            Colour tex_sample = face.material.texture->sample(uv_coord.x, uv_coord.y);
                            closest.color = glm::vec3((tex_sample.red / 255.0f) * face.material.base_color.r, (tex_sample.green / 255.0f) * face.material.base_color.g, (tex_sample.blue / 255.0f) * face.material.base_color.b);
                        } else {
                            closest.color = face.material.base_color;
                        }
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
    glm::vec3 transmittance(const glm::vec3& point, const glm::vec3& light_pos, const std::vector<Face>& faces) const noexcept {
        glm::vec3 to_light = light_pos - point;
        FloatType light_distance = glm::length(to_light);
        glm::vec3 light_dir = to_light / light_distance;
        if (nodes_.empty()) return glm::vec3(1.0f, 1.0f, 1.0f);
        glm::vec3 trans(1.0f, 1.0f, 1.0f);
        std::vector<int> stack;
        stack.reserve(64);
        stack.push_back(0);
        constexpr FloatType min_t = 0.001f;
        struct Intersection { FloatType t; const Face* face; FloatType u; FloatType v; };
        std::vector<Intersection> intersections;
        while (!stack.empty()) {
            int ni = stack.back();
            stack.pop_back();
            const BVHNode& n = nodes_[ni];
            if (!IntersectAabb(point, light_dir, n.box, light_distance)) continue;
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
                    if (hit && t > min_t && t < (light_distance - 1e-4f)) {
                        intersections.push_back(Intersection{ .t = t, .face = &face, .u = u, .v = v });
                    }
                }
            }
        }
        std::sort(intersections.begin(), intersections.end(), [](const Intersection& a, const Intersection& b) { return a.t < b.t; });
        for (size_t i = 0; i < intersections.size(); ++i) {
            const auto& isect = intersections[i];
            const Material& mat = isect.face->material;
            if (mat.tw < 0.01f) {
                return glm::vec3(0.0f, 0.0f, 0.0f);
            }
            if (i + 1 < intersections.size()) {
                const auto& next_isect = intersections[i + 1];
                FloatType distance_in_medium = next_isect.t - isect.t;
                glm::vec3 effective_sigma_a;
                if (glm::length(mat.sigma_a) > 0.0f) {
                    effective_sigma_a = mat.sigma_a;
                } else if (mat.td > 0.0f) {
                    effective_sigma_a = glm::vec3(-std::log(std::max(mat.base_color.r, 0.001f)) / mat.td, -std::log(std::max(mat.base_color.g, 0.001f)) / mat.td, -std::log(std::max(mat.base_color.b, 0.001f)) / mat.td);
                } else {
                    effective_sigma_a = glm::vec3(0.0f);
                }
                glm::vec3 absorption = glm::vec3(std::exp(-effective_sigma_a.r * distance_in_medium), std::exp(-effective_sigma_a.g * distance_in_medium), std::exp(-effective_sigma_a.b * distance_in_medium));
                trans = trans * absorption;
                ++i;
            }
            if (trans.r < 0.001f && trans.g < 0.001f && trans.b < 0.001f) {
                return glm::vec3(0.0f, 0.0f, 0.0f);
            }
        }
        return trans;
    }
};
class World {
private:
    std::vector<Model> models_;
    std::vector<Face> all_faces_;  // Cached flattened faces from all models
    std::vector<glm::vec3> all_vertices_;
    std::vector<glm::vec2> all_texcoords_;
    std::vector<glm::vec3> all_vertex_normals_;
    std::vector<glm::vec3> all_vertex_normals_by_vertex_;
    EnvironmentMap env_map_;  // HDR environment map
    std::vector<const Face*> emissive_faces_;
    BvhAccelerator accelerator_;
public:
    Camera camera_;
    
    World();
    void load_files(const std::vector<std::string>& filenames);
    void update() noexcept;  // Update world state (camera movement, etc.)
    void orbiting() noexcept;
    // Accessors for Renderer
    Camera& camera() noexcept { return camera_; }
    const Camera& camera() const noexcept { return camera_; }
    const std::vector<Model>& models() const noexcept { return models_; }
    const std::vector<Face>& all_faces() const noexcept { return all_faces_; }
    const std::vector<glm::vec3>& all_vertices() const noexcept { return all_vertices_; }
    const std::vector<glm::vec2>& all_texcoords() const noexcept { return all_texcoords_; }
    const std::vector<glm::vec3>& all_vertex_normals() const noexcept { return all_vertex_normals_; }
    const std::vector<glm::vec3>& all_vertex_normals_by_vertex() const noexcept { return all_vertex_normals_by_vertex_; }
    const EnvironmentMap& env_map() const noexcept { return env_map_; }
    const std::vector<const Face*>& area_lights() const noexcept { return emissive_faces_; }
    const BvhAccelerator& accelerator() const noexcept { return accelerator_; }
};
