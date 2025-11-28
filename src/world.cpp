#include <numeric>
#include <algorithm>
#include <functional>
#include <limits>
#include <cstdlib>
#include <random>
#include "world.hpp"
#include "window.hpp"
#include "scene_loader.hpp"

#include "../libs/stb_image.h"

// Parsing helpers moved to scene_loader.cpp

ColourHDR ColourHDR::from_srgb(const Colour& srgb, FloatType gamma) noexcept {
    auto to_linear = [gamma](std::uint8_t component) -> FloatType {
        FloatType normalized = component / 255.0f;
        if (gamma == 1.0f) return normalized;
        return std::pow(normalized, gamma);
    };
    return ColourHDR{
        .red = to_linear(srgb.red),
        .green = to_linear(srgb.green),
        .blue = to_linear(srgb.blue)
    };
}

EnvironmentMap::EnvironmentMap() noexcept : width_(0), height_(0), intensity_(1.0f) {}

EnvironmentMap::EnvironmentMap(std::size_t w, std::size_t h, std::vector<ColourHDR> data, FloatType intensity) noexcept
    : width_(w), height_(h), data_(std::move(data)), intensity_(intensity) {}

ColourHDR EnvironmentMap::sample(const glm::vec3& direction) const noexcept {
    if (!is_loaded()) return ColourHDR{ .red = 0.0f, .green = 0.0f, .blue = 0.0f };
    FloatType theta = std::atan2(direction.x, -direction.z);
    FloatType phi = std::asin(std::clamp(direction.y, -1.0f, 1.0f));
    FloatType u = (theta / (2.0f * std::numbers::pi)) + 0.5f;
    FloatType v = (phi / std::numbers::pi) + 0.5f;
    std::size_t x = static_cast<std::size_t>(std::clamp(u * static_cast<FloatType>(width_), 0.0f, static_cast<FloatType>(width_ - 1)));
    std::size_t y = static_cast<std::size_t>(std::clamp(v * static_cast<FloatType>(height_), 0.0f, static_cast<FloatType>(height_ - 1)));
    return data_[y * width_ + x] * intensity_;
}

// Camera implementation
glm::vec4 Camera::world_to_clip(const glm::vec3& vertex, double aspect_ratio) const noexcept {
    // Step: Transform world-space vertex into camera view space, then project to homogeneous clip
    glm::vec3 view_vector = vertex - position_;
    glm::mat3 view_rotation = glm::transpose(orientation());
    glm::vec3 view_space = view_rotation * view_vector;
    FloatType w = view_space.z;
    double fov_rad = glm::radians(FOV);
    double tan_half_fov = std::tan(fov_rad / 2.0);
    FloatType x_ndc = view_space.x / (view_space.z * tan_half_fov * aspect_ratio);
    FloatType y_ndc = view_space.y / (view_space.z * tan_half_fov);
    FloatType z_ndc = (FarPlane * (view_space.z - NearPlane)) / ((view_space.z) * (FarPlane - NearPlane));
    return glm::vec4(x_ndc * w, y_ndc * w, z_ndc * w, w);
}

glm::vec3 Camera::clip_to_ndc(const glm::vec4& clip) const noexcept {
    if (std::abs(clip.w) < 1e-6f) {
        return glm::vec3(0.0f, 0.0f, -1.0f);
    }
    // Step: Divide by w to get NDC; guards above avoid division by near-zero
    return glm::vec3(clip) / clip.w;
}

glm::mat3 Camera::orientation() const noexcept {
    // Step: Derive camera basis (right, up, forward) from yaw/pitch, then apply roll about forward
    FloatType cos_pitch = std::cos(pitch_);
    FloatType sin_pitch = std::sin(pitch_);
    FloatType cos_yaw = std::cos(yaw_);
    FloatType sin_yaw = std::sin(yaw_);
    glm::vec3 f(
        sin_yaw * cos_pitch,
        sin_pitch,
        -cos_yaw * cos_pitch
    );
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
    return glm::vec3(
        sin_yaw * cos_pitch,
        sin_pitch,
        -cos_yaw * cos_pitch
    );
}

glm::vec3 Camera::right() const noexcept {
    glm::mat3 o = orientation();
    return glm::normalize(o[0]);
}

glm::vec3 Camera::up() const noexcept {
    glm::mat3 o = orientation();
    return glm::normalize(o[1]);
}

void Camera::start_orbiting(glm::vec3 target) noexcept {
    orbit_target_ = target;
    orbit_radius_ = glm::length(position_ - orbit_target_);
    is_orbiting_ = true;
}

void Camera::orbiting() noexcept {
    if (is_orbiting_) {
        constexpr static FloatType angle_increment = glm::radians(-0.5f);
        yaw_ += angle_increment;
        position_ = orbit_target_ - forward() * orbit_radius_;
    }
}

void Camera::stop_orbiting() noexcept {
    is_orbiting_ = false;
}

void Camera::rotate(FloatType delta_yaw, FloatType delta_pitch) noexcept {
    yaw_ += delta_yaw;
    pitch_ += delta_pitch;
    
    constexpr FloatType max_pitch = glm::radians(89.0f);
    pitch_ = std::clamp(pitch_, -max_pitch, max_pitch);
}

void Camera::roll(FloatType delta_roll) noexcept {
    roll_ += delta_roll;
}

void Camera::move(FloatType forward_delta, FloatType right_delta, FloatType up_delta, FloatType dt) noexcept {
    position_ += forward() * (forward_delta * dt);
    position_ += right() * (right_delta * dt);
    position_ += up() * (up_delta * dt);
}

// Input handling removed from Camera; movement is driven by external input callbacks

// Input callbacks are registered centrally in main; World no longer registers to Window

std::pair<glm::vec3, glm::vec3> Camera::generate_ray(int pixel_x, int pixel_y, int screen_width, int screen_height, double aspect_ratio) const noexcept {
    FloatType u = (static_cast<FloatType>(pixel_x) + 0.5f) / static_cast<FloatType>(screen_width);
    FloatType v = (static_cast<FloatType>(pixel_y) + 0.5f) / static_cast<FloatType>(screen_height);
    return generate_ray_uv(u, v, screen_width, screen_height, aspect_ratio);
}

std::pair<glm::vec3, glm::vec3> Camera::generate_ray_uv(FloatType u, FloatType v, int screen_width, int screen_height, double aspect_ratio) const noexcept {
    // Step: Map pixel UV to view-space direction using pinhole camera model, then rotate to world space
    FloatType ndc_x = u * 2.0f - 1.0f;
    FloatType ndc_y = 1.0f - v * 2.0f;
    double fov_rad = glm::radians(FOV);
    double tan_half_fov = std::tan(fov_rad / 2.0);
    FloatType view_x = ndc_x * static_cast<FloatType>(tan_half_fov) * static_cast<FloatType>(aspect_ratio);
    FloatType view_y = ndc_y * static_cast<FloatType>(tan_half_fov);
    FloatType view_z = 1.0f;
    glm::vec3 ray_dir_view(view_x, view_y, view_z);
    glm::vec3 ray_dir_world = orientation() * ray_dir_view;
    return {position_, glm::normalize(ray_dir_world)};
}

void Camera::set_position(const glm::vec3& pos) noexcept { position_ = pos; }
void Camera::set_yaw(FloatType y) noexcept { yaw_ = y; }
void Camera::set_pitch(FloatType p) noexcept { pitch_ = p; }
void Camera::set_roll(FloatType r) noexcept { roll_ = r; }

// Model implementation
void Model::load_file(std::string filename) {
    SceneLoader::LoadObj(*this, filename);
}

void Model::load_scene_txt(std::string filename) {
    SceneLoader::LoadSceneTxt(*this, filename);
}

void Model::compute_face_normals() noexcept {
    // Step: Compute geometric normals per face from vertex positions; used for flat shading and backface tests
    for (auto& obj : objects_) {
        for (auto& f : obj.faces) {
            const glm::vec3& v0 = vertices_[f.v_indices[0]];
            const glm::vec3& v1 = vertices_[f.v_indices[1]];
            const glm::vec3& v2 = vertices_[f.v_indices[2]];
            f.face_normal = CalculateNormal(v0, v1, v2);
        }
    }
}

// Removed smooth_missing_vertex_normals as part of code cleanup

void Model::cache_faces() noexcept {
    // Step: Flatten per-object faces into a contiguous array for fast traversal and BVH build
    all_faces_.clear();
    all_faces_.reserve(std::accumulate(objects_.begin(), objects_.end(), std::size_t(0),
        [](std::size_t sum, const Object& obj) { return sum + obj.faces.size(); }));
    
    for (const auto& object : objects_) {
        all_faces_.insert(all_faces_.end(), object.faces.begin(), object.faces.end());
    }
}

void Model::load_materials(std::string filename) {
    // MTL loader: populate material table (albedo, shininess, metallic, IOR, transmission, absorption, emission, textures)
    auto current_material = materials_.end();
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open material file: " + filename);
    }
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;
        if (type == "newmtl") {
            std::string name;
            iss >> name;
            assert(materials_.find(name) == materials_.end());
            current_material = materials_.emplace(name, Material{}).first;
        } else if (type == "Kd") {
            assert(current_material != materials_.end());
            FloatType r, g, b;
            iss >> r >> g >> b;
            current_material->second.base_color = glm::vec3(r, g, b);
        } else if (type == "Ns") {
            assert(current_material != materials_.end());
            FloatType ns;
            iss >> ns;
            current_material->second.shininess = ns;
        } else if (type == "metallic") {
            assert(current_material != materials_.end());
            FloatType metallic_value;
            iss >> metallic_value;
            current_material->second.metallic = std::clamp(metallic_value, 0.0f, 1.0f);
        } else if (type == "ior") {
            assert(current_material != materials_.end());
            FloatType ior_value;
            iss >> ior_value;
            current_material->second.ior = std::max(1.0f, ior_value);
        } else if (type == "td") {
            assert(current_material != materials_.end());
            FloatType td_value;
            iss >> td_value;
            current_material->second.td = std::max(0.0f, td_value);
        } else if (type == "tw") {
            assert(current_material != materials_.end());
            FloatType tw_value;
            iss >> tw_value;
            current_material->second.tw = std::clamp(tw_value, 0.0f, 1.0f);
        } else if (type == "Ke") {
            assert(current_material != materials_.end());
            FloatType r, g, b;
            iss >> r >> g >> b;
            current_material->second.emission = glm::vec3(std::max(0.0f, r), std::max(0.0f, g), std::max(0.0f, b));
        } else if (type == "sigma_a") {
            assert(current_material != materials_.end());
            FloatType r, g, b;
            iss >> r >> g >> b;
            current_material->second.sigma_a = glm::vec3(
                std::max(0.0f, r),
                std::max(0.0f, g),
                std::max(0.0f, b)
            );
        } else if (type == "map_Kd") {
            assert(current_material != materials_.end());
            std::string texture_filename;
            iss >> texture_filename;
            texture_filename = (std::filesystem::path(filename).parent_path() / texture_filename).string();
            current_material->second.texture = std::make_shared<Texture>(load_texture(texture_filename));
        }
    }
}

Texture Model::load_texture(std::string filename) {
    // PPM (P6) loader: parse header, dimensions and raw RGB data into a Texture
    std::ifstream file(filename, std::ifstream::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open texture file: " + filename);
    }
    
    std::string magic_number;
    std::getline(file, magic_number);
    if (magic_number != "P6") {
        throw std::runtime_error("Invalid PPM format (expected P6): " + filename);
    }
    
    std::string line;
    std::getline(file, line);
    while (!line.empty() && line[0] == '#') {
        std::getline(file, line);
    }
    
    std::istringstream size_stream(line);
    std::size_t width, height;
    if (!(size_stream >> width >> height)) {
        throw std::runtime_error("Failed to parse texture dimensions: " + filename);
    }
    
    std::getline(file, line);
    
    std::vector<Colour> texture_data;
    texture_data.resize(width * height);
    for (std::size_t i = 0; i < width * height; i++) {
        int red = file.get();
        int green = file.get();
        int blue = file.get();
        if (red == EOF || green == EOF || blue == EOF) {
            throw std::runtime_error("Unexpected end of file while reading texture: " + filename);
        }
        texture_data[i] = Colour{
            .red = static_cast<std::uint8_t>(red),
            .green = static_cast<std::uint8_t>(green),
            .blue = static_cast<std::uint8_t>(blue)
        };
    }
    return Texture(width, height, std::move(texture_data));
}

// EnvironmentMap implementation
FloatType EnvironmentMap::ComputeAutoExposure(const std::vector<ColourHDR>& hdr_data) noexcept {
    // Step: Compute robust exposure using log-average luminance blended with 90th percentile
    if (hdr_data.empty()) return 1.0f;
    
    std::vector<FloatType> luminances;
    luminances.reserve(hdr_data.size());
    
    for (const auto& pixel : hdr_data) {
        FloatType luma = 0.2126f * pixel.red + 0.7152f * pixel.green + 0.0722f * pixel.blue;
        if (luma > 0.0f) {
            luminances.push_back(luma);
        }
    }
    
    if (luminances.empty()) return 1.0f;
    
    std::sort(luminances.begin(), luminances.end());
    
    std::size_t percentile_90_idx = static_cast<std::size_t>(luminances.size() * 0.90f);
    FloatType percentile_90 = luminances[percentile_90_idx];
    
    FloatType log_lum_sum = 0.0f;
    for (FloatType lum : luminances) {
        log_lum_sum += std::log(lum + 1e-6f);
    }
    FloatType avg_log_lum = std::exp(log_lum_sum / luminances.size());
    
    constexpr FloatType target_middle_gray = 0.3f;
    
    FloatType exposure_from_avg = target_middle_gray / (avg_log_lum + 1e-6f);
    FloatType exposure_from_p90 = target_middle_gray / (percentile_90 + 1e-6f);
    
    FloatType auto_exposure = 0.7f * exposure_from_avg + 0.3f * exposure_from_p90;
    auto_exposure = std::clamp(auto_exposure, 0.05f, 2.0f);
    
    return auto_exposure * 0.5f;
}

bool BVHAccelerator::empty() const noexcept { return nodes_.empty(); }

void BVHAccelerator::set_vertices(const std::vector<glm::vec3>& verts) noexcept { vertices_ = &verts; }
void BVHAccelerator::set_texcoords(const std::vector<glm::vec2>& uvs) noexcept { texcoords_ = &uvs; }
void BVHAccelerator::set_normals(const std::vector<glm::vec3>& norms) noexcept { normals_ = &norms; }
void BVHAccelerator::set_normals_by_vertex(const std::vector<glm::vec3>& norms) noexcept { normals_by_vertex_ = &norms; }

void BVHAccelerator::build(const std::vector<Face>& faces) noexcept {
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

bool BVHAccelerator::IntersectAABB(const glm::vec3& ro, const glm::vec3& rd, const AABB& box, FloatType tmax) noexcept {
    glm::vec3 inv = glm::vec3(1.0f) / rd;
    glm::vec3 t0 = (box.min - ro) * inv;
    glm::vec3 t1 = (box.max - ro) * inv;
    glm::vec3 tmin = glm::min(t0, t1);
    glm::vec3 tmaxv = glm::max(t0, t1);
    FloatType t_enter = std::max(std::max(tmin.x, tmin.y), tmin.z);
    FloatType t_exit = std::min(std::min(tmaxv.x, tmaxv.y), tmaxv.z);
    return t_enter <= t_exit && t_exit >= 0.0f && t_enter <= tmax;
}

RayTriangleIntersection BVHAccelerator::intersect(const glm::vec3& ro, const glm::vec3& rd, const std::vector<Face>& faces) const noexcept {
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
        if (!IntersectAABB(ro, rd, n.box, closest.distanceFromCamera)) continue;
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

glm::vec3 BVHAccelerator::transmittance(const glm::vec3& point, const glm::vec3& light_pos, const std::vector<Face>& faces) const noexcept {
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
            i++;
        }
        if (trans.r < 0.001f && trans.g < 0.001f && trans.b < 0.001f) {
            return glm::vec3(0.0f, 0.0f, 0.0f);
        }
    }
    return trans;
}

// World implementation
World::World() {}

void World::load_files(const std::vector<std::string>& filenames) {
    // Scene ingestion: load HDR environment, OBJ models or text scenes; then merge into a single geometry store
    for (const auto& filename : filenames) {
        std::string ext = std::filesystem::path(filename).extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        
        if (ext == ".hdr") {
            // Step 1: Load environment map and compute auto exposure
            int width, height, channels;
            float* data = stbi_loadf(filename.c_str(), &width, &height, &channels, 3);
            
            if (data) {
                std::vector<ColourHDR> hdr_data;
                hdr_data.reserve(width * height);
                
                for (int i = 0; i < width * height; ++i) {
                    hdr_data.emplace_back(
                        data[i * 3 + 0],
                        data[i * 3 + 1],
                        data[i * 3 + 2]
                    );
                }
                
                stbi_image_free(data);
                
                FloatType auto_intensity = EnvironmentMap::ComputeAutoExposure(hdr_data);
                env_map_ = EnvironmentMap(width, height, std::move(hdr_data), auto_intensity);
            } else {
                throw std::runtime_error("Failed to load HDR environment map: " + filename);
            }
        } else if (ext == ".txt") {
            // Step 2: Text scene — optionally references HDR via Environ; then load embedded geometry
            std::ifstream file(filename);
            if (file.is_open()) {
                std::string line;
                while (std::getline(file, line)) {
                    std::istringstream iss(line);
                    std::string type;
                    iss >> type;
                    if (type == "Environ") {
                        std::string hdr_name;
                        iss >> hdr_name;
                        std::string hdr_path = (std::filesystem::path(filename).parent_path() / hdr_name).string();
                        int width, height, channels;
                        float* data = stbi_loadf(hdr_path.c_str(), &width, &height, &channels, 3);
                        if (data) {
                            std::vector<ColourHDR> hdr_data;
                            hdr_data.reserve(width * height);
                            for (int i = 0; i < width * height; ++i) {
                                hdr_data.emplace_back(
                                    data[i * 3 + 0],
                                    data[i * 3 + 1],
                                    data[i * 3 + 2]
                                );
                            }
                            stbi_image_free(data);
                            FloatType auto_intensity = EnvironmentMap::ComputeAutoExposure(hdr_data);
                            env_map_ = EnvironmentMap(width, height, std::move(hdr_data), auto_intensity);
                        }
                        break;
                    }
                }
            }
            Model group;
            group.load_scene_txt(filename);
            models_.emplace_back(std::move(group));
        } else {
            // Step 3: Standard OBJ model
            Model group;
            group.load_file(filename);
            models_.emplace_back(std::move(group));
        }
    }
    
    // Step 4: Flatten all models into shared arrays (vertices, texcoords, normals, faces)
    std::size_t total_vertices = 0;
    std::size_t total_texcoords = 0;
    std::size_t total_normals = 0;
    std::size_t total_normals_by_vertex = 0;
    std::size_t total_faces = 0;
    for (const auto& model : models_) {
        total_vertices += model.vertices().size();
        total_texcoords += model.texture_coords().size();
        total_normals += model.vertex_normals().size();
        total_normals_by_vertex += model.vertex_normals_by_vertex().size();
        total_faces += model.all_faces().size();
    }
    all_vertices_.clear();
    all_vertices_.reserve(total_vertices);
    all_texcoords_.clear();
    all_texcoords_.reserve(total_texcoords);
    all_vertex_normals_.clear();
    all_vertex_normals_.reserve(total_normals);
    all_vertex_normals_by_vertex_.clear();
    all_vertex_normals_by_vertex_.reserve(total_normals_by_vertex);
    all_faces_.clear();
    all_faces_.reserve(total_faces);
    std::vector<std::size_t> vertex_offsets;
    std::vector<std::size_t> tex_offsets;
    std::vector<std::size_t> normal_offsets;
    std::vector<std::size_t> normal_by_vertex_offsets;
    vertex_offsets.reserve(models_.size());
    tex_offsets.reserve(models_.size());
    normal_offsets.reserve(models_.size());
    normal_by_vertex_offsets.reserve(models_.size());
    std::size_t running_offset = 0;
    std::size_t running_tex_offset = 0;
    std::size_t running_normal_offset = 0;
    std::size_t running_normal_by_vertex_offset = 0;
    for (const auto& model : models_) {
        vertex_offsets.push_back(running_offset);
        const auto& verts = model.vertices();
        all_vertices_.insert(all_vertices_.end(), verts.begin(), verts.end());
        running_offset += verts.size();
        tex_offsets.push_back(running_tex_offset);
        const auto& uvs = model.texture_coords();
        all_texcoords_.insert(all_texcoords_.end(), uvs.begin(), uvs.end());
        running_tex_offset += uvs.size();
        normal_offsets.push_back(running_normal_offset);
        const auto& norms = model.vertex_normals();
        all_vertex_normals_.insert(all_vertex_normals_.end(), norms.begin(), norms.end());
        running_normal_offset += norms.size();
        normal_by_vertex_offsets.push_back(running_normal_by_vertex_offset);
        const auto& norms_by_v = model.vertex_normals_by_vertex();
        all_vertex_normals_by_vertex_.insert(all_vertex_normals_by_vertex_.end(), norms_by_v.begin(), norms_by_v.end());
        running_normal_by_vertex_offset += norms_by_v.size();
    }
    for (std::size_t mi = 0; mi < models_.size(); ++mi) {
        const auto& model = models_[mi];
        std::size_t offset = vertex_offsets[mi];
        std::size_t toffset = tex_offsets[mi];
        std::size_t noffset = normal_offsets[mi];
        for (const auto& f : model.all_faces()) {
            Face f2 = f;
            for (int k = 0; k < 3; ++k) {
                f2.v_indices[k] = static_cast<std::uint32_t>(offset + f.v_indices[k]);
                if (f.vt_indices[k] != std::numeric_limits<std::uint32_t>::max()) {
                    f2.vt_indices[k] = static_cast<std::uint32_t>(toffset + f.vt_indices[k]);
                } else {
                    f2.vt_indices[k] = std::numeric_limits<std::uint32_t>::max();
                }
                if (f.vn_indices[k] != std::numeric_limits<std::uint32_t>::max()) {
                    f2.vn_indices[k] = static_cast<std::uint32_t>(noffset + f.vn_indices[k]);
                } else {
                    f2.vn_indices[k] = std::numeric_limits<std::uint32_t>::max();
                }
            }
            all_faces_.push_back(std::move(f2));
        }
    }

    emissive_faces_.clear();
    // Step 5: Identify emissive faces (area lights)
    for (const auto& f : all_faces_) {
        if (glm::length(f.material.emission) > 1e-6f) {
            emissive_faces_.push_back(&f);
        }
    }

    accelerator_.set_vertices(all_vertices_);
    accelerator_.set_texcoords(all_texcoords_);
    accelerator_.set_normals(all_vertex_normals_);
    accelerator_.set_normals_by_vertex(all_vertex_normals_by_vertex_);
    // Step 6: Build acceleration structure (BVH) over all faces
    accelerator_.build(all_faces_);
}

// World accessors
const std::vector<Model>& World::models() const noexcept { return models_; }
const std::vector<Face>& World::all_faces() const noexcept { return all_faces_; }
const std::vector<glm::vec3>& World::all_vertices() const noexcept { return all_vertices_; }
const std::vector<glm::vec2>& World::all_texcoords() const noexcept { return all_texcoords_; }
const std::vector<glm::vec3>& World::all_vertex_normals() const noexcept { return all_vertex_normals_; }
const std::vector<glm::vec3>& World::all_vertex_normals_by_vertex() const noexcept { return all_vertex_normals_by_vertex_; }
const EnvironmentMap& World::env_map() const noexcept { return env_map_; }
const std::vector<const Face*>& World::area_lights() const noexcept { return emissive_faces_; }
const BVHAccelerator& World::accelerator() const noexcept { return accelerator_; }
