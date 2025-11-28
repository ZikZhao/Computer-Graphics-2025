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

// parsing helpers moved to scene_loader.cpp

// Camera implementation
glm::vec4 Camera::world_to_clip(const glm::vec3& vertex, double aspect_ratio) const noexcept {
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
    return glm::vec3(clip) / clip.w;
}

glm::mat3 Camera::orientation() const noexcept {
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

// Model implementation
void Model::load_file(std::string filename) {
    SceneLoader::LoadObj(*this, filename);
}

void Model::load_scene_txt(std::string filename) {
    SceneLoader::LoadSceneTxt(*this, filename);
}

void Model::compute_face_normals() noexcept {
    for (auto& obj : objects_) {
        for (auto& f : obj.faces) {
            const glm::vec3& v0 = vertices_[f.vertex_indices[0]];
            const glm::vec3& v1 = vertices_[f.vertex_indices[1]];
            const glm::vec3& v2 = vertices_[f.vertex_indices[2]];
            f.face_normal = CalculateNormal(v0, v1, v2);
        }
    }
}

// Removed smooth_missing_vertex_normals as part of code cleanup

void Model::cache_faces() noexcept {
    all_faces_.clear();
    all_faces_.reserve(std::accumulate(objects_.begin(), objects_.end(), std::size_t(0),
        [](std::size_t sum, const Object& obj) { return sum + obj.faces.size(); }));
    
    for (const auto& object : objects_) {
        all_faces_.insert(all_faces_.end(), object.faces.begin(), object.faces.end());
    }
}

void Model::load_materials(std::string filename) {
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
            static_cast<std::uint8_t>(red),
            static_cast<std::uint8_t>(green),
            static_cast<std::uint8_t>(blue)
        };
    }
    return Texture(width, height, std::move(texture_data));
}

// EnvironmentMap implementation
FloatType EnvironmentMap::compute_auto_exposure(const std::vector<ColourHDR>& hdr_data) noexcept {
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

// World implementation
World::World() : light_position_(0.0f, 0.0f, 0.0f), has_light_(false) {}

void World::load_files(const std::vector<std::string>& filenames) {
    for (const auto& filename : filenames) {
        std::string ext = std::filesystem::path(filename).extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        
        if (ext == ".hdr") {
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
                
                FloatType auto_intensity = EnvironmentMap::compute_auto_exposure(hdr_data);
                env_map_ = EnvironmentMap(width, height, std::move(hdr_data), auto_intensity);
            } else {
                throw std::runtime_error("Failed to load HDR environment map: " + filename);
            }
        } else if (ext == ".txt") {
            {
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
                                FloatType auto_intensity = EnvironmentMap::compute_auto_exposure(hdr_data);
                                env_map_ = EnvironmentMap(width, height, std::move(hdr_data), auto_intensity);
                            }
                            break;
                        }
                    }
                }
            }
            Model group;
            group.load_scene_txt(filename);
            models_.emplace_back(std::move(group));
        } else {
            Model group;
            group.load_file(filename);
            models_.emplace_back(std::move(group));
        }
    }
    
    glm::vec3 light_pos(0.0f, 0.0f, 0.0f);
    bool found_light = false;
    for (const auto& model : models_) {
        if (model.has_light()) {
            light_pos = model.light_position();
            found_light = true;
            break;
        }
    }
    const_cast<glm::vec3&>(light_position_) = light_pos;
    const_cast<bool&>(has_light_) = found_light;
    
    std::size_t total_vertices = 0;
    std::size_t total_faces = 0;
    for (const auto& model : models_) {
        total_vertices += model.vertices().size();
        total_faces += model.all_faces().size();
    }
    all_vertices_.clear();
    all_vertices_.reserve(total_vertices);
    all_faces_.clear();
    all_faces_.reserve(total_faces);
    std::vector<std::size_t> vertex_offsets;
    vertex_offsets.reserve(models_.size());
    std::size_t running_offset = 0;
    for (const auto& model : models_) {
        vertex_offsets.push_back(running_offset);
        const auto& verts = model.vertices();
        all_vertices_.insert(all_vertices_.end(), verts.begin(), verts.end());
        running_offset += verts.size();
    }
    for (std::size_t mi = 0; mi < models_.size(); ++mi) {
        const auto& model = models_[mi];
        std::size_t offset = vertex_offsets[mi];
        for (const auto& f : model.all_faces()) {
            Face f2 = f;
            for (int k = 0; k < 3; ++k) {
                f2.vertex_indices[k] = static_cast<std::uint32_t>(offset + f.vertex_indices[k]);
            }
            all_faces_.push_back(std::move(f2));
        }
    }

    emissive_faces_.clear();
    for (const auto& f : all_faces_) {
        if (glm::length(f.material.emission) > 1e-6f) {
            emissive_faces_.push_back(&f);
        }
    }

    accelerator_.set_vertices(all_vertices_);
    accelerator_.build(all_faces_);
}

// Legacy SDL event hooks removed; input is handled via keystate callbacks

void World::update() noexcept {
}

void World::orbiting() noexcept {
    camera_.orbiting();
}
