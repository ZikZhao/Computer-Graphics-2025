#include "world.hpp"
#include "window.hpp"
#include <numeric>
#include <algorithm>
#include <functional>
#include <limits>
#include <cstdlib>
#include <random>

#define STB_IMAGE_IMPLEMENTATION
#include "../libs/stb_image.h"

// Camera implementation
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
    glm::vec3 f = forward();
    glm::vec3 world_up(0.0f, 1.0f, 0.0f);
    return glm::normalize(glm::cross(f, world_up));
}

glm::vec3 Camera::up() const noexcept {
    glm::vec3 r = right();
    glm::vec3 f = forward();
    return glm::normalize(glm::cross(r, f));
}

void Camera::start_orbiting(glm::vec3 target) noexcept {
    orbit_target_ = target;
    orbit_radius_ = glm::length(position_ - orbit_target_);
    is_orbiting_ = true;
}

void Camera::orbiting() noexcept {
    if (is_orbiting_) {
        constexpr static FloatType angle_increment = glm::radians(0.5f);
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

void Camera::move(FloatType forward_delta, FloatType right_delta, FloatType up_delta) noexcept {
    position_ += forward() * forward_delta;
    position_ += right() * right_delta;
    position_ += up() * up_delta;
}

void Camera::update_movement() noexcept {
    constexpr FloatType move_step = 0.1f;
    
    const Uint8* keystate = SDL_GetKeyboardState(nullptr);
    
    if (keystate[SDL_SCANCODE_W]) {
        position_ += forward() * move_step;
    }
    if (keystate[SDL_SCANCODE_S]) {
        position_ -= forward() * move_step;
    }
    if (keystate[SDL_SCANCODE_A]) {
        position_ -= right() * move_step;
    }
    if (keystate[SDL_SCANCODE_D]) {
        position_ += right() * move_step;
    }
    if (keystate[SDL_SCANCODE_SPACE]) {
        position_ += up() * move_step;
    }
    if (keystate[SDL_SCANCODE_LALT] || keystate[SDL_SCANCODE_RALT]) {
        position_ -= up() * move_step;
    }
}

void Camera::update() noexcept {
    update_movement();
}

// Input callbacks are registered centrally in main; World no longer registers to Window

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

std::pair<glm::vec3, glm::vec3> Camera::generate_ray(int pixel_x, int pixel_y, int screen_width, int screen_height, double aspect_ratio) const noexcept {
    FloatType ndc_x = (static_cast<FloatType>(pixel_x) + 0.5f) / static_cast<FloatType>(screen_width) * 2.0f - 1.0f;
    FloatType ndc_y = 1.0f - (static_cast<FloatType>(pixel_y) + 0.5f) / static_cast<FloatType>(screen_height) * 2.0f;
    
    double fov_rad = glm::radians(FOV);
    double tan_half_fov = std::tan(fov_rad / 2.0);
    
    FloatType view_x = ndc_x * tan_half_fov * aspect_ratio;
    FloatType view_y = ndc_y * tan_half_fov;
    FloatType view_z = 1.0f;
    
    glm::vec3 ray_dir_view(view_x, view_y, view_z);
    glm::vec3 ray_dir_world = orientation() * ray_dir_view;
    
    return {position_, glm::normalize(ray_dir_world)};
}

// Model implementation
void Model::load_file(std::string filename) {
    auto current_obj = objects_.end();
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;
        if (type == "mtllib") {
            std::string relative_path;
            iss >> relative_path;
            std::string material_filename = (std::filesystem::path(filename).parent_path() / relative_path).string();
            load_materials(std::move(material_filename));
        } else if (type == "l") {
            FloatType x, y, z;
            iss >> x >> y >> z;
            light_position_ = glm::vec3(x, y, z);
            has_light_ = true;
        } else if (type == "o") {
            std::string name;
            iss >> name;
            objects_.emplace_back(name);
            current_obj = std::prev(objects_.end());
        } else if (type == "usemtl") {
            assert(current_obj != objects_.end());
            std::string colour_name;
            iss >> colour_name;
            assert(materials_.find(colour_name) != materials_.end());
            auto prev_shading = current_obj->material.shading;
            current_obj->material = materials_[colour_name];
            current_obj->material.shading = prev_shading;
        } else if (type == "shading") {
            std::string mode;
            iss >> mode;
            if (current_obj != objects_.end()) {
                if (mode == "Gouraud") {
                    current_obj->material.shading = Material::Shading::Gouraud;
                } else if (mode == "Phong") {
                    current_obj->material.shading = Material::Shading::Phong;
                }
            }
        } else if (type == "v") {
            assert(current_obj != objects_.end());
            FloatType x, y, z;
            iss >> x >> y >> z;
            vertices_.emplace_back(x, y, z);
        } else if (type == "vt") {
            FloatType u, v;
            iss >> u >> v;
            texture_coords_.emplace_back(u, v);
        } else if (type == "vn") {
            FloatType x, y, z;
            iss >> x >> y >> z;
            vertex_normals_.emplace_back(glm::normalize(glm::vec3(x, y, z)));
        } else if (type == "f") {
            assert(current_obj != objects_.end());
            glm::vec3 vertice[3];
            std::uint8_t tex_indices[3];
            glm::vec2 tex_coords[3];
            int vi_idx[3];
            int normal_indices[3];
            bool has_normals = false;
            
            for (int i = 0; i < 3; i++) {
                int vertex_index;
                char slash;
                iss >> vertex_index >> slash;
                vertice[i] = vertices_[vertex_index - 1];
                vi_idx[i] = vertex_index - 1;
                tex_indices[i] = 0;
                tex_coords[i] = glm::vec2(0.0f, 0.0f);
                normal_indices[i] = -1;
                
                // Check for texture coordinate
                if (int c = iss.peek(); c >= '0' && c <= '9') {
                    int tex_idx;
                    iss >> tex_idx;
                    tex_indices[i] = tex_idx;
                    if (tex_idx > 0 && static_cast<size_t>(tex_idx) <= texture_coords_.size()) {
                        tex_coords[i] = texture_coords_[tex_idx - 1];
                    } else {
                        tex_coords[i] = glm::vec2(0.0f, 0.0f);
                    }
                }
                
                // Check for normal index (after second slash)
                if (iss.peek() == '/') {
                    iss >> slash;  // consume the second slash
                    if (int c = iss.peek(); c >= '0' && c <= '9') {
                        int normal_idx;
                        iss >> normal_idx;
                        normal_indices[i] = normal_idx - 1;
                        has_normals = true;
                    }
                }
            }
            
            Face new_face{
                { vertice[0], vertice[1], vertice[2] },
                { tex_indices[0], tex_indices[1], tex_indices[2] },
                { tex_coords[0], tex_coords[1], tex_coords[2] },
                current_obj->material,
                { vi_idx[0], vi_idx[1], vi_idx[2] },
                { glm::vec3(0.0f), glm::vec3(0.0f), glm::vec3(0.0f) },
                glm::vec3(0.0f)
            };
            
            // If normals were provided in the file, use them
            if (has_normals) {
                for (int i = 0; i < 3; i++) {
                    if (normal_indices[i] >= 0 && static_cast<size_t>(normal_indices[i]) < vertex_normals_.size()) {
                        new_face.vertex_normals[i] = vertex_normals_[normal_indices[i]];
                    }
                }
            }
            
            current_obj->faces.emplace_back(std::move(new_face));
        }
    }
    
    // Compute face normals
    for (auto& obj : objects_) {
        for (auto& f : obj.faces) {
            f.face_normal = CalculateNormal(f.vertices[0], f.vertices[1], f.vertices[2]);
        }
    }
    
    // Compute vertex normals only for faces that don't have them from the file
    std::vector<glm::vec3> accum_normals(vertices_.size(), glm::vec3(0.0f));
    for (auto& obj : objects_) {
        for (auto& f : obj.faces) {
            // Check if this face needs computed normals
            bool needs_computed = false;
            for (int k = 0; k < 3; ++k) {
                if (glm::length(f.vertex_normals[k]) < 0.001f) {
                    needs_computed = true;
                    break;
                }
            }
            
            if (needs_computed) {
                glm::vec3 n = f.face_normal;
                glm::vec3 e0 = f.vertices[1] - f.vertices[0];
                glm::vec3 e1 = f.vertices[2] - f.vertices[0];
                FloatType area2 = glm::length(glm::cross(e0, e1));
                glm::vec3 weighted = n * area2;
                for (int k = 0; k < 3; ++k) {
                    int idx = f.vertex_indices[k];
                    if (idx >= 0 && static_cast<std::size_t>(idx) < accum_normals.size()) {
                        accum_normals[idx] += weighted;
                    }
                }
            }
        }
    }
    
    // Normalize accumulated normals
    for (auto& acc : accum_normals) {
        if (glm::length(acc) > 0.0f) acc = glm::normalize(acc);
    }
    
    // Apply computed normals to faces that need them
    for (auto& obj : objects_) {
        for (auto& f : obj.faces) {
            for (int k = 0; k < 3; ++k) {
                // Only set if not already set from file
                if (glm::length(f.vertex_normals[k]) < 0.001f) {
                    int idx = f.vertex_indices[k];
                    f.vertex_normals[k] = (idx >= 0 && static_cast<std::size_t>(idx) < accum_normals.size()) 
                        ? accum_normals[idx] 
                        : CalculateNormal(f.vertices[0], f.vertices[1], f.vertices[2]);
                }
            }
        }
    }
    cache_faces();
}

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
    
    if (!found_light) {
        light_pos = glm::vec3(0.0f, 2.0f, 0.0f);
    }
    
    const_cast<glm::vec3&>(light_position_) = light_pos;
    const_cast<bool&>(has_light_) = found_light;
    
    std::size_t total_faces = 0;
    for (const auto& model : models_) {
        total_faces += model.all_faces().size();
    }
    all_faces_.reserve(total_faces);
    for (const auto& model : models_) {
        all_faces_.insert(all_faces_.end(), model.all_faces().begin(), model.all_faces().end());
    }
}

// Legacy SDL event hooks removed; input is handled via keystate callbacks

void World::update() noexcept {
}

void World::orbiting() noexcept {
    camera_.orbiting();
}
