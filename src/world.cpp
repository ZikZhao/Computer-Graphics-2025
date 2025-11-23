#include "world.hpp"
#include <numeric>
#include <algorithm>
#include <functional>
#include <limits>

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
    
    // Clamp pitch to prevent gimbal lock (±89 degrees)
    constexpr FloatType max_pitch = glm::radians(89.0f);
    pitch_ = Clamp(pitch_, -max_pitch, max_pitch);
    
    
}

void Camera::handle_event(const SDL_Event& event) noexcept {
    if (event.type == SDL_KEYDOWN) {
        constexpr FloatType move_step = 0.1f;
        switch (event.key.keysym.sym) {
        case SDLK_w:
            position_ += up() * move_step;
            break;
        case SDLK_s:
            position_ -= up() * move_step;
            break;
        case SDLK_a:
            position_ -= right() * move_step;
            break;
        case SDLK_d:
            position_ += right() * move_step;
            break;
        case SDLK_q:
            position_ += forward() * move_step;
            break;
        case SDLK_e:
            position_ -= forward() * move_step;
            break;
        case SDLK_UP:
            rotate(0.0f, -glm::radians(2.0f));
            return;
        case SDLK_DOWN:
            rotate(0.0f, glm::radians(2.0f));
            return;
        case SDLK_LEFT:
            rotate(-glm::radians(2.0f), 0.0f);
            return;
        case SDLK_RIGHT:
            rotate(glm::radians(2.0f), 0.0f);
            return;
        case SDLK_o:
            if (!is_orbiting_) {
                start_orbiting(orbit_target_);
            } else {
                stop_orbiting();
            }
            return;
        }
    } else if (event.type == SDL_MOUSEBUTTONDOWN) {
        if (event.button.button == SDL_BUTTON_LEFT) {
            // If we're in orbiting mode, stop it before drag
            if (is_orbiting_) {
                stop_orbiting();
            }
            
            // Start dragging and mark that we should skip the first motion event
            is_dragging_ = true;
            first_drag_motion_ = true;
        }
    } else if (event.type == SDL_MOUSEBUTTONUP) {
        if (event.button.button == SDL_BUTTON_LEFT) {
            // Stop dragging
            is_dragging_ = false;
            first_drag_motion_ = false;
        }
    } else if (event.type == SDL_MOUSEMOTION) {
        // Only rotate when dragging
        if (is_dragging_) {
            // Skip the first motion event to avoid initial jump from accumulated SDL motion
            if (first_drag_motion_) {
                first_drag_motion_ = false;
            } else {
                // Use SDL's relative motion (xrel, yrel) - these are always relative deltas
                FloatType delta_yaw = -static_cast<FloatType>(event.motion.xrel) * mouse_sensitivity_;
                FloatType delta_pitch = static_cast<FloatType>(event.motion.yrel) * mouse_sensitivity_;
                rotate(delta_yaw, delta_pitch);
            }
        }
    }
}
glm::vec4 Camera::world_to_clip(const glm::vec3& vertex, double aspect_ratio) const noexcept {
    // View transformation
    glm::vec3 view_vector = vertex - position_;
    glm::mat3 view_rotation = glm::transpose(orientation());
    glm::vec3 view_space = view_rotation * view_vector;
    
    // In our coordinate system, positive view_space.z means in front of camera
    FloatType w = view_space.z;  // Distance from camera
    
    // Perspective projection (similar to original world_to_ndc but keep w separate)
    double fov_rad = glm::radians(FOV);
    double tan_half_fov = std::tan(fov_rad / 2.0);
    
    // Compute NDC x, y (but multiply by w to keep in clip space)
    FloatType x_ndc = view_space.x / (view_space.z * tan_half_fov * aspect_ratio);
    FloatType y_ndc = view_space.y / (view_space.z * tan_half_fov);
    FloatType z_ndc = (FarPlane * (view_space.z - NearPlane)) / ((view_space.z) * (FarPlane - NearPlane));
    
    // Return clip space: multiply NDC by w
    return glm::vec4(
        x_ndc * w,
        y_ndc * w,
        z_ndc * w,
        w
    );
}
glm::vec3 Camera::clip_to_ndc(const glm::vec4& clip) const noexcept {
    // Perspective division
    if (std::abs(clip.w) < 1e-6f) {
        return glm::vec3(0.0f, 0.0f, -1.0f);
    }
    return glm::vec3(clip) / clip.w;
}
std::pair<glm::vec3, glm::vec3> Camera::generate_ray(int pixel_x, int pixel_y, int screen_width, int screen_height, double aspect_ratio) const noexcept {
    // Convert pixel coordinates to NDC space [-1, 1]
    FloatType ndc_x = (static_cast<FloatType>(pixel_x) + 0.5f) / static_cast<FloatType>(screen_width) * 2.0f - 1.0f;
    FloatType ndc_y = 1.0f - (static_cast<FloatType>(pixel_y) + 0.5f) / static_cast<FloatType>(screen_height) * 2.0f;
    
    // Convert NDC to view space using inverse projection
    double fov_rad = glm::radians(FOV);
    double tan_half_fov = std::tan(fov_rad / 2.0);
    
    FloatType view_x = ndc_x * tan_half_fov * aspect_ratio;
    FloatType view_y = ndc_y * tan_half_fov;
    FloatType view_z = 1.0f;  // Forward direction (camera looks in +z in view space)
    
    // Transform from view space to world space
    glm::vec3 ray_dir_view(view_x, view_y, view_z);
    glm::vec3 ray_dir_world = orientation() * ray_dir_view;
    
    return {position_, glm::normalize(ray_dir_world)};
}

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
            // Light position definition
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
        } else if (type == "s") {
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
        } else if (type == "f") {
            assert(current_obj != objects_.end());
            glm::vec3 vertice[3];
            std::uint8_t tex_indices[3];
            glm::vec2 tex_coords[3];
            int vi_idx[3];
            for (int i = 0; i < 3; i++) {
                int vertex_index;
                char slash;
                iss >> vertex_index >> slash;
                vertice[i] = vertices_[vertex_index - 1];
                vi_idx[i] = vertex_index - 1;
                tex_indices[i] = 0;
                tex_coords[i] = glm::vec2(0.0f, 0.0f);
                
                if (int c = iss.peek(); c >= '0' && c <= '9') {
                    int tex_idx;
                    iss >> tex_idx;
                    tex_indices[i] = tex_idx;
                    // Store actual UV coordinates if we have them
                    if (tex_idx > 0 && static_cast<size_t>(tex_idx) <= texture_coords_.size()) {
                        tex_coords[i] = texture_coords_[tex_idx - 1];
                    } else {
                        tex_coords[i] = glm::vec2(0.0f, 0.0f);
                    }
                } else {
                }
            }
            Face new_face{
                { vertice[0], vertice[1], vertice[2] },
                { tex_indices[0], tex_indices[1], tex_indices[2] },
                { tex_coords[0], tex_coords[1], tex_coords[2] },
                current_obj->material,
                { vi_idx[0], vi_idx[1], vi_idx[2] },
                { glm::vec3(0.0f), glm::vec3(0.0f), glm::vec3(0.0f) }
            };
            current_obj->faces.emplace_back(std::move(new_face));
        }
    }
    std::vector<glm::vec3> accum_normals(vertices_.size(), glm::vec3(0.0f));
    for (auto& obj : objects_) {
        for (auto& f : obj.faces) {
            glm::vec3 n = CalculateNormal(f.vertices[0], f.vertices[1], f.vertices[2]);
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
    for (auto& acc : accum_normals) {
        if (glm::length(acc) > 0.0f) acc = glm::normalize(acc);
    }
    for (auto& obj : objects_) {
        for (auto& f : obj.faces) {
            for (int k = 0; k < 3; ++k) {
                int idx = f.vertex_indices[k];
                f.vertex_normals[k] = (idx >= 0 && static_cast<std::size_t>(idx) < accum_normals.size()) ? accum_normals[idx] : CalculateNormal(f.vertices[0], f.vertices[1], f.vertices[2]);
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
            current_material = materials_.emplace(name, Colour{255, 255, 255}).first;
        } else if (type == "Kd") {
            assert(current_material != materials_.end());
            FloatType r, g, b;
            iss >> r >> g >> b;
            current_material->second.colour = Colour{
                static_cast<std::uint8_t>(Clamp(r * 255.0f, 0.0f, 255.0f)),
                static_cast<std::uint8_t>(Clamp(g * 255.0f, 0.0f, 255.0f)),
                static_cast<std::uint8_t>(Clamp(b * 255.0f, 0.0f, 255.0f))
            };
        } else if (type == "Ns") {
            // Shininess/specular exponent (0-1000 range in MTL, typically 0-200)
            assert(current_material != materials_.end());
            FloatType ns;
            iss >> ns;
            current_material->second.shininess = ns;
        } else if (type == "m") {
            // Mirror material flag
            assert(current_material != materials_.end());
            std::string flag;
            iss >> flag;
            current_material->second.is_mirror = (flag == "True" || flag == "true" || flag == "1");
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

World::World() : light_position_(0.0f, 0.0f, 0.0f), has_light_(false) {}

void World::load_files(const std::vector<std::string>& filenames) {
    for (const auto& filename : filenames) {
        Model group;
        group.load_file(filename);
        models_.emplace_back(std::move(group));
    }
    
    // Extract light position from first model that has a light definition
    glm::vec3 light_pos(0.0f, 0.0f, 0.0f);
    bool found_light = false;
    for (const auto& model : models_) {
        if (model.has_light()) {
            light_pos = model.light_position();
            found_light = true;
            break;
        }
    }
    
    // Fallback: if no light found, use default position
    if (!found_light) {
        light_pos = glm::vec3(0.0f, 2.0f, 0.0f);
    }
    
    // Initialize const members via const_cast (safe since this is during construction phase)
    const_cast<glm::vec3&>(light_position_) = light_pos;
    const_cast<bool&>(has_light_) = found_light;
    
    // Cache all faces from all models (computed once)
    std::size_t total_faces = 0;
    for (const auto& model : models_) {
        total_faces += model.all_faces().size();
    }
    all_faces_.reserve(total_faces);
    for (const auto& model : models_) {
        all_faces_.insert(all_faces_.end(), model.all_faces().begin(), model.all_faces().end());
    }
}
void World::handle_event(const SDL_Event& event) noexcept {
    camera_.handle_event(event);
    
    // Handle light intensity adjustment
    if (event.type == SDL_KEYDOWN) {
        switch (event.key.keysym.sym) {
        case SDLK_EQUALS:  // '+' key (without shift)
        case SDLK_PLUS:    // '+' key (with shift)
            light_intensity_ += 1.0f;
            break;
        case SDLK_MINUS:
            light_intensity_ = std::max(0.1f, light_intensity_ - 1.0f);
            break;
        }
    }
}
void World::orbiting() noexcept {
    camera_.orbiting();
}

Renderer::Renderer(DrawingWindow& window, const World& world) noexcept
    : window_(window),
      world_(world),
      z_buffer_(window.width * window.height, 0.0f),
      hdr_buffer_(window.width * window.height, ColourHDR()),
      bvh_tri_indices_([]() { return std::vector<int>(); }()),
      bvh_nodes_([]() { return std::vector<Renderer::BVHNode>(); }()),
      frame_barrier_(std::thread::hardware_concurrency() + 1) {
    // Build BVH once and store both parts
    auto [indices, nodes] = build_bvh(world.all_faces());
    const_cast<std::vector<int>&>(bvh_tri_indices_) = std::move(indices);
    const_cast<std::vector<Renderer::BVHNode>&>(bvh_nodes_) = std::move(nodes);
    
    // Start worker threads
    for (unsigned int i = 0; i < std::thread::hardware_concurrency(); ++i) {
        workers_.emplace_back([this](std::stop_token) { this->worker_thread(); });
    }
}
void Renderer::clear() noexcept {
    z_buffer_.assign(window_.width * window_.height, 0.0f);
    hdr_buffer_.assign(window_.width * window_.height, ColourHDR());
}
ScreenNdcCoord Renderer::ndc_to_screen(const glm::vec3& ndc, const glm::vec2& uv, FloatType w) const noexcept {
    return ScreenNdcCoord{
        (ndc.x + 1.0f) * 0.5f * window_.width,
        (1.0f - ndc.y) * 0.5f * window_.height,
        ndc.z,
        uv,
        1.0f / w  // Store 1/w for perspective correction
    };
}
bool Renderer::inside_plane(const glm::vec4& v, ClipPlane plane) noexcept {
    switch (plane) {
        case ClipPlane::Left:   return v.x >= -v.w;
        case ClipPlane::Right:  return v.x <= v.w;
        case ClipPlane::Bottom: return v.y >= -v.w;
        case ClipPlane::Top:    return v.y <= v.w;
        case ClipPlane::Near:   return v.z >= 0.0f;
        case ClipPlane::Far:    return v.z <= v.w;
    }
    return false;
}
FloatType Renderer::compute_intersection_t(const glm::vec4& v0, const glm::vec4& v1, ClipPlane plane) noexcept {
    FloatType d0, d1;
    switch (plane) {
        case ClipPlane::Left:
            d0 = v0.x + v0.w;
            d1 = v1.x + v1.w;
            break;
        case ClipPlane::Right:
            d0 = v0.w - v0.x;
            d1 = v1.w - v1.x;
            break;
        case ClipPlane::Bottom:
            d0 = v0.y + v0.w;
            d1 = v1.y + v1.w;
            break;
        case ClipPlane::Top:
            d0 = v0.w - v0.y;
            d1 = v1.w - v1.y;
            break;
        case ClipPlane::Near:
            d0 = v0.z;
            d1 = v1.z;
            break;
        case ClipPlane::Far:
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
ClipVertex Renderer::intersect_plane(const ClipVertex& v0, const ClipVertex& v1, ClipPlane plane) noexcept {
    FloatType t = compute_intersection_t(v0.position_clip, v1.position_clip, plane);
    // Linear interpolation of vertex attributes at intersection point
    return ClipVertex{
        v0.position_clip * (1.0f - t) + v1.position_clip * t,
        Colour{
            static_cast<std::uint8_t>(v0.colour.red * (1.0f - t) + v1.colour.red * t),
            static_cast<std::uint8_t>(v0.colour.green * (1.0f - t) + v1.colour.green * t),
            static_cast<std::uint8_t>(v0.colour.blue * (1.0f - t) + v1.colour.blue * t)
        },
        v0.uv * (1.0f - t) + v1.uv * t  // Interpolate UV coordinates
    };
}
InplaceVector<ClipVertex, 9> Renderer::clip_against_plane(const InplaceVector<ClipVertex, 9>& input, ClipPlane plane) noexcept {

    InplaceVector<ClipVertex, 9> output;
    
    if (input.size() == 0) return output;
    
    for (size_t i = 0; i < input.size(); i++) {
        const ClipVertex& current = input[i];
        const ClipVertex& next = input[(i + 1) % input.size()];
        
        bool current_inside = inside_plane(current.position_clip, plane);
        bool next_inside = inside_plane(next.position_clip, plane);
        
        if (current_inside && next_inside) {
            // Edge completely inside -> keep next vertex
            output.push_back(next);
        } 
        else if (current_inside && !next_inside) {
            // Exiting -> add intersection point
            output.push_back(intersect_plane(current, next, plane));
        } 
        else if (!current_inside && next_inside) {
            // Entering -> add intersection and next vertex
            output.push_back(intersect_plane(current, next, plane));
            output.push_back(next);
        }
        // Both outside -> skip
    }
    
    return output;
}
InplaceVector<ClipVertex, 9> Renderer::clip_triangle(const Camera& camera, const Face& face) noexcept {
    // Transform to clip space
    InplaceVector<ClipVertex, 9> polygon = {
        ClipVertex{camera.world_to_clip(face.vertices[0], aspect_ratio_), face.material.colour, face.texture_coords[0]},
        ClipVertex{camera.world_to_clip(face.vertices[1], aspect_ratio_), face.material.colour, face.texture_coords[1]},
        ClipVertex{camera.world_to_clip(face.vertices[2], aspect_ratio_), face.material.colour, face.texture_coords[2]}
    };
    
    // Clip against all 6 frustum planes
    polygon = clip_against_plane(polygon, ClipPlane::Left);
    if (polygon.size() < 3) return {};
    
    polygon = clip_against_plane(polygon, ClipPlane::Right);
    if (polygon.size() < 3) return {};
    
    polygon = clip_against_plane(polygon, ClipPlane::Bottom);
    if (polygon.size() < 3) return {};
    
    polygon = clip_against_plane(polygon, ClipPlane::Top);
    if (polygon.size() < 3) return {};
    
    polygon = clip_against_plane(polygon, ClipPlane::Near);
    if (polygon.size() < 3) return {};
    
    polygon = clip_against_plane(polygon, ClipPlane::Far);

    return polygon;
}
void Renderer::handle_event(const SDL_Event& event) noexcept {
    if (event.type == SDL_KEYDOWN) {
        switch (event.key.keysym.sym) {
        case SDLK_1:
            mode_ = Wireframe;
            break;
        case SDLK_2:
            mode_ = Rasterized;
            break;
        case SDLK_3:
            mode_ = Raytraced;
            break;
        case SDLK_g:
            gamma_ = (gamma_ == 2.2f) ? 1.0f : 2.2f;
            break;
        }
    }
}
std::uint32_t Renderer::sample_texture(const Face& face, const glm::vec3& bary,
                                       const ScreenNdcCoord& v0, const ScreenNdcCoord& v1, const ScreenNdcCoord& v2) noexcept {
    if (!face.material.texture) return face.material.colour;
    
    // Perspective-correct interpolation: interpolate uv/w and 1/w, then divide
    FloatType inv_w = bary.z * v0.inv_w + bary.x * v1.inv_w + bary.y * v2.inv_w;
    FloatType u = (bary.z * v0.uv.x * v0.inv_w + bary.x * v1.uv.x * v1.inv_w + bary.y * v2.uv.x * v2.inv_w) / inv_w;
    FloatType v = (bary.z * v0.uv.y * v0.inv_w + bary.x * v1.uv.y * v1.inv_w + bary.y * v2.uv.y * v2.inv_w) / inv_w;
    
    return face.material.texture->sample(u, v);
}
void Renderer::render() noexcept {
    clear();
    
    aspect_ratio_ = static_cast<double>(window_.width) / window_.height;
    
    // Dispatch based on rendering mode
    switch (mode_) {
    case Wireframe:
        wireframe_render();
        break;
    case Rasterized:
        rasterized_render();
        break;
    case Raytraced:
        raytraced_render();
        break;
    }
}
void Renderer::wireframe_render() noexcept {
    for (const auto& model : world_.models()) {
        for (const auto& face : model.all_faces()) {
            wireframe_render(world_.camera(), face);
        }
    }
}
void Renderer::rasterized_render() noexcept {
    for (const auto& model : world_.models()) {
        for (const auto& face : model.all_faces()) {
            rasterized_render(world_.camera(), face);
        }
    }
}
void Renderer::raytraced_render() noexcept {
    // Set up frame context for worker threads (only camera changes per frame)
    current_camera_ = &world_.camera();
    tile_counter_.store(0, std::memory_order_relaxed);
    
    // Signal workers to start processing and wait for completion
    frame_barrier_.arrive_and_wait();  // Start signal
    frame_barrier_.arrive_and_wait();  // Completion wait
}
void Renderer::wireframe_render(const Camera& camera, const Face& face) noexcept {
    auto clipped = clip_triangle(camera, face);
    if (clipped.size() < 3) return;
    
    // Convert to screen space
    InplaceVector<ScreenNdcCoord, 9> screen_verts;
    for (size_t i = 0; i < clipped.size(); i++) {
        glm::vec3 ndc = camera.clip_to_ndc(clipped[i].position_clip);
        screen_verts.push_back(ndc_to_screen(ndc, clipped[i].uv, clipped[i].position_clip.w));
    }
    
    std::uint32_t colour = clipped[0].colour;
    
    // Draw edges of the polygon
    for (size_t i = 0; i < screen_verts.size(); i++) {
        ScreenNdcCoord from = screen_verts[i];
        ScreenNdcCoord to = screen_verts[(i + 1) % screen_verts.size()];
        
        if (std::abs(to.x - from.x) >= std::abs(to.y - from.y)) {
            std::size_t from_x = std::max<std::size_t>(static_cast<std::size_t>(std::min(from.x, to.x)), 0);
            std::size_t to_x = std::min(static_cast<std::size_t>(std::max(from.x, to.x)), window_.width - 1);
            for (std::size_t x = from_x; x <= to_x; x++) {
                std::int64_t y = static_cast<std::int64_t>(std::round(from.y + (to.y - from.y) * (x - from.x) / (to.x - from.x)));
                if (x >= window_.width || y < 0 || y >= static_cast<std::int64_t>(window_.height)) {
                    continue;
                }
                FloatType progress = (to.x == from.x) ? 0.0f : static_cast<FloatType>(x - from.x) / static_cast<FloatType>(to.x - from.x);
                FloatType inv_z = ComputeInvZndc(progress, std::array<FloatType, 2>{from.z_ndc, to.z_ndc});
                FloatType& depth = z_buffer_[y * window_.width + x];
                if (inv_z > depth) {
                    depth = inv_z;
                    window_.setPixelColour(x, y, colour);
                }
            }
        } else {
            std::size_t from_y = std::max<std::size_t>(static_cast<std::size_t>(std::min(from.y, to.y)), 0);
            std::size_t to_y = std::min(static_cast<std::size_t>(std::max(from.y, to.y)), static_cast<std::size_t>(window_.height - 1));
            for (std::size_t y = from_y; y <= to_y; y++) {
                std::int64_t x = static_cast<std::int64_t>(std::round(from.x + (to.x - from.x) * (y - from.y) / (to.y - from.y)));
                if (x < 0 || x >= static_cast<std::int64_t>(window_.width) || y >= window_.height) {
                    continue;
                }
                FloatType progress = (to.y == from.y) ? 0.0f : static_cast<FloatType>(y - from.y) / static_cast<FloatType>(to.y - from.y);
                FloatType inv_z = ComputeInvZndc(progress, std::array<FloatType, 2>{from.z_ndc, to.z_ndc});
                FloatType& depth = z_buffer_[y * window_.width + x];
                if (inv_z > depth) {
                    depth = inv_z;
                    window_.setPixelColour(x, y, colour);
                }
            }
        }
    }
}
void Renderer::rasterized_render(const Camera& camera, const Face& face) noexcept {
    auto clipped = clip_triangle(camera, face);
    if (clipped.size() < 3) return;
    
    // Convert all vertices to screen space
    InplaceVector<ScreenNdcCoord, 9> screen_verts;
    for (size_t i = 0; i < clipped.size(); i++) {
        glm::vec3 ndc = camera.clip_to_ndc(clipped[i].position_clip);
        screen_verts.push_back(ndc_to_screen(ndc, clipped[i].uv, clipped[i].position_clip.w));
    }
    
    // Triangle fan rasterization: (0, i, i+1) for i = 1..n-2
    for (size_t i = 1; i + 1 < clipped.size(); i++) {
        ScreenNdcCoord v0 = screen_verts[0];
        ScreenNdcCoord v1 = screen_verts[i];
        ScreenNdcCoord v2 = screen_verts[i + 1];
        
        // Sort vertices by y-coordinate: v0.y <= v1.y <= v2.y
        if (v0.y > v1.y) std::swap(v0, v1);
        if (v0.y > v2.y) std::swap(v0, v2);
        if (v1.y > v2.y) std::swap(v1, v2);

        // Clamp y range to screen bounds
        std::int64_t from_y = std::max<std::int64_t>(static_cast<std::int64_t>(std::ceil(v0.y)), 0);
        std::int64_t mid_y = std::min<std::int64_t>(static_cast<std::int64_t>(std::ceil(v1.y)), static_cast<std::int64_t>(window_.height - 1));
        std::int64_t to_y = std::min<std::int64_t>(static_cast<std::int64_t>(std::ceil(v2.y)), static_cast<std::int64_t>(window_.height - 1));

        // Calculate inverse slopes for edge interpolation
        FloatType inv_slope_v0v1 = (v1.y - v0.y) == 0 ? 0 : (v1.x - v0.x) / (v1.y - v0.y);
        FloatType inv_slope_v0v2 = (v2.y - v0.y) == 0 ? 0 : (v2.x - v0.x) / (v2.y - v0.y);
        FloatType inv_slope_v1v2 = (v2.y - v1.y) == 0 ? 0 : (v2.x - v1.x) / (v2.y - v1.y);

        // Rasterize upper triangle (v0 -> v1)
        for (std::int64_t y = from_y; y < mid_y; y++) {
            FloatType y_center = static_cast<FloatType>(y) + 0.5f;
            FloatType x01 = inv_slope_v0v1 * (y_center - v0.y) + v0.x;
            FloatType x02 = inv_slope_v0v2 * (y_center - v0.y) + v0.x;

            std::int64_t start_x = std::max<std::int64_t>(static_cast<std::int64_t>(std::floor(std::min(x01, x02))), 0);
            std::int64_t end_x = std::min<std::int64_t>(static_cast<std::int64_t>(std::ceil(std::max(x01, x02))), static_cast<std::int64_t>(window_.width - 1));
            
            for (std::int64_t x = start_x; x <= end_x; x++) {
                FloatType x_center = static_cast<FloatType>(x) + 0.5f;
                glm::vec3 bary = convertToBarycentricCoordinates(
                    { v0.x, v0.y }, { v1.x, v1.y }, { v2.x, v2.y }, { x_center, y_center });
                
                if (bary.x >= 0.0f && bary.y >= 0.0f && bary.z >= 0.0f) {
                    std::uint32_t colour = sample_texture(face, bary, v0, v1, v2);
                    FloatType inv_z = ComputeInvZndc(std::array<FloatType, 3>{bary.z, bary.x, bary.y}, 
                                                     std::array<FloatType, 3>{v0.z_ndc, v1.z_ndc, v2.z_ndc});
                    FloatType& depth = z_buffer_[y * window_.width + x];
                    if (inv_z > depth) {
                        depth = inv_z;
                        window_.setPixelColour(x, y, colour);
                    }
                }
            }
        }

        // Rasterize lower triangle (v1 -> v2)
        for (std::int64_t y = mid_y; y <= to_y; y++) {
            FloatType y_center = static_cast<FloatType>(y) + 0.5f;
            FloatType x12 = inv_slope_v1v2 * (y_center - v1.y) + v1.x;
            FloatType x02 = inv_slope_v0v2 * (y_center - v0.y) + v0.x;

            std::int64_t start_x = std::max<std::int64_t>(static_cast<std::int64_t>(std::floor(std::min(x12, x02))), 0);
            std::int64_t end_x = std::min<std::int64_t>(static_cast<std::int64_t>(std::ceil(std::max(x12, x02))), static_cast<std::int64_t>(window_.width - 1));
            
            for (std::int64_t x = start_x; x <= end_x; x++) {
                FloatType x_center = static_cast<FloatType>(x) + 0.5f;
                glm::vec3 bary = convertToBarycentricCoordinates(
                    { v0.x, v0.y }, { v1.x, v1.y }, { v2.x, v2.y }, { x_center, y_center });
                
                if (bary.x >= 0.0f && bary.y >= 0.0f && bary.z >= 0.0f) {
                    std::uint32_t colour = sample_texture(face, bary, v0, v1, v2);
                    FloatType inv_z = ComputeInvZndc(std::array<FloatType, 3>{bary.z, bary.x, bary.y}, 
                                                     std::array<FloatType, 3>{v0.z_ndc, v1.z_ndc, v2.z_ndc});
                    FloatType& depth = z_buffer_[y * window_.width + x];
                    if (inv_z > depth) {
                        depth = inv_z;
                        window_.setPixelColour(x, y, colour);
                    }
                }
            }
        }
    }
}
RayTriangleIntersection Renderer::find_closest_intersection(const glm::vec3& ray_origin, const glm::vec3& ray_dir, const std::vector<Face>& faces) noexcept {
    RayTriangleIntersection closest;
    closest.distanceFromCamera = std::numeric_limits<FloatType>::infinity();
    closest.triangleIndex = static_cast<std::size_t>(-1);
    
    
    for (std::size_t i = 0; i < faces.size(); ++i) {
        const Face& face = faces[i];
        FloatType t, u, v;
        
        if (IntersectRayTriangle(ray_origin, ray_dir,
                                  face.vertices[0], face.vertices[1], face.vertices[2],
                                  t, u, v) && t < closest.distanceFromCamera) {
            closest.distanceFromCamera = t;
            closest.intersectionPoint = ray_origin + ray_dir * t;
            closest.triangleIndex = i;
            closest.u = u;
            closest.v = v;
            
            // Calculate surface normal
            glm::vec3 geometric_normal = CalculateNormal(face.vertices[0], face.vertices[1], face.vertices[2]);
            
            // For proper lighting, the normal should face the ray origin (viewer)
            // If dot(normal, -ray_dir) < 0, the normal points away from viewer (backface)
            // In this case, flip it to face the viewer for proper shading
            if (glm::dot(geometric_normal, -ray_dir) < 0.0f) {
                geometric_normal = -geometric_normal;
            }
            closest.normal = geometric_normal;
            
            // Compute barycentric coordinate w = 1 - u - v
            FloatType w = 1.0f - u - v;
            
            // Interpolate texture coordinates using barycentric coordinates
            glm::vec2 uv_coord = face.texture_coords[0] * w +
                                 face.texture_coords[1] * u +
                                 face.texture_coords[2] * v;
            
            // Sample texture or use material color
            if (face.material.texture) {
                closest.colour = face.material.texture->sample(uv_coord.x, uv_coord.y);
            } else {
                closest.colour = face.material.colour;
            }
        }
    }
    
    return closest;
}
static inline Renderer::AABB tri_aabb(const Face& f) noexcept {
    glm::vec3 mn = glm::min(glm::min(f.vertices[0], f.vertices[1]), f.vertices[2]);
    glm::vec3 mx = glm::max(glm::max(f.vertices[0], f.vertices[1]), f.vertices[2]);
    return Renderer::AABB{mn, mx};
}
bool Renderer::intersect_aabb(const glm::vec3& ro, const glm::vec3& rd, const AABB& box, FloatType tmax) noexcept {
    glm::vec3 inv = glm::vec3(1.0f) / rd;
    glm::vec3 t0 = (box.min - ro) * inv;
    glm::vec3 t1 = (box.max - ro) * inv;
    glm::vec3 tmin = glm::min(t0, t1);
    glm::vec3 tmaxv = glm::max(t0, t1);
    FloatType t_enter = std::max(std::max(tmin.x, tmin.y), tmin.z);
    FloatType t_exit = std::min(std::min(tmaxv.x, tmaxv.y), tmaxv.z);
    return t_enter <= t_exit && t_exit >= 0.0f && t_enter <= tmax;
}
std::pair<std::vector<int>, std::vector<Renderer::BVHNode>> Renderer::build_bvh(const std::vector<Face>& faces) noexcept {
    std::vector<int> tri_indices(faces.size());
    std::iota(tri_indices.begin(), tri_indices.end(), 0);
    struct Cent { glm::vec3 c; Renderer::AABB b; };
    std::vector<Cent> data(faces.size());
    for (std::size_t i = 0; i < faces.size(); ++i) {
        auto b = tri_aabb(faces[i]);
        glm::vec3 c = (faces[i].vertices[0] + faces[i].vertices[1] + faces[i].vertices[2]) / 3.0f;
        data[i] = Cent{c, b};
    }
    std::vector<Renderer::BVHNode> nodes;
    
    // Helper function to compute surface area of AABB
    auto surface_area = [](const Renderer::AABB& box) -> FloatType {
        glm::vec3 extent = box.max - box.min;
        return 2.0f * (extent.x * extent.y + extent.y * extent.z + extent.z * extent.x);
    };
    
    // SAH constants
    constexpr FloatType traversal_cost = 1.0f;
    constexpr FloatType intersection_cost = 1.0f;
    constexpr int sah_buckets = 12;
    constexpr int leaf_threshold = 4;
    
    std::function<int(int,int)> build = [&](int start, int end) -> int {
        // Compute bounding box for all primitives
        Renderer::AABB box{glm::vec3(std::numeric_limits<float>::infinity()), glm::vec3(-std::numeric_limits<float>::infinity())};
        Renderer::AABB cbox{box.min, box.max};
        for (int i = start; i < end; ++i) {
            box.min = glm::min(box.min, data[tri_indices[i]].b.min);
            box.max = glm::max(box.max, data[tri_indices[i]].b.max);
            cbox.min = glm::min(cbox.min, data[tri_indices[i]].c);
            cbox.max = glm::max(cbox.max, data[tri_indices[i]].c);
        }
        
        int count = end - start;
        int node_index = (int)nodes.size();
        nodes.push_back(Renderer::BVHNode{box, -1, -1, start, count});
        
        // Create leaf if too few primitives
        if (count <= leaf_threshold) {
            return node_index;
        }
        
        // Find best split using SAH
        FloatType best_cost = std::numeric_limits<FloatType>::infinity();
        int best_axis = -1;
        int best_split = -1;
        
        glm::vec3 extent = cbox.max - cbox.min;
        FloatType parent_area = surface_area(box);
        
        // Try each axis
        for (int axis = 0; axis < 3; ++axis) {
            // Skip degenerate axes
            if (extent[axis] < 1e-6f) continue;
            
            // Initialize buckets
            struct Bucket {
                int count = 0;
                Renderer::AABB bounds{glm::vec3(std::numeric_limits<float>::infinity()), 
                                      glm::vec3(-std::numeric_limits<float>::infinity())};
            };
            std::array<Bucket, sah_buckets> buckets;
            
            // Assign primitives to buckets
            for (int i = start; i < end; ++i) {
                FloatType centroid = data[tri_indices[i]].c[axis];
                int bucket_idx = static_cast<int>(sah_buckets * 
                    ((centroid - cbox.min[axis]) / extent[axis]));
                bucket_idx = Clamp(bucket_idx, 0, sah_buckets - 1);
                
                buckets[bucket_idx].count++;
                buckets[bucket_idx].bounds.min = glm::min(buckets[bucket_idx].bounds.min, 
                                                          data[tri_indices[i]].b.min);
                buckets[bucket_idx].bounds.max = glm::max(buckets[bucket_idx].bounds.max, 
                                                          data[tri_indices[i]].b.max);
            }
            
            // Compute costs for splitting after each bucket
            for (int split = 0; split < sah_buckets - 1; ++split) {
                // Left partition [0, split]
                Renderer::AABB left_box{glm::vec3(std::numeric_limits<float>::infinity()), 
                                       glm::vec3(-std::numeric_limits<float>::infinity())};
                int left_count = 0;
                for (int i = 0; i <= split; ++i) {
                    if (buckets[i].count > 0) {
                        left_box.min = glm::min(left_box.min, buckets[i].bounds.min);
                        left_box.max = glm::max(left_box.max, buckets[i].bounds.max);
                        left_count += buckets[i].count;
                    }
                }
                
                // Right partition (split, sah_buckets - 1]
                Renderer::AABB right_box{glm::vec3(std::numeric_limits<float>::infinity()), 
                                        glm::vec3(-std::numeric_limits<float>::infinity())};
                int right_count = 0;
                for (int i = split + 1; i < sah_buckets; ++i) {
                    if (buckets[i].count > 0) {
                        right_box.min = glm::min(right_box.min, buckets[i].bounds.min);
                        right_box.max = glm::max(right_box.max, buckets[i].bounds.max);
                        right_count += buckets[i].count;
                    }
                }
                
                // Skip invalid splits
                if (left_count == 0 || right_count == 0) continue;
                
                // SAH cost = traversal_cost + (left_area/parent_area * left_count + right_area/parent_area * right_count) * intersection_cost
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
        
        // Leaf cost (no traversal, just intersection tests)
        FloatType leaf_cost = count * intersection_cost;
        
        // Create leaf if SAH suggests it's better
        if (best_cost >= leaf_cost || best_axis == -1) {
            return node_index;
        }
        
        // Partition primitives based on best split
        auto mid_iter = std::partition(tri_indices.begin() + start, tri_indices.begin() + end,
            [&](int idx) {
                FloatType centroid = data[idx].c[best_axis];
                int bucket_idx = static_cast<int>(sah_buckets * 
                    ((centroid - cbox.min[best_axis]) / extent[best_axis]));
                bucket_idx = Clamp(bucket_idx, 0, sah_buckets - 1);
                return bucket_idx <= best_split;
            });
        
        int mid = static_cast<int>(mid_iter - tri_indices.begin());
        
        // Ensure we don't create empty partitions
        if (mid == start || mid == end) {
            mid = (start + end) / 2;
        }
        
        // Recursively build left and right children
        int left = build(start, mid);
        int right = build(mid, end);
        nodes[node_index].left = left;
        nodes[node_index].right = right;
        nodes[node_index].count = 0;
        return node_index;
    };
    
    build(0, (int)faces.size());
    
    return {std::move(tri_indices), std::move(nodes)};
}
RayTriangleIntersection Renderer::find_closest_intersection_bvh(const glm::vec3& ray_origin, const glm::vec3& ray_dir) const noexcept {
    RayTriangleIntersection closest;
    closest.distanceFromCamera = std::numeric_limits<FloatType>::infinity();
    closest.triangleIndex = static_cast<std::size_t>(-1);
    if (bvh_nodes_.empty()) return closest;
    
    const std::vector<Face>& faces = world_.all_faces();
    std::vector<int> stack;
    stack.reserve(64);
    stack.push_back(0);
    while (!stack.empty()) {
        int ni = stack.back();
        stack.pop_back();
        const BVHNode& n = bvh_nodes_[ni];
        if (!intersect_aabb(ray_origin, ray_dir, n.box, closest.distanceFromCamera)) continue;
        if (n.count == 0) {
            stack.push_back(n.left);
            stack.push_back(n.right);
        } else {
            for (int i = 0; i < n.count; ++i) {
                int tri_index = bvh_tri_indices_[n.start + i];
                const Face& face = faces[tri_index];
                FloatType t,u,v;
                if (IntersectRayTriangle(ray_origin, ray_dir, face.vertices[0], face.vertices[1], face.vertices[2], t,u,v) && t < closest.distanceFromCamera) {
                    closest.distanceFromCamera = t;
                    closest.intersectionPoint = ray_origin + ray_dir * t;
                    closest.triangleIndex = tri_index;
                    closest.u = u;
                    closest.v = v;
                    glm::vec3 geometric_normal = CalculateNormal(face.vertices[0], face.vertices[1], face.vertices[2]);
                    if (glm::dot(geometric_normal, -ray_dir) < 0.0f) geometric_normal = -geometric_normal;
                    closest.normal = geometric_normal;
                    FloatType w = 1.0f - u - v;
                    glm::vec2 uv_coord = face.texture_coords[0] * w + face.texture_coords[1] * u + face.texture_coords[2] * v;
                    if (face.material.texture) {
                        closest.colour = face.material.texture->sample(uv_coord.x, uv_coord.y);
                    } else {
                        closest.colour = face.material.colour;
                    }
                }
            }
        }
    }
    return closest;
}
bool Renderer::is_in_shadow(const glm::vec3& point, const glm::vec3& light_pos, const std::vector<Face>& faces) noexcept {
    glm::vec3 to_light = light_pos - point;
    FloatType light_distance = glm::length(to_light);
    glm::vec3 light_dir = to_light / light_distance;
    
    // Use the intersection point directly as shadow ray origin
    // The epsilon in intersect_ray_triangle will handle avoiding self-intersection
    glm::vec3 shadow_ray_origin = point;
    
    for (std::size_t i = 0; i < faces.size(); i++) {
        const Face& face = faces[i];
        FloatType t, u, v;
        
        bool hit = IntersectRayTriangle(
            shadow_ray_origin,
            light_dir,
            face.vertices[0],
            face.vertices[1],
            face.vertices[2],
            t, u, v
        );
        
        // Check if intersection is between point and light
        // Use a small epsilon to avoid self-intersection
        constexpr FloatType min_t = 0.001f;
        if (hit && t > min_t && t < light_distance) {
            return true;  // In shadow
        }
    }
    
    return false;  // Not in shadow
}
bool Renderer::is_in_shadow_bvh(const glm::vec3& point, const glm::vec3& light_pos) const noexcept {
    glm::vec3 to_light = light_pos - point;
    FloatType light_distance = glm::length(to_light);
    glm::vec3 light_dir = to_light / light_distance;
    if (bvh_nodes_.empty()) return false;
    
    const std::vector<Face>& faces = world_.all_faces();
    std::vector<int> stack;
    stack.reserve(64);
    stack.push_back(0);
    constexpr FloatType min_t = 0.001f;
    while (!stack.empty()) {
        int ni = stack.back();
        stack.pop_back();
        const BVHNode& n = bvh_nodes_[ni];
        if (!intersect_aabb(point, light_dir, n.box, light_distance)) continue;
        if (n.count == 0) {
            stack.push_back(n.left);
            stack.push_back(n.right);
        } else {
            for (int i = 0; i < n.count; ++i) {
                int tri_index = bvh_tri_indices_[n.start + i];
                const Face& face = faces[tri_index];
                FloatType t,u,v;
                bool hit = IntersectRayTriangle(point, light_dir, face.vertices[0], face.vertices[1], face.vertices[2], t,u,v);
                if (hit && t > min_t && t < light_distance) {
                    return true;
                }
            }
        }
    }
    return false;
}
void Renderer::process_rows(int y0, int y1) noexcept {
    const Camera& camera = *current_camera_;
    const std::vector<Face>& all_faces = world_.all_faces();
    const glm::vec3& light_pos = world_.light_position();
    const FloatType light_intensity = world_.light_intensity();
    
    for (int y = y0; y < y1; ++y) {
        for (int x = 0; x < static_cast<int>(window_.width); x++) {
            auto [ray_origin, ray_dir] = camera.generate_ray(x, y, window_.width, window_.height, aspect_ratio_);
            ColourHDR final_hdr = trace_ray(ray_origin, ray_dir, 0);
            Colour final_colour = tonemap_and_gamma_correct(final_hdr, gamma_);
            window_.setPixelColour(x, y, final_colour);
        }
    }
}

ColourHDR Renderer::trace_ray(const glm::vec3& ray_origin, const glm::vec3& ray_dir, int depth) const noexcept {
    constexpr int MAX_DEPTH = 5;  // Maximum recursion depth for reflections
    
    if (depth >= MAX_DEPTH) {
        return ColourHDR(0.0f, 0.0f, 0.0f);  // Return black after max depth
    }
    
    RayTriangleIntersection intersection = find_closest_intersection_bvh(ray_origin, ray_dir);
    
    if (intersection.triangleIndex != static_cast<std::size_t>(-1)) {
        const std::vector<Face>& all_faces = world_.all_faces();
        const glm::vec3& light_pos = world_.light_position();
        const FloatType light_intensity = world_.light_intensity();
        const Camera& camera = *current_camera_;
        
        const Face& face = all_faces[intersection.triangleIndex];
        
        // Handle mirror materials with perfect reflection
        if (face.material.is_mirror) {
            // Calculate reflection direction: r = d - 2(d·n)n
            glm::vec3 reflected_dir = glm::reflect(ray_dir, intersection.normal);
            
            // Offset ray origin slightly along normal to avoid self-intersection
            constexpr FloatType epsilon = 0.001f;
            glm::vec3 offset_origin = intersection.intersectionPoint + intersection.normal * epsilon;
            
            // Trace reflected ray recursively
            return trace_ray(offset_origin, reflected_dir, depth + 1);
        }
        
        // Non-mirror materials: use standard lighting
        RayTriangleIntersection old_intersection = intersection;
        if (old_intersection.triangleIndex != static_cast<std::size_t>(-1)) {
            glm::vec3 to_light_hit = light_pos - old_intersection.intersectionPoint;
            FloatType dist_light_hit = glm::length(to_light_hit);
            glm::vec3 to_camera_hit = camera.position_ - old_intersection.intersectionPoint;
            bool in_shadow = is_in_shadow_bvh(old_intersection.intersectionPoint, light_pos);
            ColourHDR hdr_colour = ColourHDR::from_srgb(old_intersection.colour, gamma_);
            FloatType ambient = 0.025f;
            FloatType lambertian = 0.0f;
            FloatType specular = 0.0f;
            if (!in_shadow) {
                FloatType w = 1.0f - old_intersection.u - old_intersection.v;
                if (face.material.shading == Material::Shading::Gouraud) {
                    FloatType lam_v[3] = {0.0f, 0.0f, 0.0f};
                    FloatType spec_v[3] = {0.0f, 0.0f, 0.0f};
                    for (int k = 0; k < 3; ++k) {
                        glm::vec3 to_light_v = light_pos - face.vertices[k];
                        FloatType dist_v = glm::length(to_light_v);
                        glm::vec3 to_camera_v = camera.position_ - face.vertices[k];
                        lam_v[k] = compute_lambertian_lighting(face.vertex_normals[k], to_light_v, dist_v, light_intensity);
                        if (lam_v[k] > 0.0f) {
                            spec_v[k] = compute_specular_lighting(face.vertex_normals[k], to_light_v, to_camera_v, dist_v, light_intensity, face.material.shininess);
                        }
                    }
                    lambertian = w * lam_v[0] + old_intersection.u * lam_v[1] + old_intersection.v * lam_v[2];
                    specular = w * spec_v[0] + old_intersection.u * spec_v[1] + old_intersection.v * spec_v[2];
                } else {
                    glm::vec3 n_interp = glm::normalize(w * face.vertex_normals[0] + old_intersection.u * face.vertex_normals[1] + old_intersection.v * face.vertex_normals[2]);
                    lambertian = compute_lambertian_lighting(n_interp, to_light_hit, dist_light_hit, light_intensity);
                    if (lambertian > 0.0f) {
                        specular = compute_specular_lighting(n_interp, to_light_hit, to_camera_hit, dist_light_hit, light_intensity, face.material.shininess);
                    }
                }
            }
            FloatType diffuse_component = ambient + (1.0f - ambient) * lambertian;
            return hdr_colour * diffuse_component + ColourHDR(specular, specular, specular);
        }
    }
    
    // No intersection - return background color (black)
    return ColourHDR(0.0f, 0.0f, 0.0f);
}
FloatType Renderer::compute_lambertian_lighting(const glm::vec3& normal, const glm::vec3& to_light, FloatType distance, FloatType intensity) noexcept {
    // Lambert's cosine law: I = I₀ * cos(θ) / (4πr²)
    // where θ is the angle between surface normal and light direction
    
    glm::vec3 light_dir = glm::normalize(to_light);
    
    // Calculate cos(θ) - angle of incidence
    FloatType cos_theta = glm::dot(normal, light_dir);
    
    // If cos_theta <= 0, the surface is backfacing (facing away from light)
    if (cos_theta <= 0.0f) {
        return 0.0f;  // No lighting for backfacing surfaces
    }
    
    // Distance attenuation with lambertian cosine
    // Using 4π instead of 2π for proper spherical distribution
    FloatType attenuation = (intensity * cos_theta) / (4.0f * std::numbers::pi * distance * distance);
    
    return Clamp(attenuation, 0.0f, 1.0f);
}

FloatType Renderer::compute_specular_lighting(const glm::vec3& normal, const glm::vec3& to_light, const glm::vec3& to_camera, FloatType distance, FloatType intensity, FloatType shininess) noexcept {
    // Blinn-Phong specular reflection
    // Important: This should only be called when the surface is front-facing to the viewer
    
    glm::vec3 light_dir = glm::normalize(to_light);
    glm::vec3 view_dir = glm::normalize(to_camera);
    
    // Check that we're viewing the front face (N·V > 0)
    // This prevents specular on back-facing surfaces
    FloatType n_dot_v = glm::dot(normal, view_dir);
    if (n_dot_v <= 0.0f) {
        return 0.0f;
    }
    
    // Calculate halfway vector
    glm::vec3 halfway = glm::normalize(light_dir + view_dir);
    
    // Calculate cos(α) - angle between normal and halfway vector
    FloatType cos_alpha = glm::dot(normal, halfway);
    
    if (cos_alpha <= 0.0f) {
        return 0.0f;
    }
    
    // Specular term with distance attenuation
    FloatType spec_term = std::pow(cos_alpha, shininess);
    FloatType attenuation = (intensity * spec_term) / (4.0f * std::numbers::pi * distance * distance);
    
    return Clamp(attenuation, 0.0f, 1.0f);
}

FloatType Renderer::aces_tonemap(FloatType hdr_value) noexcept {
    // ACES Filmic tonemapping approximation
    // https://knarkowicz.wordpress.com/2016/01/06/aces-filmic-tone-mapping-curve/
    const FloatType a = 2.51f;
    const FloatType b = 0.03f;
    const FloatType c = 2.43f;
    const FloatType d = 0.59f;
    const FloatType e = 0.14f;
    
    FloatType numerator = hdr_value * (a * hdr_value + b);
    FloatType denominator = hdr_value * (c * hdr_value + d) + e;
    
    return Clamp(numerator / denominator, 0.0f, 1.0f);
}

Colour Renderer::tonemap_and_gamma_correct(const ColourHDR& hdr, FloatType gamma) noexcept {
    // Step 1: Tonemapping (HDR -> LDR, but still in linear space)
    FloatType r_ldr = aces_tonemap(hdr.red);
    FloatType g_ldr = aces_tonemap(hdr.green);
    FloatType b_ldr = aces_tonemap(hdr.blue);
    
    // Step 2: Gamma correction (linear -> sRGB)
    FloatType r_out, g_out, b_out;
    if (gamma == 1.0f) {
        // No gamma correction
        r_out = r_ldr;
        g_out = g_ldr;
        b_out = b_ldr;
    } else {
        // Apply gamma correction
        r_out = std::pow(r_ldr, 1.0f / gamma);
        g_out = std::pow(g_ldr, 1.0f / gamma);
        b_out = std::pow(b_ldr, 1.0f / gamma);
    }
    
    // Step 3: Convert to 8-bit
    return Colour{
        static_cast<std::uint8_t>(Clamp(r_out * 255.0f, 0.0f, 255.0f)),
        static_cast<std::uint8_t>(Clamp(g_out * 255.0f, 0.0f, 255.0f)),
        static_cast<std::uint8_t>(Clamp(b_out * 255.0f, 0.0f, 255.0f))
    };
}

void Renderer::worker_thread() noexcept {
    while (true) {
        // Wait for frame start signal
        frame_barrier_.arrive_and_wait();
        
        // Process tiles until all are done
        const int num_tiles = (window_.height + TileHeight - 1) / TileHeight;
        while (true) {
            int tile_idx = tile_counter_.fetch_add(1, std::memory_order_relaxed);
            if (tile_idx >= num_tiles) break;
            
            int y0 = tile_idx * TileHeight;
            int y1 = std::min(y0 + TileHeight, static_cast<int>(window_.height));
            process_rows(y0, y1);
        }
        
        // Signal frame completion
        frame_barrier_.arrive_and_wait();
    }
}
