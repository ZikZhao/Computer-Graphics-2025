#include "world.hpp"

void Camera::start_orbiting(glm::vec3 target) {
    orbit_target_ = target;
    last_orbit_time_ = std::chrono::system_clock::now().time_since_epoch().count();
}
void Camera::orbiting() {
    auto now = std::chrono::system_clock::now().time_since_epoch().count();
    if (now - last_orbit_time_ > OrbitInterval) {
        constexpr static FloatType angle_increment = glm::radians(0.5f);
        FloatType cos_angle = std::cos(angle_increment);
        FloatType sin_angle = std::sin(angle_increment);
        glm::vec3& up = orientation_[1];
        glm::vec3 k_cross_pos = glm::cross(up, position_);
        FloatType k_dot_pos = glm::dot(up, position_);
        position_ = position_ * cos_angle + 
                   k_cross_pos * sin_angle + 
                   up * k_dot_pos * (1.0f - cos_angle);
        orientation_[2] = glm::normalize(orbit_target_ - position_);
        orientation_[0] = glm::normalize(glm::cross(orientation_[2], up));
        last_orbit_time_ = now;
    }
}
void Camera::stop_orbiting() {
    last_orbit_time_ = std::numeric_limits<std::int64_t>::max();
}
void Camera::rotate(FloatType angle_x, FloatType angle_y) {
    if (angle_x != 0.0f) {
        glm::mat3 rotation_y = glm::mat3(
            std::cos(angle_x), 0.0f, std::sin(angle_x),
            0.0f, 1.0f, 0.0f,
            -std::sin(angle_x), 0.0f, std::cos(angle_x)
        );
        orientation_ = rotation_y * orientation_;
    }
    if (angle_y != 0.0f) {
        glm::mat3 rotation_x = glm::mat3(
            1.0f, 0.0f, 0.0f,
            0.0f, std::cos(angle_y), -std::sin(angle_y),
            0.0f, std::sin(angle_y), std::cos(angle_y)
        );
        orientation_ = rotation_x * orientation_;
    }
}
void Camera::handle_event(const SDL_Event& event) {
    if (event.type == SDL_KEYDOWN) {
        constexpr FloatType move_step = 0.1f;
        glm::vec3 movement(0.0f);
        switch (event.key.keysym.sym) {
        case SDLK_w:
            movement.y = move_step;
            break;
        case SDLK_s:
            movement.y = -move_step;
            break;
        case SDLK_a:
            movement.x = -move_step;
            break;
        case SDLK_d:
            movement.x = move_step;
            break;
        case SDLK_q:
            movement.z = move_step;
            break;
        case SDLK_e:
            movement.z = -move_step;
            break;
        case SDLK_UP:
            rotate(0.0f, -glm::radians(1.0f));
            return;
        case SDLK_DOWN:
            rotate(0.0f, glm::radians(1.0f));
            return;
        case SDLK_LEFT:
            rotate(-glm::radians(1.0f), 0.0f);
            return;
        case SDLK_RIGHT:
            rotate(glm::radians(1.0f), 0.0f);
            return;
        // case SDLK_t:
        //     start_orbiting(glm::vec3(0.0f, 0.0f, 0.0f));
        //     return;
        case SDLK_o:
            if (last_orbit_time_ == std::numeric_limits<std::int64_t>::max()) {
                last_orbit_time_ = std::chrono::system_clock::now().time_since_epoch().count();
            } else {
                last_orbit_time_ = std::numeric_limits<std::int64_t>::max();
            }
            return;
        }
        position_ += orientation_ * movement;
    }
}
glm::vec4 Camera::world_to_clip(const glm::vec3& vertex, double aspect_ratio) const noexcept {
    // View transformation
    glm::vec3 view_vector = vertex - position_;
    glm::mat3 view_rotation = glm::transpose(orientation_);
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

Object::Object(const std::string& name) : name_(name), colour_{255, 255, 255} {}
void Object::set_colour(const Colour& colour) {
    colour_ = colour;
}
void Object::add_face(const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3) {
    faces_.push_back({{v1, v2, v3}, colour_});
}

Renderer::Renderer(DrawingWindow& window) noexcept
    : window_(window), z_buffer_(window.width * window.height, std::numeric_limits<FloatType>::infinity()) {}
void Renderer::clear() noexcept {
    z_buffer_.assign(window_.width * window_.height, std::numeric_limits<FloatType>::infinity());
}
ScreenNdcCoord Renderer::ndc_to_screen(const glm::vec3& ndc) const noexcept {
    return ScreenNdcCoord{
        (ndc.x + 1.0f) * 0.5f * window_.width,
        (1.0f - ndc.y) * 0.5f * window_.height,
        ndc.z
    };
}

bool Renderer::inside_plane(const glm::vec4& v, ClipPlane plane) noexcept {
    // In clip space, frustum planes are: -w <= x,y,z <= w
    // But since w can be negative (w = -z_view), we need to handle signs correctly
    switch (plane) {
        case ClipPlane::Left:   return v.x >= -std::abs(v.w);
        case ClipPlane::Right:  return v.x <= std::abs(v.w);
        case ClipPlane::Bottom: return v.y >= -std::abs(v.w);
        case ClipPlane::Top:    return v.y <= std::abs(v.w);
        case ClipPlane::Near:   return v.z >= -std::abs(v.w);
        case ClipPlane::Far:    return v.z <= std::abs(v.w);
    }
    return false;
}
FloatType Renderer::compute_intersection_t(const glm::vec4& v0, const glm::vec4& v1, ClipPlane plane) noexcept {
    FloatType d0, d1;
    switch (plane) {
        case ClipPlane::Left:
            d0 = v0.x + std::abs(v0.w);
            d1 = v1.x + std::abs(v1.w);
            break;
        case ClipPlane::Right:
            d0 = std::abs(v0.w) - v0.x;
            d1 = std::abs(v1.w) - v1.x;
            break;
        case ClipPlane::Bottom:
            d0 = v0.y + std::abs(v0.w);
            d1 = v1.y + std::abs(v1.w);
            break;
        case ClipPlane::Top:
            d0 = std::abs(v0.w) - v0.y;
            d1 = std::abs(v1.w) - v1.y;
            break;
        case ClipPlane::Near:
            d0 = v0.z + std::abs(v0.w);
            d1 = v1.z + std::abs(v1.w);
            break;
        case ClipPlane::Far:
            d0 = std::abs(v0.w) - v0.z;
            d1 = std::abs(v1.w) - v1.z;
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
        }
    };
}
InplaceVector<ClipVertex, 9> Renderer::clip_against_plane(
    const InplaceVector<ClipVertex, 9>& input,
    ClipPlane plane) noexcept {

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
InplaceVector<ClipVertex, 9> Renderer::clip_triangle(
    const Camera& camera,
    const Face& face) noexcept {

    // Transform to clip space
    InplaceVector<ClipVertex, 9> polygon = {
        ClipVertex{camera.world_to_clip(face.vertices[0], aspect_ratio_), face.colour},
        ClipVertex{camera.world_to_clip(face.vertices[1], aspect_ratio_), face.colour},
        ClipVertex{camera.world_to_clip(face.vertices[2], aspect_ratio_), face.colour}
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
void Renderer::rasterize_polygon(
    const InplaceVector<ClipVertex, 9>& polygon,
    const Camera& camera) noexcept {

    if (polygon.size() < 3) return;
    
    // Convert all vertices to screen space
    InplaceVector<ScreenNdcCoord, 9> screen_verts;
    for (size_t i = 0; i < polygon.size(); i++) {
        glm::vec3 ndc = camera.clip_to_ndc(polygon[i].position_clip);
        screen_verts.push_back(ndc_to_screen(ndc));
    }
    
    // Triangle fan: (0, i, i+1) for i = 1..n-2
    for (size_t i = 1; i + 1 < polygon.size(); i++) {
        std::array<ScreenNdcCoord, 3> tri_screen = {
            screen_verts[0],
            screen_verts[i],
            screen_verts[i + 1]
        };
        
        std::uint32_t colour = polygon[0].colour;
        
        // Rasterize triangle using existing code
        ScreenNdcCoord v0 = tri_screen[0];
        ScreenNdcCoord v1 = tri_screen[1];
        ScreenNdcCoord v2 = tri_screen[2];
        
        if (v0.y > v1.y) std::swap(v0, v1);
        if (v0.y > v2.y) std::swap(v0, v2);
        if (v1.y > v2.y) std::swap(v1, v2);

        FloatType from_y = std::max<FloatType>(v0.y, 0);
        FloatType mid_y = std::min<FloatType>(v1.y, window_.height - 1);
        FloatType to_y = std::min<FloatType>(v2.y, window_.height - 1);

        FloatType inv_slope_v0v1 = (v1.y - v0.y) == 0 ? 0 : (v1.x - v0.x) / (v1.y - v0.y);
        FloatType inv_slope_v0v2 = (v2.y - v0.y) == 0 ? 0 : (v2.x - v0.x) / (v2.y - v0.y);
        FloatType inv_slope_v1v2 = (v2.y - v1.y) == 0 ? 0 : (v2.x - v1.x) / (v2.y - v1.y);

        for (FloatType y = from_y; y < mid_y; y++) {
            FloatType x01 = inv_slope_v0v1 * (y - v0.y) + v0.x;
            FloatType x02 = inv_slope_v0v2 * (y - v0.y) + v0.x;

            std::size_t start_x = std::max<std::int64_t>(std::round(std::min(x01, x02)) - 1, 0);
            std::size_t end_x = std::min<std::size_t>(std::round(std::max(x01, x02)), window_.width - 1);
            for (std::size_t x = start_x; x <= end_x; x++) {
                glm::vec3 bary = convertToBarycentricCoordinates(
                    { v0.x, v0.y }, { v1.x, v1.y }, { v2.x, v2.y }, { static_cast<FloatType>(x), static_cast<FloatType>(y) });
                FloatType z_ndc = ComputeZndc(std::array<FloatType, 3>{bary.z, bary.x, bary.y}, std::array<FloatType, 3>{ v0.z_ndc, v1.z_ndc, v2.z_ndc });
                std::int64_t yi = static_cast<std::int64_t>(y);
                if (x < window_.width && yi >= 0 && yi < static_cast<std::int64_t>(window_.height)) {
                    FloatType& depth = z_buffer_[yi * window_.width + x];
                    if (z_ndc >= 0.0f && z_ndc <= 1.0f && z_ndc < depth) {
                        depth = z_ndc;
                        window_.setPixelColour(x, yi, colour);
                    }
                }
            }
        }

        for (FloatType y = mid_y; y < to_y; y++) {
            FloatType x12 = inv_slope_v1v2 * (y - v1.y) + v1.x;
            FloatType x02 = inv_slope_v0v2 * (y - v0.y) + v0.x;

            std::size_t start_x = std::max<std::int64_t>(std::round(std::min(x12, x02)) - 1, 0);
            std::size_t end_x = std::min<std::size_t>(std::round(std::max(x12, x02)), window_.width - 1);
            for (std::size_t x = start_x; x <= end_x; x++) {
                glm::vec3 bary = convertToBarycentricCoordinates(
                    { v0.x, v0.y }, { v1.x, v1.y }, { v2.x, v2.y }, { static_cast<FloatType>(x), static_cast<FloatType>(y) });
                FloatType z_ndc = ComputeZndc(std::array<FloatType, 3>{bary.z, bary.x, bary.y}, std::array<FloatType, 3>{ v0.z_ndc, v1.z_ndc, v2.z_ndc });
                std::int64_t yi = static_cast<std::int64_t>(y);
                if (x < window_.width && yi >= 0 && yi < static_cast<std::int64_t>(window_.height)) {
                    FloatType& depth = z_buffer_[yi * window_.width + x];
                    if (z_ndc >= 0.0f && z_ndc <= 1.0f && z_ndc < depth) {
                        depth = z_ndc;
                        window_.setPixelColour(x, yi, colour);
                    }
                }
            }
        }
    }
}
void Renderer::render(const Camera& camera, const Face& face) noexcept {
    aspect_ratio_ = static_cast<double>(window_.width) / window_.height;
    switch (mode_) {
    case Wireframe:
        wireframe_render(camera, face);
        break;
    case Rasterized:
        rasterized_render(camera, face);
        break;
    case Raytraced:
        raytraced_render(camera, face);
        break;
    }
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
        }
    }
}
void Renderer::wireframe_render(const Camera& camera, const Face& face) noexcept {
    // Use clipping pipeline
    auto clipped = clip_triangle(camera, face);
    if (clipped.size() < 3) return;
    
    // Convert to screen space
    InplaceVector<ScreenNdcCoord, 9> screen_verts;
    for (size_t i = 0; i < clipped.size(); i++) {
        glm::vec3 ndc = camera.clip_to_ndc(clipped[i].position_clip);
        screen_verts.push_back(ndc_to_screen(ndc));
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
                FloatType z_ndc = ComputeZndc(progress, std::array<FloatType, 2>{from.z_ndc, to.z_ndc});
                FloatType& depth = z_buffer_[y * window_.width + x];
                if (z_ndc >= 0.0f && z_ndc <= 1.0f && z_ndc < depth) {
                    depth = z_ndc;
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
                FloatType z_ndc = ComputeZndc(progress, std::array<FloatType, 2>{from.z_ndc, to.z_ndc});
                FloatType& depth = z_buffer_[y * window_.width + x];
                if (z_ndc >= 0.0f && z_ndc <= 1.0f && z_ndc < depth) {
                    depth = z_ndc;
                    window_.setPixelColour(x, y, colour);
                }
            }
        }
    }
}
void Renderer::rasterized_render(const Camera& camera, const Face& face) noexcept {
    // Use clipping pipeline and polygon rasterization
    auto clipped = clip_triangle(camera, face);
    rasterize_polygon(clipped, camera);
}
void Renderer::raytraced_render(const Camera& camera, const Face& face) noexcept {
    // Raytracing not implemented yet
}

void Group::load_file(std::string filename) {
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
            Colour colour = materials_[colour_name];
            current_obj->set_colour(colour);
        } else if (type == "v") {
            assert(current_obj != objects_.end());
            FloatType x, y, z;
            iss >> x >> y >> z;
            vertices_.emplace_back(x / 3, y / 3, z / 3);
        } else if (type == "f") {
            assert(current_obj != objects_.end());
            int v1, v2, v3;
            char slash; // to consume the '/' characters
            iss >> v1 >> slash >> v2 >> slash >> v3 >> slash;
            current_obj->add_face(vertices_[v1 - 1], vertices_[v2 - 1], vertices_[v3 - 1]);
        }
    }
}
void Group::draw(Renderer& renderer, const Camera& camera) const noexcept {
    renderer.clear();
    for (const auto& object : objects_) {
        for (const auto& face : object.faces_) {
            renderer.render(camera, face);
        }
    }
}
void Group::load_materials(std::string filename) {
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
            current_material->second = Colour{
                static_cast<std::uint8_t>(Clamp(r * 255.0f, 0.0f, 255.0f)),
                static_cast<std::uint8_t>(Clamp(g * 255.0f, 0.0f, 255.0f)),
                static_cast<std::uint8_t>(Clamp(b * 255.0f, 0.0f, 255.0f))
            };
        }
    }
}

void World::load_files(const std::vector<std::string>& filenames) {
    for (const auto& filename : filenames) {
        Group group;
        group.load_file(filename);
        groups_.emplace_back(std::move(group));
    }
}
void World::draw(Renderer& renderer) const noexcept {
    for (const auto& group : groups_) {
        group.draw(renderer, camera_);
    }
}
void World::handle_event(const SDL_Event& event) noexcept {
    camera_.handle_event(event);
}
void World::orbiting() noexcept {
    camera_.orbiting();
}
