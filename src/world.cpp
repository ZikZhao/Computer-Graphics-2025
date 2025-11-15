#include "world.hpp"

float ComputeZndc(std::array<float, 3> bary, std::array<float, 3> vertices_z_ndc) {
    float inv_z = bary[0] / vertices_z_ndc[0] +
                  bary[1] / vertices_z_ndc[1] +
                  bary[2] / vertices_z_ndc[2];
    return 1.0f / inv_z;
}

float ComputeZndc(float progress, std::array<float, 2> vertices_z_ndc) {
    float inv_z = (1.0f - progress) / vertices_z_ndc[0] + progress / vertices_z_ndc[1];
    return 1.0f / inv_z;
}

void Camera::orbiting() {
    auto now = std::chrono::system_clock::now().time_since_epoch().count();
    if (now - last_orbit_time_ > OrbitInterval) {
        constexpr static float angle_increment = glm::radians(0.5f);
        float cos_angle = std::cos(angle_increment);
        float sin_angle = std::sin(angle_increment);
        glm::vec3& up = orientation_[1];
        glm::vec3 k_cross_pos = glm::cross(up, position_);
        float k_dot_pos = glm::dot(up, position_);
        position_ = position_ * cos_angle + 
                   k_cross_pos * sin_angle + 
                   up * k_dot_pos * (1.0f - cos_angle);
        orientation_[2] = glm::normalize(orbit_target_ - position_);
        orientation_[0] = glm::normalize(glm::cross(orientation_[2], up));
        last_orbit_time_ = now;
    }
}
void Camera::set_orbit(glm::vec3 target) {
    orbit_target_ = target;
    last_orbit_time_ = std::chrono::system_clock::now().time_since_epoch().count();
}
void Camera::stop_orbit() {
    last_orbit_time_ = std::numeric_limits<std::int64_t>::max();
}
void Camera::rotate(float angle_x, float angle_y) {
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
        constexpr float move_step = 0.1f;
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
glm::vec3 Camera::world_to_ndc(const glm::vec3& vertex, double aspect_ratio) const noexcept {
    // World to view space
    glm::vec3 view_vector = vertex - position_;
    glm::mat3 view_rotation = glm::transpose(orientation_);
    glm::vec3 view_space = view_rotation * view_vector;
    
    // Check if vertex is in front of camera and within near/far planes
    if (view_space.z < NearPlane || view_space.z > FarPlane) {
        // Vertex is clipped
        return glm::vec3(0.0f, 0.0f, -1.0f);  // Use negative z to indicate clipped
    }
    
    // Perspective projection to NDC
    double fov_rad = glm::radians(FOV);
    double tan_half_fov = std::tan(fov_rad / 2.0);
    
    // Normalize z to [0, 1] range using perspective-correct depth
    // z_ndc = (far * (z - near)) / (z * (far - near))
    // This ensures: z=near -> z_ndc=0, z=far -> z_ndc=1, and preserves perspective-correct interpolation
    double z_ndc = (FarPlane * (view_space.z - NearPlane)) / (view_space.z * (FarPlane - NearPlane));
    
    return glm::vec3(
        view_space.x / (view_space.z * tan_half_fov * aspect_ratio),
        view_space.y / (view_space.z * tan_half_fov),
        z_ndc
    );
}

Object::Object(const std::string& name) : name_(name), colour_{255, 255, 255} {}
void Object::set_colour(const Colour& colour) {
    colour_ = colour;
}
void Object::add_face(const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3) {
    faces_.push_back({{v1, v2, v3}, colour_});
}

Renderer::Renderer(DrawingWindow& window) noexcept
    : window_(window), z_buffer_(window.width * window.height, std::numeric_limits<float>::infinity()) {}
void Renderer::clear() noexcept {
    z_buffer_.assign(window_.width * window_.height, std::numeric_limits<float>::infinity());
}
ScreenNdcCoord Renderer::ndc_to_screen(const glm::vec3& ndc) const noexcept {
    return ScreenNdcCoord{
        (ndc.x + 1.0f) * 0.5f * window_.width,
        (1.0f - ndc.y) * 0.5f * window_.height,
        ndc.z
    };
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
    std::array<ScreenNdcCoord, 3> screen;
    for (size_t i = 0; i < 3; i++) {
        glm::vec3 ndc = camera.world_to_ndc(face.vertices[i], aspect_ratio_);
        screen[i] = ndc_to_screen(ndc);
    }
    
    for (size_t i = 0; i < 3; i++) {
        ScreenNdcCoord from = screen[i];
        ScreenNdcCoord to = screen[(i + 1) % 3];
        
        if (std::abs(to.x - from.x) >= std::abs(to.y - from.y)) {
            std::size_t from_x = std::max<std::size_t>(static_cast<std::size_t>(std::min(from.x, to.x)), 0);
            std::size_t to_x = std::min(static_cast<std::size_t>(std::max(from.x, to.x)), window_.width - 1);
            for (std::size_t x = from_x; x <= to_x; x++) {
                std::int64_t y = static_cast<std::int64_t>(std::round(from.y + (to.y - from.y) * (x - from.x) / (to.x - from.x)));
                if (x >= window_.width || y < 0 || y >= static_cast<std::int64_t>(window_.height)) {
                    continue;
                }
                float progress = (to.x == from.x) ? 0.0f : static_cast<float>(x - from.x) / static_cast<float>(to.x - from.x);
                float z_ndc = ComputeZndc(progress, std::array<float, 2>{from.z_ndc, to.z_ndc});
                float& depth = z_buffer_[y * window_.width + x];
                if (z_ndc >= 0.0f && z_ndc <= 1.0f && z_ndc < depth) {
                    depth = z_ndc;
                    window_.setPixelColour(x, y, face.colour);
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
                float progress = (to.y == from.y) ? 0.0f : static_cast<float>(y - from.y) / static_cast<float>(to.y - from.y);
                float z_ndc = ComputeZndc(progress, std::array<float, 2>{from.z_ndc, to.z_ndc});
                float& depth = z_buffer_[y * window_.width + x];
                if (z_ndc >= 0.0f && z_ndc <= 1.0f && z_ndc < depth) {
                    depth = z_ndc;
                    window_.setPixelColour(x, y, face.colour);
                }
            }
        }
    }
}

void Renderer::rasterized_render(const Camera& camera, const Face& face) noexcept {
    std::array<ScreenNdcCoord, 3> screen;
    for (size_t i = 0; i < 3; i++) {
        glm::vec3 ndc = camera.world_to_ndc(face.vertices[i], aspect_ratio_);
        screen[i] = ndc_to_screen(ndc);
    }
    
    std::uint32_t colour = face.colour;
    
    ScreenNdcCoord v0 = screen[0];
    ScreenNdcCoord v1 = screen[1];
    ScreenNdcCoord v2 = screen[2];
    
    if (v0.y > v1.y) std::swap(v0, v1);
    if (v0.y > v2.y) std::swap(v0, v2);
    if (v1.y > v2.y) std::swap(v1, v2);

    float from_y = std::max<float>(v0.y, 0);
    float mid_y = std::min<float>(v1.y, window_.height - 1);
    float to_y = std::min<float>(v2.y, window_.height - 1);

    float inv_slope_v0v1 = (v1.y - v0.y) == 0 ? 0 : (v1.x - v0.x) / (v1.y - v0.y);
    float inv_slope_v0v2 = (v2.y - v0.y) == 0 ? 0 : (v2.x - v0.x) / (v2.y - v0.y);
    float inv_slope_v1v2 = (v2.y - v1.y) == 0 ? 0 : (v2.x - v1.x) / (v2.y - v1.y);

    for (float y = from_y; y < mid_y; y++) {
        float x01 = inv_slope_v0v1 * (y - v0.y) + v0.x;
        float x02 = inv_slope_v0v2 * (y - v0.y) + v0.x;

        std::size_t start_x = std::max<std::int64_t>(std::round(std::min(x01, x02)) - 1, 0);
        std::size_t end_x = std::min<std::size_t>(std::round(std::max(x01, x02)), window_.width - 1);
        for (std::size_t x = start_x; x <= end_x; x++) {
            glm::vec3 bary = convertToBarycentricCoordinates(
                { v0.x, v0.y }, { v1.x, v1.y }, { v2.x, v2.y }, { static_cast<float>(x), static_cast<float>(y) });
            float z_ndc = ComputeZndc(std::array<float, 3>{bary.z, bary.x, bary.y}, std::array<float, 3>{ v0.z_ndc, v1.z_ndc, v2.z_ndc });
            std::int64_t yi = static_cast<std::int64_t>(y);
            if (x < window_.width && yi >= 0 && yi < static_cast<std::int64_t>(window_.height)) {
                float& depth = z_buffer_[yi * window_.width + x];
                if (z_ndc >= 0.0f && z_ndc <= 1.0f && z_ndc < depth) {
                    depth = z_ndc;
                    window_.setPixelColour(x, yi, colour);
                }
            }
        }
    }

    for (float y = mid_y; y < to_y; y++) {
        float x12 = inv_slope_v1v2 * (y - v1.y) + v1.x;
        float x02 = inv_slope_v0v2 * (y - v0.y) + v0.x;

        std::size_t start_x = std::max<std::int64_t>(std::round(std::min(x12, x02)) - 1, 0);
        std::size_t end_x = std::min<std::size_t>(std::round(std::max(x12, x02)), window_.width - 1);
        for (std::size_t x = start_x; x <= end_x; x++) {
            glm::vec3 bary = convertToBarycentricCoordinates(
                { v0.x, v0.y }, { v1.x, v1.y }, { v2.x, v2.y }, { static_cast<float>(x), static_cast<float>(y) });
            float z_ndc = ComputeZndc(std::array<float, 3>{bary.z, bary.x, bary.y}, std::array<float, 3>{ v0.z_ndc, v1.z_ndc, v2.z_ndc });
            std::int64_t yi = static_cast<std::int64_t>(y);
            if (x < window_.width && yi >= 0 && yi < static_cast<std::int64_t>(window_.height)) {
                float& depth = z_buffer_[yi * window_.width + x];
                if (z_ndc >= 0.0f && z_ndc <= 1.0f && z_ndc < depth) {
                    depth = z_ndc;
                    window_.setPixelColour(x, yi, colour);
                }
            }
        }
    }
}

void Renderer::raytraced_render(const Camera& camera, const Face& face) noexcept {
    // Raytracing not implemented yet
}

void World::load_file(std::string filename) {
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
            std::string material_filename = filename.substr(0, filename.find_last_of("/\\") + 1) + relative_path;
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
            float x, y, z;
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
void World::draw(Renderer& renderer) const noexcept {
    // camera_.orbiting();
    renderer.clear();
    for (const auto& object : objects_) {
        for (const auto& face : object.faces_) {
            renderer.render(camera_, face);
        }
    }
}
void World::handle_event(const SDL_Event& event) noexcept {
    camera_.handle_event(event);
}
void World::load_materials(std::string filename) {
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
            float r, g, b;
            iss >> r >> g >> b;
            current_material->second = Colour{
                static_cast<std::uint8_t>(Clamp(r * 255.0f, 0.0f, 255.0f)),
                static_cast<std::uint8_t>(Clamp(g * 255.0f, 0.0f, 255.0f)),
                static_cast<std::uint8_t>(Clamp(b * 255.0f, 0.0f, 255.0f))
            };
        }
    }
}
