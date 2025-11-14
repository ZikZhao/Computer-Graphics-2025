#include "world.hpp"

float ComputeZndc(glm::vec3 bary, glm::vec3 vertices_z_view) {
    float inv_z = bary.x / vertices_z_view.x + bary.y / vertices_z_view.y + bary.z / vertices_z_view.z;
    return 1.0f / inv_z;
}

float ComputeZndc(float progress, glm::vec2 vertices_z_view) {
    float inv_z = (1.0f - progress) / vertices_z_view.x + progress / vertices_z_view.y;
    return 1.0f / inv_z - 0.01;
}

void StrokeLine(DrawingWindow& window, ZBuffer& z_buffer, glm::vec3 from, glm::vec3 to, std::uint32_t colour) {
    if (std::abs(to.x - from.x) >= std::abs(to.y - from.y)) {
        std::size_t from_x = std::max<std::size_t>(static_cast<std::size_t>(std::min(from.x, to.x)), 0);
        std::size_t to_x = std::min(static_cast<std::size_t>(std::max(from.x, to.x)), window.width - 1);
        for (std::size_t x = from_x; x <= to_x; x++) {
            std::int64_t y = static_cast<std::int64_t>(std::round(from.y + (to.y - from.y) * (x - from.x) / (to.x - from.x)));
            float progress = (to.x == from.x) ? 0.0f : static_cast<float>(x - from.x) / static_cast<float>(to.x - from.x);
            if (z_buffer.replace_if_closer(x, y, ComputeZndc(progress, glm::vec2(from.z, to.z)))) {
                window.setPixelColour(x, y, colour);
            }
        }
    } else {
        std::size_t from_y = std::max<std::size_t>(static_cast<std::size_t>(std::min(from.y, to.y)), 0);
        std::size_t to_y = std::min(static_cast<std::size_t>(std::max(from.y, to.y)), static_cast<std::size_t>(window.height - 1));
        for (std::size_t y = from_y; y <= to_y; y++) {
            std::int64_t x = static_cast<std::int64_t>(std::round(from.x + (to.x - from.x) * (y - from.y) / (to.y - from.y)));
            float progress = (to.y == from.y) ? 0.0f : static_cast<float>(y - from.y) / static_cast<float>(to.y - from.y);
            if (z_buffer.replace_if_closer(x, y, ComputeZndc(progress, glm::vec2(from.z, to.z)))) {
                window.setPixelColour(x, y, colour);
            }
        }
    }
}

void FillTriangle(DrawingWindow& window, ZBuffer& z_buffer, glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, std::uint32_t colour) {
    if (v0.y > v1.y) std::swap(v0, v1);
    if (v0.y > v2.y) std::swap(v0, v2);
    if (v1.y > v2.y) std::swap(v1, v2);

    float from_y = std::max<float>(v0.y, 0);
    float mid_y = std::min<float>(v1.y, window.height - 1);
    float to_y = std::min<float>(v2.y, window.height - 1);

    float inv_slope_v0v1 = (v1.y - v0.y) == 0 ? 0 : (v1.x - v0.x) / (v1.y - v0.y);
    float inv_slope_v0v2 = (v2.y - v0.y) == 0 ? 0 : (v2.x - v0.x) / (v2.y - v0.y);
    float inv_slope_v1v2 = (v2.y - v1.y) == 0 ? 0 : (v2.x - v1.x) / (v2.y - v1.y);

    for (float y = from_y; y < mid_y; y++) {
        float x01 = inv_slope_v0v1 * (y - v0.y) + v0.x;
        float x02 = inv_slope_v0v2 * (y - v0.y) + v0.x;

        std::size_t start_x = std::max<std::size_t>(static_cast<std::size_t>(std::round(std::min(x01, x02))), 0);
        std::size_t end_x = std::min(static_cast<std::size_t>(std::round(std::max(x01, x02))), window.width - 1);
        for (std::size_t x = start_x; x <= end_x; x++) {
            glm::vec3 bary = convertToBarycentricCoordinates(
                { v0.x, v0.y }, { v1.x, v1.y }, { v2.x, v2.y }, { static_cast<float>(x), static_cast<float>(y) });
            float z_ndc = ComputeZndc(glm::vec3(bary.z, bary.x, bary.y), { v0.z, v1.z, v2.z });
            if (z_buffer.replace_if_closer(x, y, z_ndc)) {
                window.setPixelColour(x, y, colour);
            }
        }
    }

    for (float y = mid_y; y < to_y; y++) {
        float x12 = inv_slope_v1v2 * (y - v1.y) + v1.x;
        float x02 = inv_slope_v0v2 * (y - v0.y) + v0.x;

        std::size_t start_x = std::max<std::size_t>(static_cast<std::size_t>(std::round(std::min(x12, x02))), 0);
        std::size_t end_x = std::min(static_cast<std::size_t>(std::round(std::max(x12, x02))), window.width - 1);
        for (std::size_t x = start_x; x <= end_x; x++) {
            glm::vec3 bary = convertToBarycentricCoordinates(
                { v0.x, v0.y }, { v1.x, v1.y }, { v2.x, v2.y }, { static_cast<float>(x), static_cast<float>(y) });
            float z_ndc = ComputeZndc(glm::vec3(bary.z, bary.x, bary.y), { v0.z, v1.z, v2.z });
            if (z_buffer.replace_if_closer(x, y, z_ndc)) {
                window.setPixelColour(x, y, colour);
            }
        }
    }
}

ZBuffer::ZBuffer(size_t width, size_t height)
    : buffer_(width * height, std::numeric_limits<float>::infinity()), width_(width), height_(height) {}
ZBuffer& ZBuffer::reset(size_t width, size_t height) {
    width_ = width;
    height_ = height;
    buffer_.assign(width * height, std::numeric_limits<float>::infinity());
    return *this;
}
bool ZBuffer::replace_if_closer(std::int64_t x, std::int64_t y, float z_ndc) {
    if (x < 0 || x >= static_cast<std::int64_t>(width_) || y < 0 || y >= static_cast<std::int64_t>(height_)) {
        return false;
    }
    float& depth = buffer_[y * width_ + x];
    if (z_ndc > 0 && z_ndc < depth) {
        depth = z_ndc;
        return true;
    }
    return false;
}

void Camera::orbiting() {
    auto now = std::chrono::system_clock::now().time_since_epoch().count();
    if (now - last_orbit_time_ > OrbitInterval) {
        constexpr static float angle_increment = glm::radians(0.5f);
        float cos_angle = std::cos(angle_increment);
        float sin_angle = std::sin(angle_increment);
        glm::vec3 k_cross_pos = glm::cross(up_, position_);
        float k_dot_pos = glm::dot(up_, position_);
        position_ = position_ * cos_angle + 
                   k_cross_pos * sin_angle + 
                   up_ * k_dot_pos * (1.0f - cos_angle);
        forward_ = glm::normalize(orbit_target_ - position_);
        right_ = glm::normalize(glm::cross(forward_, up_));
        last_orbit_time_ = now;
    }
}
void Camera::set_orbit(glm::vec3 target) {
    orbit_target_ = target;
    last_orbit_time_ = std::chrono::system_clock::now().time_since_epoch().count();
}
void Camera::remove_orbit() {
    last_orbit_time_ = std::numeric_limits<std::int64_t>::max();
}
void Camera::rotate(float angle_x, float angle_y) {
    forward_ = glm::normalize(
        forward_ * std::cos(angle_x) + right_ * std::sin(angle_x));
    right_ = glm::normalize(glm::cross(forward_, up_));
    forward_ = glm::normalize(
        forward_ * std::cos(-angle_y) + up_ * std::sin(-angle_y));
    up_ = glm::normalize(glm::cross(right_, forward_));
}
void Camera::handle_event(const SDL_Event& event) {
    if (event.type == SDL_KEYDOWN) {
        constexpr float move_step = 0.1f;
        switch (event.key.keysym.sym) {
        case SDLK_w:
            position_ += up_ * move_step;
            break;
        case SDLK_s:
            position_ -= up_ * move_step;
            break;
        case SDLK_a:
            position_ -= right_ * move_step;
            break;
        case SDLK_d:
            position_ += right_ * move_step;
            break;
        case SDLK_q:
            position_ += forward_ * move_step;
            break;
        case SDLK_e:
            position_ -= forward_ * move_step;
            break;
        case SDLK_UP:
            rotate(0.0f, -glm::radians(1.0f));
            break;
        case SDLK_DOWN:
            rotate(0.0f, glm::radians(1.0f));
            break;
        case SDLK_LEFT:
            rotate(-glm::radians(1.0f), 0.0f);
            break;
        case SDLK_RIGHT:
            rotate(glm::radians(1.0f), 0.0f);
            break;
        case SDLK_o:
            if (last_orbit_time_ == std::numeric_limits<std::int64_t>::max()) {
                last_orbit_time_ = std::chrono::system_clock::now().time_since_epoch().count();
            } else {
                last_orbit_time_ = std::numeric_limits<std::int64_t>::max();
            }
        }
    }
}

void Face::draw(DrawingWindow& window, const Camera& camera, ZBuffer& z_buffer) const {
    auto ndc = to_ndc(window, camera);
    auto screen = to_screen(window, ndc);
    
    std::uint32_t colour_value = (255 << 24) + (colour_.red << 16) + (colour_.green << 8) + colour_.blue;
    FillTriangle(window, z_buffer, screen[0], screen[1], screen[2], colour_value);
    for (size_t i = 0; i < 3; i++) {
        const glm::vec3& v0 = screen[i];
        const glm::vec3& v1 = screen[(i + 1) % 3];
        StrokeLine(window, z_buffer, v0, v1, 0xFFFFFFFFu);
    }
}
std::array<glm::vec3, 3> Face::to_ndc(const DrawingWindow& window, const Camera& camera) const {
    std::array<glm::vec3, 3> transformed;
    for (size_t i = 0; i < 3; i++) {
        glm::vec3 view_vector = vertices_[i] - camera.position_;
        transformed[i].x = glm::dot(view_vector, camera.right_);
        transformed[i].y = glm::dot(view_vector, camera.up_);
        transformed[i].z = glm::dot(view_vector, camera.forward_);
    }
    
    std::array<glm::vec3, 3> projected;
    double fov_rad = glm::radians(camera.fov);
    double tan_half_fov = std::tan(fov_rad / 2.0);
    
    for (size_t i = 0; i < 3; i++) {
        if (transformed[i].z > 0.001) {
            // Standard perspective projection to NDC
            projected[i].x = transformed[i].x / (transformed[i].z * tan_half_fov * (static_cast<double>(window.width) / window.height));
            projected[i].y = transformed[i].y / (transformed[i].z * tan_half_fov);
            projected[i].z = transformed[i].z;
        } else {
            // Handle vertices behind camera
            projected[i].x = 0.0;
            projected[i].y = 0.0;
            projected[i].z = 0.01;
        }
    }
    return projected;
}
std::array<glm::vec3, 3> Face::to_screen(const DrawingWindow& window, const std::array<glm::vec3, 3>& ndc) const {
    std::array<glm::vec3, 3> screen;
    for (size_t i = 0; i < 3; i++) {
        screen[i].x = (ndc[i].x + 1) / 2.0 * window.width;
        screen[i].y = (1 - ndc[i].y) / 2.0 * window.height;
        screen[i].z = ndc[i].z;
    }
    return screen;
}

Object::Object(const std::string& name) : name_(name), colour_(255, 255, 255) {}
void Object::set_colour(const Colour& colour) {
    colour_ = colour;
}
void Object::add_face(const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3) {
    faces_.emplace_back(v1, v2, v3, colour_);
}
void Object::draw(DrawingWindow& window, const Camera& camera, ZBuffer& z_buffer) const {
    for (const auto& face : faces_) {
        face.draw(window, camera, z_buffer);
    }
}

void World::LoadFromFile(const std::string& filename) {
    decltype(objects_)::iterator current_obj = objects_.end();
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;
        if (type == "o") {
            std::string name;
            iss >> name;
            objects_.emplace_back(name);
            current_obj = std::prev(objects_.end());
        } else if (type == "usemtl") {
            assert(current_obj != objects_.end());
            std::string colour_name;
            iss >> colour_name;
            Colour colour;
            if (colour_name == "Red") colour = Colour(255, 0, 0);
            else if (colour_name == "Green") colour = Colour(0, 255, 0);
            else if (colour_name == "Blue") colour = Colour(0, 0, 255);
            else if (colour_name == "Yellow") colour = Colour(255, 255, 0);
            else if (colour_name == "Magenta") colour = Colour(255, 0, 255);
            else if (colour_name == "Cyan") colour = Colour(0, 255, 255);
            else if (colour_name == "Grey") colour = Colour(128, 128, 128);
            else colour = Colour(255, 255, 255);
            current_obj->set_colour(colour);
        }
        else if (type == "v") {
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
void World::draw(DrawingWindow& window) {
    camera_.orbiting();
    z_buffer_.reset(window.width, window.height);
    for (const auto& object : objects_) {
        object.draw(window, camera_, z_buffer_);
    }
}
void World::handle_event(const SDL_Event& event, DrawingWindow& window) {
    if (event.type == SDL_KEYDOWN) {
        camera_.handle_event(event);
    } else if (event.type == SDL_MOUSEBUTTONDOWN) {
        window.savePPM("output.ppm");
        window.saveBMP("output.bmp");
    }
}
