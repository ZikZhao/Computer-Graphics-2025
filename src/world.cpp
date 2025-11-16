#include "world.hpp"

glm::vec3 Object::compute_centroid(const Object& object) noexcept {
    glm::vec3 sum(0.0f, 0.0f, 0.0f);
    std::size_t vertex_count = 0;
    
    for (const auto& face : object.faces) {
        for (const auto& vertex : face.vertices) {
            sum += vertex;
            vertex_count++;
        }
    }
    
    if (vertex_count == 0) {
        return glm::vec3(0.0f, 0.0f, 0.0f);
    }
    
    return sum / static_cast<FloatType>(vertex_count);
}

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
    glm::vec3 ray_dir_world = orientation_ * ray_dir_view;
    
    return {position_, glm::normalize(ray_dir_world)};
}

Renderer::Renderer(DrawingWindow& window) noexcept
    : window_(window), z_buffer_(window.width * window.height, 0.0f) {}
void Renderer::clear() noexcept {
    z_buffer_.assign(window_.width * window_.height, 0.0f);
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
    FloatType abs_w = std::abs(v.w);
    
    switch (plane) {
        case ClipPlane::Left:   return v.x >= -abs_w;
        case ClipPlane::Right:  return v.x <= abs_w;
        case ClipPlane::Bottom: return v.y >= -abs_w;
        case ClipPlane::Top:    return v.y <= abs_w;
        case ClipPlane::Near:   return v.z >= -abs_w;
        case ClipPlane::Far:    return v.z <= abs_w;
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
        // Raytraced mode is handled by render(World&)
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
std::uint32_t Renderer::sample_texture(const Face& face, const glm::vec3& bary,
                                       const ScreenNdcCoord& v0, const ScreenNdcCoord& v1, const ScreenNdcCoord& v2) noexcept {
    if (!face.material.texture) return face.material.colour;
    
    // Perspective-correct interpolation: interpolate uv/w and 1/w, then divide
    FloatType inv_w = bary.z * v0.inv_w + bary.x * v1.inv_w + bary.y * v2.inv_w;
    FloatType u = (bary.z * v0.uv.x * v0.inv_w + bary.x * v1.uv.x * v1.inv_w + bary.y * v2.uv.x * v2.inv_w) / inv_w;
    FloatType v = (bary.z * v0.uv.y * v0.inv_w + bary.x * v1.uv.y * v1.inv_w + bary.y * v2.uv.y * v2.inv_w) / inv_w;
    
    return face.material.texture->sample(u, v);
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

// Möller-Trumbore ray-triangle intersection algorithm
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
bool Renderer::is_in_shadow(const glm::vec3& point, const glm::vec3& light_pos, const std::vector<Face>& faces,
                            std::size_t light_face_start, std::size_t light_face_end) noexcept {
    glm::vec3 to_light = light_pos - point;
    FloatType light_distance = glm::length(to_light);
    glm::vec3 light_dir = to_light / light_distance;
    
    // Use the intersection point directly as shadow ray origin
    // The epsilon in intersect_ray_triangle will handle avoiding self-intersection
    glm::vec3 shadow_ray_origin = point;
    
    for (std::size_t i = 0; i < faces.size(); i++) {
        // Skip light source faces - light passes through itself
        if (i >= light_face_start && i < light_face_end) {
            continue;
        }
        
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
void Renderer::render_raytraced(const Camera& camera, const std::vector<Face>& all_faces, const glm::vec3& light_pos,
                                std::size_t light_face_start, std::size_t light_face_end) noexcept {
    aspect_ratio_ = static_cast<double>(window_.width) / window_.height;
    
    static bool first_frame = true;
    int hit_count = 0;
    int shadow_count = 0;
    int total_pixels = window_.width * window_.height;
    
    // Iterate over all pixels
    for (int y = 0; y < static_cast<int>(window_.height); y++) {
        for (int x = 0; x < static_cast<int>(window_.width); x++) {
            // Generate ray for this pixel
            auto [ray_origin, ray_dir] = camera.generate_ray(x, y, window_.width, window_.height, aspect_ratio_);
            
            // Debug: print center pixel ray
            if (first_frame && x == window_.width / 2 && y == window_.height / 2) {
                std::cout << "Center ray: origin=(" << ray_origin.x << "," << ray_origin.y << "," << ray_origin.z 
                          << ") dir=(" << ray_dir.x << "," << ray_dir.y << "," << ray_dir.z << ")" << std::endl;
            }
            
            // Find closest intersection
            RayTriangleIntersection intersection = find_closest_intersection(ray_origin, ray_dir, all_faces);
            
            std::uint32_t pixel_colour;
            
            if (intersection.triangleIndex != static_cast<size_t>(-1)) {
                hit_count++;
                // Check if point is in shadow
                if (is_in_shadow(intersection.intersectionPoint, light_pos, all_faces, light_face_start, light_face_end)) {
                    shadow_count++;
                    // In shadow - render pure black
                    pixel_colour = 0xFF000000;
                } else {
                    // Not in shadow - use the color from intersection
                    pixel_colour = intersection.colour;
                }
            } else {
                // No intersection - render black (background)
                pixel_colour = 0xFF000000;
            }
            
            window_.setPixelColour(x, y, pixel_colour);
        }
    }
    
    if (first_frame) {
        std::cout << "First frame stats: " << hit_count << "/" << total_pixels << " pixels hit geometry ("
                  << (100.0 * hit_count / total_pixels) << "%)" << std::endl;
        std::cout << "Shadow pixels: " << shadow_count << "/" << hit_count << " ("
                  << (100.0 * shadow_count / hit_count) << "%)" << std::endl;
        std::cout << "Light position: (" << light_pos.x << "," << light_pos.y << "," << light_pos.z << ")" << std::endl;
        std::cout << "Total faces: " << all_faces.size() << std::endl;
        first_frame = false;
    }
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
            current_obj->material = materials_[colour_name];
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
            for (int i = 0; i < 3; i++) {
                int vertex_index;
                char slash;
                iss >> vertex_index >> slash;
                vertice[i] = vertices_[vertex_index - 1];
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
                    tex_indices[i] = 0;
                    tex_coords[i] = glm::vec2(0.0f, 0.0f);
                }
            }
            current_obj->faces.emplace_back(Face{
                { vertice[0], vertice[1], vertice[2] },
                { tex_indices[0], tex_indices[1], tex_indices[2] },
                { tex_coords[0], tex_coords[1], tex_coords[2] },
                current_obj->material,
            });
        }
    }
}
void Model::draw(Renderer& renderer, const Camera& camera) const noexcept {
    for (const auto& object : objects_) {
        for (const auto& face : object.faces) {
            renderer.render(camera, face);
        }
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

void World::compute_light_position() noexcept {
    // Find first pure white (255, 255, 255) object without texture
    std::size_t face_offset = 0;
    for (const auto& model : models_) {
        for (const auto& object : model.objects_) {
            const Material& mat = object.material;
            if (mat.colour.red == 255 && 
                mat.colour.green == 255 && 
                mat.colour.blue == 255 && 
                mat.texture == nullptr) {
                // Found white non-textured object - use its centroid as light
                light_position_ = Object::compute_centroid(object);
                light_face_start_ = face_offset;
                light_face_end_ = face_offset + object.faces.size();
                std::cout << "Light source found: " << object.name 
                          << " at position (" << light_position_.x << ", " 
                          << light_position_.y << ", " << light_position_.z << ")" << std::endl;
                std::cout << "Light faces: [" << light_face_start_ << ", " << light_face_end_ << ")" << std::endl;
                return;
            }
            face_offset += object.faces.size();
        }
    }
    
    // Fallback: no white object found, use default position
    light_position_ = glm::vec3(0.0f, 2.0f, 0.0f);
    light_face_start_ = 0;
    light_face_end_ = 0;
    std::cout << "No white light source found, using default position (0, 2, 0)" << std::endl;
}
void World::load_files(const std::vector<std::string>& filenames) {
    for (const auto& filename : filenames) {
        Model group;
        group.load_file(filename);
        models_.emplace_back(std::move(group));
    }
    
    // After loading all models, compute light position
    compute_light_position();
}

void Renderer::render(World& world) noexcept {
    clear();
    
    // Check if we're in raytraced mode
    if (mode_ == Raytraced) {
        // Collect all faces from all models
        std::vector<Face> all_faces;
        for (const auto& model : world.models()) {
            for (const auto& object : model.objects_) {
                for (const auto& face : object.faces) {
                    all_faces.push_back(face);
                }
            }
        }
        
        // Render using ray tracing
        render_raytraced(world.camera(), all_faces, world.light_position(), 
                        world.light_face_start(), world.light_face_end());
    } else {
        // Use traditional per-face rendering for wireframe and rasterized modes
        for (const auto& model : world.models()) {
            model.draw(*this, world.camera());
        }
    }
}

void World::handle_event(const SDL_Event& event) noexcept {
    camera_.handle_event(event);
}
void World::orbiting() noexcept {
    camera_.orbiting();
}
