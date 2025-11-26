#pragma once
#include <fstream>
#include <vector>
#include <sstream>
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
#include <SDL.h>
#include "utils.hpp"

// Clipping planes for frustum
enum class ClipPlane {
    Left,   // x >= -w
    Right,  // x <= w
    Bottom, // y >= -w
    Top,    // y <= w
    Near,   // z >= 0
    Far     // z <= w
};

struct Colour {
    std::uint8_t red;
    std::uint8_t green;
    std::uint8_t blue;
    constexpr operator std::uint32_t () const {
        return (255 << 24) + (red << 16) + (green << 8) + blue;
    }
};

// HDR color with floating point precision
struct ColourHDR {
    FloatType red;
    FloatType green;
    FloatType blue;
    
    constexpr ColourHDR() noexcept : red(0.0f), green(0.0f), blue(0.0f) {}
    constexpr ColourHDR(FloatType r, FloatType g, FloatType b) noexcept : red(r), green(g), blue(b) {}
    
    // Convert from 8-bit sRGB to linear HDR
    static ColourHDR from_srgb(const Colour& srgb, FloatType gamma) noexcept {
        auto to_linear = [gamma](std::uint8_t component) -> FloatType {
            FloatType normalized = component / 255.0f;
            if (gamma == 1.0f) return normalized;  // No gamma correction
            return std::pow(normalized, gamma);
        };
        return ColourHDR(
            to_linear(srgb.red),
            to_linear(srgb.green),
            to_linear(srgb.blue)
        );
    }
    
    // Multiply by scalar (for lighting)
    constexpr ColourHDR operator*(FloatType scalar) const noexcept {
        return ColourHDR(red * scalar, green * scalar, blue * scalar);
    }
    
    // Component-wise multiplication (for color filtering)
    constexpr ColourHDR operator*(const ColourHDR& other) const noexcept {
        return ColourHDR(red * other.red, green * other.green, blue * other.blue);
    }
    
    // Add colors (for ambient + diffuse)
    constexpr ColourHDR operator+(const ColourHDR& other) const noexcept {
        return ColourHDR(red + other.red, green + other.green, blue + other.blue);
    }
};

// Environment map for HDR lighting
class EnvironmentMap {
private:
    std::size_t width_;
    std::size_t height_;
    std::vector<ColourHDR> data_;  // HDR pixel data
    FloatType intensity_;  // Scaling factor for environment map brightness
public:
    EnvironmentMap() noexcept : width_(0), height_(0), intensity_(1.0f) {}
    EnvironmentMap(std::size_t w, std::size_t h, std::vector<ColourHDR> data, FloatType intensity = 0.3f) noexcept
        : width_(w), height_(h), data_(std::move(data)), intensity_(intensity) {}
    
    bool is_loaded() const noexcept { return width_ > 0 && height_ > 0; }
    
    // Compute automatic exposure intensity based on HDR image luminance histogram
    static FloatType compute_auto_exposure(const std::vector<ColourHDR>& hdr_data) noexcept;
    
    // Sample environment map using spherical coordinates from direction vector
    ColourHDR sample(const glm::vec3& direction) const noexcept {
        if (!is_loaded()) return ColourHDR(0.0f, 0.0f, 0.0f);
        
        // Convert direction to spherical coordinates (latitude-longitude)
        // Assumes equirectangular projection
        FloatType theta = std::atan2(direction.x, -direction.z);  // Azimuth angle
        FloatType phi = std::asin(std::clamp(direction.y, -1.0f, 1.0f));  // Elevation angle
        
        // Map to UV coordinates [0, 1]
        FloatType u = (theta / (2.0f * std::numbers::pi)) + 0.5f;
        FloatType v = (phi / std::numbers::pi) + 0.5f;
        
        // Convert to pixel coordinates
        std::size_t x = static_cast<std::size_t>(std::clamp(u * static_cast<FloatType>(width_), 0.0f, static_cast<FloatType>(width_ - 1)));
        std::size_t y = static_cast<std::size_t>(std::clamp(v * static_cast<FloatType>(height_), 0.0f, static_cast<FloatType>(height_ - 1)));
        
        // Apply intensity scaling to reduce brightness
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
        // Convert UV (0-1) to pixel coordinates
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
    enum class Shading { Flat, Gouraud, Phong } shading = Shading::Flat;
    FloatType ior = 1.0f;           // Index of refraction (1.0 = no refraction, 1.5 = glass)
    FloatType td = 1.0f;            // Transmission Depth: The distance at which the base_color is reached
    FloatType tw = 0.0f;            // Transparency weight (0 = opaque, 1 = fully transparent)
    glm::vec3 sigma_a = glm::vec3(0.0f, 0.0f, 0.0f);  // Absorption coefficient for Beer-Lambert (optional override)
};

// Vertex in clip space with attributes
struct ClipVertex {
    glm::vec4 position_clip;  // Homogeneous clip space coordinates
    Colour colour;
    glm::vec2 uv;  // Texture coordinates
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
    glm::vec3 orbit_target_ = { 0.0f, 0.0f, 0.0f };
    bool is_orbiting_ = false;
    FloatType orbit_radius_ = 0.0f;
    FloatType mouse_sensitivity_ = 0.002f;
    bool is_dragging_ = false;
    bool first_drag_motion_ = false;  // Flag to skip first motion event after drag starts
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
    void move(FloatType forward_delta, FloatType right_delta, FloatType up_delta, FloatType dt) noexcept;
    void update_movement() noexcept;  // Update camera position based on keyboard state
    void update() noexcept;  // Legacy update function (calls update_movement)
    std::pair<glm::vec3, glm::vec3> generate_ray(int pixel_x, int pixel_y, int screen_width, int screen_height, double aspect_ratio) const noexcept;
};

struct Face {
    std::array<glm::vec3, 3> vertices;
    std::array<std::uint8_t, 3> texture_vertices;
    std::array<glm::vec2, 3> texture_coords;  // Actual UV coordinates for this face
    Material material;
    std::array<int, 3> vertex_indices;  // Indices into model's vertices_
    std::array<glm::vec3, 3> vertex_normals; // Smoothed vertex normals
    glm::vec3 face_normal;  // Geometric face normal (for flat shading)
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
    bool has_light_ = false;
    glm::vec3 light_position_ = glm::vec3(0.0f, 0.0f, 0.0f);
public:
    Model() noexcept = default;
    void load_file(std::string filename);
    // Getter for all_faces_
    const std::vector<Face>& all_faces() const noexcept { return all_faces_; }
    // Light accessors
    bool has_light() const noexcept { return has_light_; }
    const glm::vec3& light_position() const noexcept { return light_position_; }
private:
    void load_materials(std::string filename);
    Texture load_texture(std::string filename);
    void cache_faces() noexcept;  // Build all_faces_ from objects_
};

class World {
private:
    std::vector<Model> models_;
    std::vector<Face> all_faces_;  // Cached flattened faces from all models
    const glm::vec3 light_position_;  // Immutable after loading
    const bool has_light_;
    FloatType light_intensity_ = 100.0f;  // Adjustable light intensity constant
    static constexpr FloatType light_radius_ = 0.05f;  // Sphere light radius for soft shadows
    EnvironmentMap env_map_;  // HDR environment map
public:
    Camera camera_;  // Now public for direct access
    
    World();
    void load_files(const std::vector<std::string>& filenames);
    void update() noexcept;  // Update world state (camera movement, etc.)
    void orbiting() noexcept;
    // Accessors for Renderer
    Camera& camera() noexcept { return camera_; }
    const Camera& camera() const noexcept { return camera_; }
    const std::vector<Model>& models() const noexcept { return models_; }
    const std::vector<Face>& all_faces() const noexcept { return all_faces_; }
    const glm::vec3& light_position() const noexcept { return light_position_; }
    bool has_light() const noexcept { return has_light_; }
    FloatType light_intensity() const noexcept { return light_intensity_; }
    FloatType light_radius() const noexcept { return light_radius_; }
    void set_light_intensity(FloatType intensity) noexcept { light_intensity_ = intensity; }
    const EnvironmentMap& env_map() const noexcept { return env_map_; }
};
