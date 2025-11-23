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
#include "DrawingWindow.h"
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
    Colour colour;
    std::size_t triangleIndex;
    glm::vec3 normal;  // Surface normal at intersection
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
    Colour colour;
    std::shared_ptr<Texture> texture;
    FloatType shininess = 64.0f;  // Specular shininess (higher = sharper highlights)
    FloatType metallic = 0.0f;  // Metallic property (0.0 = non-metallic, 1.0 = fully metallic)
    enum class Shading { Gouraud, Phong } shading = Shading::Phong;
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
    static constexpr std::int64_t OrbitInterval = 1'000'000'000 / 60; // 60 FPS
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
    FloatType mouse_sensitivity_ = 0.001f;
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
    void handle_event(const SDL_Event& event) noexcept;
    std::pair<glm::vec3, glm::vec3> generate_ray(int pixel_x, int pixel_y, int screen_width, int screen_height, double aspect_ratio) const noexcept;
};

struct Face {
    std::array<glm::vec3, 3> vertices;
    std::array<std::uint8_t, 3> texture_vertices;
    std::array<glm::vec2, 3> texture_coords;  // Actual UV coordinates for this face
    Material material;
    std::array<int, 3> vertex_indices;  // Indices into model's vertices_
    std::array<glm::vec3, 3> vertex_normals; // Smoothed vertex normals
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
    Camera camera_;
    const glm::vec3 light_position_;  // Immutable after loading
    const bool has_light_;
    FloatType light_intensity_ = 100.0f;  // Adjustable light intensity constant
    static constexpr FloatType light_radius_ = 0.05f;  // Sphere light radius for soft shadows
    EnvironmentMap env_map_;  // HDR environment map
public:
    World();
    void load_files(const std::vector<std::string>& filenames);
    void handle_event(const SDL_Event& event) noexcept;
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

class Renderer {
public:
    enum Mode {
        Wireframe,
        Rasterized,
        Raytraced,
    };
    Mode mode_ = Raytraced;
    FloatType gamma_ = 2.2f;  // Gamma value: 1.0 = no correction, 2.2 = standard sRGB
    bool soft_shadows_enabled_ = false;  // Toggle for soft shadows (expensive)
private:
    DrawingWindow& window_;
    const World& world_;
    std::vector<FloatType> z_buffer_;
    std::vector<ColourHDR> hdr_buffer_;  // HDR floating point color buffer
    double aspect_ratio_ = 1.0;
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
    static constexpr int TileHeight = 16;
private:
    const std::vector<int> bvh_tri_indices_;  // Immutable after construction
    const std::vector<BVHNode> bvh_nodes_;    // Immutable after construction
    // Worker threads and synchronization
    std::barrier<> frame_barrier_;
    std::vector<std::jthread> workers_;
    std::atomic<int> tile_counter_ = 0;
    // Current frame context for workers (only camera changes per frame)
    const Camera* current_camera_ = nullptr;
public:
    Renderer(DrawingWindow& window, const World& world) noexcept;
    ~Renderer() noexcept = default;
    void clear() noexcept;
    ScreenNdcCoord ndc_to_screen(const glm::vec3& ndc, const glm::vec2& uv, FloatType w) const noexcept;
    void render() noexcept;
    void handle_event(const SDL_Event& event) noexcept;
private:
    // World-based rendering (used by render() dispatch)
    void wireframe_render() noexcept;
    void rasterized_render() noexcept;
    void raytraced_render() noexcept;
    // Per-face rendering (used by Model::draw for wireframe/rasterized)
    void wireframe_render(const Camera& camera, const Face& face) noexcept;
    void rasterized_render(const Camera& camera, const Face& face) noexcept;
    // Ray tracing helpers
    static RayTriangleIntersection find_closest_intersection(const glm::vec3& ray_origin, const glm::vec3& ray_dir, const std::vector<Face>& faces) noexcept;
    RayTriangleIntersection find_closest_intersection_bvh(const glm::vec3& ray_origin, const glm::vec3& ray_dir) const noexcept;
    ColourHDR trace_ray(const glm::vec3& ray_origin, const glm::vec3& ray_dir, int depth) const noexcept;
    static std::pair<std::vector<int>, std::vector<Renderer::BVHNode>> build_bvh(const std::vector<Face>& faces) noexcept;
    static bool intersect_aabb(const glm::vec3& ro, const glm::vec3& rd, const AABB& box, FloatType tmax) noexcept;
    static bool is_in_shadow(const glm::vec3& point, const glm::vec3& light_pos, const std::vector<Face>& faces) noexcept;
    bool is_in_shadow_bvh(const glm::vec3& point, const glm::vec3& light_pos) const noexcept;
    // Soft shadow with stratified sampling on sphere light
    FloatType compute_soft_shadow(const glm::vec3& point, const glm::vec3& light_center, FloatType light_radius, int num_samples) const noexcept;
    static glm::vec3 sample_sphere_halton(int index, FloatType radius, const glm::vec3& center) noexcept;
    static FloatType halton(int index, int base) noexcept;
    static FloatType compute_lambertian_lighting(const glm::vec3& normal, const glm::vec3& to_light, FloatType distance, FloatType intensity) noexcept;
    static FloatType compute_specular_lighting(const glm::vec3& normal, const glm::vec3& to_light, const glm::vec3& to_camera, FloatType distance, FloatType intensity, FloatType shininess) noexcept;
    void process_rows(int y0, int y1) noexcept;
    void worker_thread() noexcept;
    // HDR tonemapping and gamma correction
    static Colour tonemap_and_gamma_correct(const ColourHDR& hdr, FloatType gamma) noexcept;
    static FloatType aces_tonemap(FloatType hdr_value) noexcept;
    // Texture sampling with perspective correction
    static std::uint32_t sample_texture(const Face& face, const glm::vec3& bary, 
                                         const ScreenNdcCoord& v0, const ScreenNdcCoord& v1, const ScreenNdcCoord& v2) noexcept;
    // Clipping utilities
    static bool inside_plane(const glm::vec4& v, ClipPlane plane) noexcept;
    static FloatType compute_intersection_t(const glm::vec4& v0, const glm::vec4& v1, ClipPlane plane) noexcept;
    static ClipVertex intersect_plane(const ClipVertex& v0, const ClipVertex& v1, ClipPlane plane) noexcept;
    static InplaceVector<ClipVertex, 9> clip_against_plane(const InplaceVector<ClipVertex, 9>& input, ClipPlane plane) noexcept;
    InplaceVector<ClipVertex, 9> clip_triangle(const Camera& camera, const Face& face) noexcept;
};
