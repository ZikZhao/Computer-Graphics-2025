#pragma once
#include <fstream>
#include <vector>
#include <sstream>
#include <algorithm>
#include <functional>
#include <numeric>
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
#include "utils.hpp"

// Clipping planes for frustum
/**
 * @brief Frustum clip planes in homogeneous clip space.
 */
enum class ClipPlane {
    LEFT,   // x >= -w
    RIGHT,  // x <= w
    BOTTOM, // y >= -w
    TOP,    // y <= w
    NEAR,   // z >= 0
    FAR     // z <= w
};

struct Colour {
    std::uint8_t red;
    std::uint8_t green;
    std::uint8_t blue;
    constexpr operator std::uint32_t () const {
        return (255 << 24) + (red << 16) + (green << 8) + blue;
    }
};

struct ColourHDR {
    FloatType red;
    FloatType green;
    FloatType blue;
    
    /**
     * @brief Converts 8-bit sRGB to linear HDR using gamma.
     * @param srgb Source colour.
     * @param gamma Gamma exponent (1.0 = identity).
     * @return Linear HDR colour.
     */
    static ColourHDR from_srgb(const Colour& srgb, FloatType gamma) noexcept;
    
    constexpr ColourHDR operator*(FloatType scalar) const noexcept {
        return ColourHDR{ .red = red * scalar, .green = green * scalar, .blue = blue * scalar };
    }
    
    constexpr ColourHDR operator*(const ColourHDR& other) const noexcept {
        return ColourHDR{ .red = red * other.red, .green = green * other.green, .blue = blue * other.blue };
    }
    
    constexpr ColourHDR operator+(const ColourHDR& other) const noexcept {
        return ColourHDR{ .red = red + other.red, .green = green + other.green, .blue = blue + other.blue };
    }
};

struct Face;
struct RayTriangleIntersection;
class BVHAccelerator; // forward declaration

/**
 * @brief Lat-long HDR environment map with auto-exposure.
 */
class EnvironmentMap {
public:
    /**
     * @brief Computes a robust exposure scalar from HDR luminance statistics.
     * @param hdr_data Linear HDR pixels.
     * @return Exposure multiplier.
     */
    static FloatType ComputeAutoExposure(const std::vector<ColourHDR>& hdr_data) noexcept;

private:
    std::size_t width_;
    std::size_t height_;
    std::vector<ColourHDR> data_;
    FloatType intensity_;

public:
    /** @brief Constructs an empty environment. */
    EnvironmentMap() noexcept;
    /**
     * @brief Constructs a loaded environment map.
     * @param w Width in pixels.
     * @param h Height in pixels.
     * @param data Linear HDR texels (lat-long).
     * @param intensity Exposure multiplier.
     */
    EnvironmentMap(std::size_t w, std::size_t h, std::vector<ColourHDR> data, FloatType intensity = 0.3f) noexcept;
    /** @brief Indicates whether a valid map is present. */
    constexpr bool is_loaded() const noexcept { return width_ > 0 && height_ > 0; }
    /**
     * @brief Samples environment radiance along a direction.
     * @param direction World-space direction.
     * @return Linear HDR radiance.
     */
    ColourHDR sample(const glm::vec3& direction) const noexcept;
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
    enum class Shading { FLAT, GOURAUD, PHONG } shading = Shading::FLAT;
    FloatType ior = 1.0f;           // Index of refraction (1.0 = no refraction, 1.5 = glass)
    FloatType td = 1.0f;            // Transmission Depth: The distance at which the base_color is reached
    FloatType tw = 0.0f;            // Transparency weight (0 = opaque, 1 = fully transparent)
    glm::vec3 sigma_a = glm::vec3(0.0f, 0.0f, 0.0f);  // Absorption coefficient for Beer-Lambert (optional override)
    glm::vec3 emission = glm::vec3(0.0f);             // Emissive radiance (area light)
};

struct ClipVertex {
    glm::vec4 position_clip;
    Colour colour;
    glm::vec2 uv;
};

struct ScreenNdcCoord {
    FloatType x;
    FloatType y;
    FloatType z_ndc;
    glm::vec2 uv;  // Texture coordinates (already divided by w)
    FloatType inv_w;  // 1/w for perspective correction
};

/**
 * @brief Pinhole camera with yaw/pitch/roll and orbit helper.
 */
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
    FloatType roll_ = 0.0f;
    glm::vec3 orbit_target_ = { 0.0f, 0.0f, 0.0f };
    bool is_orbiting_ = false;
    FloatType orbit_radius_ = 0.0f;
    Camera() noexcept = default;
    /** @brief Projects a world-space vertex to homogeneous clip space. */
    glm::vec4 world_to_clip(const glm::vec3& vertex, double aspect_ratio) const noexcept;
    /** @brief Converts clip coordinates to NDC by dividing by w. */
    glm::vec3 clip_to_ndc(const glm::vec4& clip) const noexcept;
    /** @brief Returns camera basis matrix (right, up, forward). */
    glm::mat3 orientation() const noexcept;
    /** @brief Forward direction. */
    glm::vec3 forward() const noexcept;
    /** @brief Right direction. */
    glm::vec3 right() const noexcept;
    /** @brief Up direction. */
    glm::vec3 up() const noexcept;
    /** @brief Begins orbiting around a target at current radius. */
    void start_orbiting(glm::vec3 target) noexcept;
    /** @brief Updates orbit if active. */
    void orbiting() noexcept;
    /** @brief Stops orbiting. */
    void stop_orbiting() noexcept;
    /** @brief Applies yaw/pitch deltas with pitch clamp. */
    void rotate(FloatType delta_yaw, FloatType delta_pitch) noexcept;
    /** @brief Applies roll delta. */
    void roll(FloatType delta_roll) noexcept;
    /** @brief Moves in local axes scaled by dt. */
    void move(FloatType forward_delta, FloatType right_delta, FloatType up_delta, FloatType dt) noexcept;
    /** @brief Sets position. */
    void set_position(const glm::vec3& pos) noexcept;
    /** @brief Sets yaw. */
    void set_yaw(FloatType y) noexcept;
    /** @brief Sets pitch. */
    void set_pitch(FloatType p) noexcept;
    /** @brief Sets roll. */
    void set_roll(FloatType r) noexcept;
    /** @brief Generates a ray through a pixel center. */
    std::pair<glm::vec3, glm::vec3> generate_ray(int pixel_x, int pixel_y, int screen_width, int screen_height, double aspect_ratio) const noexcept;
    /** @brief Generates a ray from normalized UV. */
    std::pair<glm::vec3, glm::vec3> generate_ray_uv(FloatType u, FloatType v, int screen_width, int screen_height, double aspect_ratio) const noexcept;
};

// Optimization: Store per-vertex indices instead of duplicating vertex data.
// Reduces Face footprint (e.g., ~100B → ~40B depending on Material/padding),
// which improves cache residency and memory bandwidth during BVH traversal and
// ray–triangle intersection. The triangles stream indices; actual positions,
// uvs and normals are fetched from shared arrays on demand.
struct Face {
    std::array<std::uint32_t, 3> v_indices;
    std::array<std::uint32_t, 3> vt_indices;
    std::array<std::uint32_t, 3> vn_indices;
    Material material;
    glm::vec3 face_normal;
};

struct Object {
    std::string name;
    Material material;
    std::vector<Face> faces;
};

/**
 * @brief Aggregates objects, geometry arrays, and materials for a scene asset.
 */
class Model {
private:
    std::vector<Object> objects_;
    std::vector<Face> all_faces_;  // Flattened faces for efficient iteration (cached)
    std::map<std::string, Material> materials_;
    std::vector<glm::vec3> vertices_;
    std::vector<glm::vec2> texture_coords_;  // vt coordinates from OBJ
    std::vector<glm::vec3> vertex_normals_;  // vn normals from OBJ
    std::vector<glm::vec3> vertex_normals_by_vertex_;
public:
    Model() noexcept = default;
    /** @brief Loads an OBJ file into this model. */
    void load_file(std::string filename);
    /** @brief Loads a text scene file into this model. */
    void load_scene_txt(std::string filename);
    friend class SceneLoader;
    /** @brief Flattened faces for iteration. */
    const std::vector<Face>& all_faces() const noexcept { return all_faces_; }
    /** @brief Shared vertex positions. */
    const std::vector<glm::vec3>& vertices() const noexcept { return vertices_; }
    /** @brief Shared texture coordinates. */
    const std::vector<glm::vec2>& texture_coords() const noexcept { return texture_coords_; }
    /** @brief Shared OBJ vertex normals. */
    const std::vector<glm::vec3>& vertex_normals() const noexcept { return vertex_normals_; }
    /** @brief Optional per-vertex normals authored inline. */
    const std::vector<glm::vec3>& vertex_normals_by_vertex() const noexcept { return vertex_normals_by_vertex_; }
    private:
    void load_materials(std::string filename);
    Texture load_texture(std::string filename);
    void cache_faces() noexcept;
    void compute_face_normals() noexcept;
};

/**
 * @brief SAH-based BVH over triangle indices for fast intersection and shadows.
 */
class BVHAccelerator {
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
private:
    std::vector<int> tri_indices_;
    std::vector<BVHNode> nodes_;
    const std::vector<glm::vec3>* vertices_ = nullptr;
    const std::vector<glm::vec2>* texcoords_ = nullptr;
    const std::vector<glm::vec3>* normals_ = nullptr;
    const std::vector<glm::vec3>* normals_by_vertex_ = nullptr;
public:
    BVHAccelerator() noexcept = default;
    /** @brief Returns true if no nodes are built. */
    bool empty() const noexcept;
    /** @brief Binds vertex array used by intersection. */
    void set_vertices(const std::vector<glm::vec3>& verts) noexcept;
    /** @brief Binds texture coordinate array. */
    void set_texcoords(const std::vector<glm::vec2>& uvs) noexcept;
    /** @brief Binds vertex normals array. */
    void set_normals(const std::vector<glm::vec3>& norms) noexcept;
    /** @brief Binds per-vertex normals array. */
    void set_normals_by_vertex(const std::vector<glm::vec3>& norms) noexcept;
    /** @brief Builds BVH nodes over provided faces. */
    void build(const std::vector<Face>& faces) noexcept;
    /** @brief Ray–AABB test used by traversal. */
    static bool IntersectAABB(const glm::vec3& ro, const glm::vec3& rd, const AABB& box, FloatType tmax) noexcept;
    /** @brief Intersects ray with triangles; returns closest hit or miss. */
    RayTriangleIntersection intersect(const glm::vec3& ro, const glm::vec3& rd, const std::vector<Face>& faces) const noexcept;
    /** @brief Computes shadow transmittance along a segment to the light. */
    glm::vec3 transmittance(const glm::vec3& point, const glm::vec3& light_pos, const std::vector<Face>& faces) const noexcept;
};

/**
 * @brief Scene container; merges models, builds BVH, and exposes accessors.
 */
class World {
private:
    std::vector<Model> models_;
    std::vector<Face> all_faces_;  // Cached flattened faces from all models
    std::vector<glm::vec3> all_vertices_;
    std::vector<glm::vec2> all_texcoords_;
    std::vector<glm::vec3> all_vertex_normals_;
    std::vector<glm::vec3> all_vertex_normals_by_vertex_;
    EnvironmentMap env_map_;  // HDR environment map
    std::vector<const Face*> emissive_faces_;
    BVHAccelerator accelerator_;
public:
    Camera camera_;
    
    /** @brief Constructs an empty world. */
    World();
    /**
     * @brief Loads environment and models from files; builds BVH.
     * @param filenames List of scene asset paths.
     */
    void load_files(const std::vector<std::string>& filenames);
    // Accessors for Renderer
    const std::vector<Model>& models() const noexcept;
    const std::vector<Face>& all_faces() const noexcept;
    const std::vector<glm::vec3>& all_vertices() const noexcept;
    const std::vector<glm::vec2>& all_texcoords() const noexcept;
    const std::vector<glm::vec3>& all_vertex_normals() const noexcept;
    const std::vector<glm::vec3>& all_vertex_normals_by_vertex() const noexcept;
    /** @brief HDR environment map accessor. */
    const EnvironmentMap& env_map() const noexcept;
    /** @brief List of emissive faces (area lights). */
    const std::vector<const Face*>& area_lights() const noexcept;
    /** @brief BVH accelerator accessor. */
    const BVHAccelerator& accelerator() const noexcept;
};
