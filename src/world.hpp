#pragma once
#include <algorithm>
#include <array>
#include <atomic>
#include <barrier>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <numbers>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include "utils.hpp"

/**
 * @brief Frustum clip planes in homogeneous clip space.
 */
enum class ClipPlane {
    LEFT,    // x >= -w
    RIGHT,   // x <= w
    BOTTOM,  // y >= -w
    TOP,     // y <= w
    NEAR,    // z >= 0
    FAR      // z <= w
};

struct Colour {
    std::uint8_t red;
    std::uint8_t green;
    std::uint8_t blue;
    constexpr operator std::uint32_t() const {
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
        return ColourHDR{.red = red * scalar, .green = green * scalar, .blue = blue * scalar};
    }

    constexpr ColourHDR operator*(const ColourHDR& other) const noexcept {
        return ColourHDR{
            .red = red * other.red, .green = green * other.green, .blue = blue * other.blue
        };
    }

    constexpr ColourHDR operator+(const ColourHDR& other) const noexcept {
        return ColourHDR{
            .red = red + other.red, .green = green + other.green, .blue = blue + other.blue
        };
    }
};

struct Face;
struct RayTriangleIntersection;
class BVHAccelerator;  // forward declaration

/**
 * @brief Lat-long HDR environment map with auto-exposure.
 */
class EnvironmentMap {
public: // Static Methods & Constants
    /**
     * @brief Computes a robust exposure scalar from HDR luminance statistics.
     * @param hdr_data Linear HDR pixels.
     * @return Exposure multiplier.
     */
    [[nodiscard]] static FloatType ComputeAutoExposure(const std::vector<ColourHDR>& hdr_data
    ) noexcept;

private: // Data
    std::size_t width_;
    std::size_t height_;
    std::vector<ColourHDR> data_;
    FloatType intensity_;

public: // Lifecycle
    /** @brief Constructs an empty environment. */
    EnvironmentMap() noexcept;
    /**
     * @brief Constructs a loaded environment map.
     * @param w Width in pixels.
     * @param h Height in pixels.
     * @param data Linear HDR texels (lat-long).
     * @param intensity Exposure multiplier.
     */
    EnvironmentMap(
        std::size_t w, std::size_t h, std::vector<ColourHDR> data, FloatType intensity = 0.3f
    ) noexcept;

public: // Accessors & Data Binding
    /** @brief Indicates whether a valid map is present. */
    [[nodiscard]] constexpr bool is_loaded() const noexcept { return width_ > 0 && height_ > 0; }

public: // Core Operations
    /**
     * @brief Samples environment radiance along a direction.
     * @param direction World-space direction.
     * @return Linear HDR radiance.
     */
    [[nodiscard]] ColourHDR sample(const glm::vec3& direction) const noexcept;
};

struct RayTriangleIntersection {
    glm::vec3 intersectionPoint;
    FloatType distanceFromCamera;
    glm::vec3 color;  // Linear RGB color (0-1 range)
    std::size_t triangleIndex;
    glm::vec3 normal;
    glm::vec3 geom_normal;
    bool front_face;
    FloatType u;  // Barycentric coordinate for vertex 1
    FloatType v;  // Barycentric coordinate for vertex 2
};

class Texture {
private: // Data
    std::size_t width_;
    std::size_t height_;
    std::vector<Colour> data_;

public: // Lifecycle
    constexpr Texture(std::size_t w, std::size_t h, std::vector<Colour> data)
        : width_(w), height_(h), data_(std::move(data)) {}

public: // Core Operations
    [[nodiscard]] constexpr Colour sample(FloatType u, FloatType v) const {
        std::size_t ix = static_cast<std::size_t>(std::clamp(
            u * static_cast<FloatType>(width_ - 1), 0.0f, static_cast<FloatType>(width_ - 1)
        ));
        std::size_t iy = static_cast<std::size_t>(std::clamp(
            v * static_cast<FloatType>(height_ - 1), 0.0f, static_cast<FloatType>(height_ - 1)
        ));
        return data_[iy * width_ + ix];
    }
};

struct Material {
    std::shared_ptr<Texture> texture;
    glm::vec3 base_color =
        glm::vec3(1.0f, 1.0f, 1.0f);  // Color multiplier for texture (or base color if no texture)
    FloatType shininess = 64.0f;
    FloatType metallic = 0.0f;
    enum class Shading { FLAT, GOURAUD, PHONG } shading = Shading::FLAT;
    FloatType ior = 1.0f;  // Index of refraction (1.0 = no refraction, 1.5 = glass)
    FloatType td = 1.0f;   // Transmission Depth: The distance at which the base_color is reached
    FloatType tw = 0.0f;   // Transparency weight (0 = opaque, 1 = fully transparent)
    glm::vec3 sigma_a =
        glm::vec3(0.0f, 0.0f, 0.0f);  // Absorption coefficient for Beer-Lambert (optional override)
    glm::vec3 emission = glm::vec3(0.0f);  // Emissive radiance (area light)
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
    glm::vec2 uv;     // Texture coordinates (already divided by w)
    FloatType inv_w;  // 1/w for perspective correction
};

/**
 * @brief Pinhole camera with yaw/pitch/roll and orbit helper.
 */
class Camera {
public: // Static Methods & Constants
    static constexpr auto OrbitInterval = std::chrono::seconds(1) / 60;
    static constexpr double FOV = 45.0;
    static constexpr double NearPlane = 0.001;
    static constexpr double FarPlane = 100.0;

private: // Data
    glm::vec3 position_ = {0.0f, 0.0f, 12.0f};
    FloatType yaw_ = 0.0f;    // Horizontal rotation (around world Y axis)
    FloatType pitch_ = 0.0f;  // Vertical rotation (clamped to ±89 degrees)
    FloatType roll_ = 0.0f;
    glm::vec3 orbit_target_ = {0.0f, 0.0f, 0.0f};
    bool is_orbiting_ = false;
    FloatType orbit_radius_ = 0.0f;

    friend class World;

public: // Lifecycle
    Camera() noexcept = default;

public: // Accessors & Data Binding
    /** @brief Returns camera basis matrix (right, up, forward). */
    [[nodiscard]] glm::mat3 orientation() const noexcept;
    /** @brief Forward direction. */
    [[nodiscard]] glm::vec3 forward() const noexcept;
    /** @brief Right direction. */
    [[nodiscard]] glm::vec3 right() const noexcept;
    /** @brief Up direction. */
    [[nodiscard]] glm::vec3 up() const noexcept;

    /** @brief Sets position. */
    void set_position(const glm::vec3& pos) noexcept;
    [[nodiscard]] glm::vec3 position() const noexcept { return position_; }

    /** @brief Sets yaw. */
    void set_yaw(FloatType y) noexcept;
    [[nodiscard]] FloatType yaw() const noexcept { return yaw_; }

    /** @brief Sets pitch. */
    void set_pitch(FloatType p) noexcept;
    [[nodiscard]] FloatType pitch() const noexcept { return pitch_; }

    /** @brief Sets roll. */
    void set_roll(FloatType r) noexcept;
    [[nodiscard]] FloatType roll() const noexcept { return roll_; }

    [[nodiscard]] bool is_orbiting() const noexcept { return is_orbiting_; }
    [[nodiscard]] glm::vec3 orbit_target() const noexcept { return orbit_target_; }

public: // Core Operations
    /** @brief Projects a world-space vertex to homogeneous clip space. */
    [[nodiscard]] glm::vec4 world_to_clip(const glm::vec3& vertex, double aspect_ratio)
        const noexcept;
    /** @brief Converts clip coordinates to NDC by dividing by w. */
    [[nodiscard]] glm::vec3 clip_to_ndc(const glm::vec4& clip) const noexcept;

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
    void move(
        FloatType forward_delta, FloatType right_delta, FloatType up_delta, FloatType dt
    ) noexcept;

    /** @brief Generates a ray through a pixel center. */
    [[nodiscard]] std::pair<glm::vec3, glm::vec3> generate_ray(
        int pixel_x, int pixel_y, int screen_width, int screen_height, double aspect_ratio
    ) const noexcept;
    /** @brief Generates a ray from normalized UV. */
    [[nodiscard]] std::pair<glm::vec3, glm::vec3> generate_ray_uv(
        FloatType u, FloatType v, int screen_width, int screen_height, double aspect_ratio
    ) const noexcept;
};

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
private: // Data
    std::vector<Object> objects_;
    std::vector<Face> all_faces_;  // Flattened faces for efficient iteration (cached)
    std::map<std::string, Material> materials_;
    std::vector<glm::vec3> vertices_;
    std::vector<glm::vec2> texture_coords_;  // vt coordinates from OBJ
    std::vector<glm::vec3> vertex_normals_;  // vn normals from OBJ
    std::vector<glm::vec3> vertex_normals_by_vertex_;

public: // Lifecycle
    Model() noexcept = default;

public: // Accessors & Data Binding
    /** @brief Number of objects in the model. */
    [[nodiscard]] std::size_t object_count() const noexcept { return objects_.size(); }
    /** @brief Number of materials in the model. */
    [[nodiscard]] std::size_t material_count() const noexcept { return materials_.size(); }
    /** @brief Flattened faces for iteration. */
    [[nodiscard]] const std::vector<Face>& all_faces() const noexcept { return all_faces_; }
    /** @brief Shared vertex positions. */
    [[nodiscard]] const std::vector<glm::vec3>& vertices() const noexcept { return vertices_; }
    /** @brief Shared texture coordinates. */
    [[nodiscard]] const std::vector<glm::vec2>& texture_coords() const noexcept {
        return texture_coords_;
    }
    /** @brief Shared OBJ vertex normals. */
    [[nodiscard]] const std::vector<glm::vec3>& vertex_normals() const noexcept {
        return vertex_normals_;
    }
    /** @brief Optional per-vertex normals authored inline. */
    [[nodiscard]] const std::vector<glm::vec3>& vertex_normals_by_vertex() const noexcept {
        return vertex_normals_by_vertex_;
    }

public: // Core Operations
    /** @brief Loads an OBJ file into this model. */
    void load_file(std::string filename);
    /** @brief Loads a text scene file into this model. */
    void load_scene_txt(std::string filename);

    friend class SceneLoader;

private: // Core Operations (Internal)
    void load_materials(std::string filename);
    Texture load_texture(std::string filename);
    void cache_faces() noexcept;
    void compute_face_normals() noexcept;
};

/**
 * @brief SAH-based BVH over triangle indices for fast intersection and shadows.
 */
class BVHAccelerator {
public: // Types
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

public: // Static Methods & Constants
    /** @brief Ray–AABB test used by traversal. */
    [[nodiscard]] static bool IntersectAABB(
        const glm::vec3& ro, const glm::vec3& rd, const AABB& box, FloatType tmax
    ) noexcept;

private: // Data
    std::vector<int> tri_indices_;
    std::vector<BVHNode> nodes_;

    const std::vector<glm::vec3>* vertices_ = nullptr;
    const std::vector<glm::vec2>* texcoords_ = nullptr;
    const std::vector<glm::vec3>* normals_ = nullptr;
    const std::vector<glm::vec3>* normals_by_vertex_ = nullptr;

public: // Lifecycle
    BVHAccelerator() noexcept = default;
    ~BVHAccelerator() noexcept = default;

public: // Accessors & Data Binding
    [[nodiscard]] bool empty() const noexcept { return nodes_.empty(); }

    void set_vertices(const std::vector<glm::vec3>& verts) noexcept;
    void set_texcoords(const std::vector<glm::vec2>& uvs) noexcept;
    void set_normals(const std::vector<glm::vec3>& norms) noexcept;
    void set_normals_by_vertex(const std::vector<glm::vec3>& norms) noexcept;

public: // Core Operations
    /** @brief Builds BVH nodes over provided faces. */
    void build(const std::vector<Face>& faces) noexcept;

    /** @brief Intersects ray with triangles; returns closest hit or miss. */
    [[nodiscard]] RayTriangleIntersection intersect(
        const glm::vec3& ro, const glm::vec3& rd, const std::vector<Face>& faces
    ) const noexcept;

    /** @brief Computes shadow transmittance along a segment to the light. */
    [[nodiscard]] glm::vec3 transmittance(
        const glm::vec3& point, const glm::vec3& light_pos, const std::vector<Face>& faces
    ) const noexcept;
};

/**
 * @brief Scene container; merges models, builds BVH, and exposes accessors.
 */
class World {
private: // Data
    std::vector<Model> models_;
    std::vector<Face> all_faces_;  // Cached flattened faces from all models
    std::vector<glm::vec3> all_vertices_;
    std::vector<glm::vec2> all_texcoords_;
    std::vector<glm::vec3> all_vertex_normals_;
    std::vector<glm::vec3> all_vertex_normals_by_vertex_;
    EnvironmentMap env_map_;  // HDR environment map
    std::vector<const Face*> emissive_faces_;
    BVHAccelerator accelerator_;
    Camera camera_;

public: // Lifecycle
    /** @brief Default constructor. */
    World() = default;

public: // Accessors & Data Binding
    Camera& camera() noexcept { return camera_; }
    const Camera& camera() const noexcept { return camera_; }

    // Accessors for Renderer
    [[nodiscard]] const std::vector<Model>& models() const noexcept { return models_; }
    [[nodiscard]] const std::vector<Face>& all_faces() const noexcept { return all_faces_; }
    [[nodiscard]] const std::vector<glm::vec3>& all_vertices() const noexcept {
        return all_vertices_;
    }
    [[nodiscard]] const std::vector<glm::vec2>& all_texcoords() const noexcept {
        return all_texcoords_;
    }
    [[nodiscard]] const std::vector<glm::vec3>& all_vertex_normals() const noexcept {
        return all_vertex_normals_;
    }
    [[nodiscard]] const std::vector<glm::vec3>& all_vertex_normals_by_vertex() const noexcept {
        return all_vertex_normals_by_vertex_;
    }
    /** @brief HDR environment map accessor. */
    [[nodiscard]] const EnvironmentMap& env_map() const noexcept { return env_map_; }
    /** @brief List of emissive faces (area lights). */
    [[nodiscard]] const std::vector<const Face*>& area_lights() const noexcept {
        return emissive_faces_;
    }
    /** @brief BVH accelerator accessor. */
    [[nodiscard]] const BVHAccelerator& accelerator() const noexcept { return accelerator_; }

public: // Core Operations
    /**
     * @brief Loads environment and models from files; builds BVH.
     * @param filenames List of scene asset paths.
     */
    void load_files(const std::vector<std::string>& filenames);
};
