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
     * @param gamma Gamma exponent (usually 2.2).
     * @return Linear HDR colour.
     */
    static ColourHDR FromSRGB(const Colour& srgb, FloatType gamma) noexcept;

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
    [[nodiscard]] static FloatType ComputeAutoExposure(const std::vector<ColourHDR>& hdr_data
    ) noexcept;

private:
    std::size_t width_ = 0;
    std::size_t height_ = 0;
    std::vector<ColourHDR> data_;
    FloatType intensity_ = 1.0f;

public:
    EnvironmentMap() = default;

    /**
     * @brief Constructs a loaded environment map.
     * @param width Width in pixels.
     * @param height Height in pixels.
     * @param data Linear HDR texels (lat-long).
     * @param intensity Exposure multiplier.
     */
    EnvironmentMap(
        std::size_t width,
        std::size_t height,
        std::vector<ColourHDR> data,
        FloatType intensity = 0.3f
    ) noexcept;

public:
    /// @brief Indicates whether a valid map is present.
    [[nodiscard]] constexpr bool is_loaded() const noexcept { return width_ > 0 && height_ > 0; }

public:
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

template <typename T>
class ImageBuffer {
private:
    std::size_t width_;
    std::size_t height_;
    std::vector<T> data_;

public:
    ImageBuffer(std::size_t width, std::size_t height, std::vector<T> data)
        : width_(width), height_(height), data_(std::move(data)) {}

    [[nodiscard]] T sample(FloatType u, FloatType v) const noexcept {
        u = u - std::floor(u);
        v = v - std::floor(v);
        std::size_t ix = static_cast<std::size_t>(u * static_cast<FloatType>(width_ - 1));
        std::size_t iy = static_cast<std::size_t>(v * static_cast<FloatType>(height_ - 1));
        if (ix >= width_) ix = width_ - 1;
        if (iy >= height_) iy = height_ - 1;
        return data_[iy * width_ + ix];
    }
};

class Texture : public ImageBuffer<Colour> {
public:
    using ImageBuffer<Colour>::ImageBuffer;
};

class NormalMap : public ImageBuffer<glm::vec3> {
public:
    using ImageBuffer<glm::vec3>::ImageBuffer;
};

struct Material {
    std::shared_ptr<Texture> texture;
    std::shared_ptr<NormalMap> normal_map;  // Tangent-space normal map
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

class Camera {
    friend class World;

public:
    glm::vec3 position_ = {0.0f, 0.0f, 12.0f};
    FloatType yaw_ = 0.0f;    // Horizontal rotation (around world Y axis)
    FloatType pitch_ = 0.0f;  // Vertical rotation (clamped to ±89 degrees)
    FloatType roll_ = 0.0f;
    bool is_orbiting_ = false;
    FloatType orbit_radius_ = 0.0f;
    double aspect_ratio_ = 1.0;

public:
    void set_aspect_ratio(double aspect_ratio) noexcept { aspect_ratio_ = aspect_ratio; }
    [[nodiscard]] double aspect_ratio() const noexcept { return aspect_ratio_; }

public:
    [[nodiscard]] glm::mat3 orientation() const noexcept;
    [[nodiscard]] glm::vec3 forward() const noexcept;
    [[nodiscard]] glm::vec3 right() const noexcept;
    [[nodiscard]] glm::vec3 up() const noexcept;

public:
    /// @brief Projects a world-space vertex to homogeneous clip space.
    [[nodiscard]] glm::vec4 world_to_clip(const glm::vec3& vertex) const noexcept;

    /// @brief Converts clip coordinates to NDC by dividing by w.
    [[nodiscard]] glm::vec3 clip_to_ndc(const glm::vec4& clip) const noexcept;

    void start_orbiting() noexcept;
    void orbiting() noexcept;
    void stop_orbiting() noexcept;

    void rotate(FloatType delta_yaw, FloatType delta_pitch) noexcept;
    void roll(FloatType delta_roll) noexcept;
    void move(
        FloatType forward_delta, FloatType right_delta, FloatType up_delta, FloatType dt
    ) noexcept;

    /// @brief Generates a ray through a pixel center.
    [[nodiscard]] std::pair<glm::vec3, glm::vec3> generate_ray(
        int pixel_x, int pixel_y, int screen_width, int screen_height
    ) const noexcept;

    /// @brief Generates a ray from normalized UV.
    [[nodiscard]] std::pair<glm::vec3, glm::vec3> generate_ray_uv(
        FloatType u, FloatType v, int screen_width, int screen_height
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

struct Model {
    std::vector<Object> objects;
    std::vector<Face> all_faces;
    std::map<std::string, Material> materials;
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec2> texture_coords;
    std::vector<glm::vec3> vertex_normals;
    std::vector<glm::vec3> vertex_normals_by_vertex;
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

public:
    /// @brief Builds a BVH from the given model data.
    [[nodiscard]] static BVHAccelerator Build(
        const std::vector<Face>& faces,
        const std::vector<glm::vec3>& vertices,
        const std::vector<glm::vec3>& normals,
        const std::vector<glm::vec3>& vertex_normals,
        const std::vector<glm::vec2>& texcoords
    );

    /// @brief Ray–AABB test used by traversal.
    [[nodiscard]] static bool IntersectAABB(
        const glm::vec3& ro, const glm::vec3& rd, const AABB& box, FloatType tmax
    ) noexcept;

private:
    std::vector<int> tri_indices_;
    std::vector<BVHNode> nodes_;

    const std::vector<glm::vec3>* vertices_ = nullptr;
    const std::vector<glm::vec2>* texcoords_ = nullptr;
    const std::vector<glm::vec3>* normals_ = nullptr;
    const std::vector<glm::vec3>* normals_by_vertex_ = nullptr;

private:
    BVHAccelerator(
        const std::vector<glm::vec3>& vertices,
        const std::vector<glm::vec3>& normals,
        const std::vector<glm::vec3>& vertex_normals,
        const std::vector<glm::vec2>& texcoords
    );

public:
    BVHAccelerator() = default;

    [[nodiscard]] bool empty() const noexcept { return nodes_.empty(); }

    /**
     * @brief Intersects ray with triangles; returns closest hit or miss.
     * @param ro Ray origin.
     * @param rd Ray direction.
     * @param faces Triangle array.
     * @return Closest intersection or miss.
     */
    [[nodiscard]] RayTriangleIntersection intersect(
        const glm::vec3& ro, const glm::vec3& rd, const std::vector<Face>& faces
    ) const noexcept;

    /**
     * @brief Computes shadow transmittance along a segment to the light.
     * @param point Starting point.
     * @param light_pos Light position.
     * @param faces Triangle array.
     * @return RGB transmittance (1.0 = unoccluded, 0.0 = fully occluded).
     */
    [[nodiscard]] glm::vec3 transmittance(
        const glm::vec3& point, const glm::vec3& light_pos, const std::vector<Face>& faces
    ) const noexcept;
};

/**
 * @brief Scene container; merges models, builds BVH, and exposes accessors.
 */
class World {
public:
    std::vector<Model> models_;
    std::vector<Face> all_faces_;  // Flattened faces from all models
    std::vector<glm::vec3> all_vertices_;
    std::vector<glm::vec2> all_texcoords_;
    std::vector<glm::vec3> all_vertex_normals_;
    std::vector<glm::vec3>
        all_vertex_normals_by_vertex_;  // Vertex normals defined by "Vertex x y z / xn yn zn" lines
    EnvironmentMap env_map_;
    std::vector<const Face*> emissive_faces_;
    BVHAccelerator accelerator_;
    Camera camera_;

public:
    World(const std::vector<std::string>& filenames);

private:
    void parse_obj(Model& model, const std::filesystem::path& path);
    void parse_mtl(Model& model, const std::filesystem::path& path);

    /**
     * @brief Parses a text scene description file.
     * @param model Model to populate.
     * @param filename Path to the text file.
     *
     * @note The text file can reference other files using relative paths.
     */
    void parse_txt(Model& model, const std::filesystem::path& path);

    void load_hdr_env_map(const std::filesystem::path& path);
    Texture load_texture(const std::filesystem::path& path);
    NormalMap load_normal_map(const std::filesystem::path& path);

    void compute_all_face_normals(Model& model);
    void flatten_model_faces(Model& model);

    void merge_models() noexcept;
    void reserve_global_buffers() noexcept;
    void append_model_data(const Model& model) noexcept;
    void collect_emissive_faces() noexcept;
};
