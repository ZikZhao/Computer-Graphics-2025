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
#include "DrawingWindow.h"
#include "Utils.h"

using FloatType = decltype(std::declval<glm::vec3>().x);

template<typename T>
constexpr T Clamp(T value, T min, T max) {
    return (value < min) ? min : (value > max) ? max : value;
}

constexpr FloatType ComputeInvZndc(std::array<FloatType, 3> bary, std::array<FloatType, 3> vertices_z_ndc) {
    return bary[0] / vertices_z_ndc[0] +
           bary[1] / vertices_z_ndc[1] +
           bary[2] / vertices_z_ndc[2];
}

constexpr FloatType ComputeInvZndc(FloatType progress, std::array<FloatType, 2> vertices_z_ndc) {
    return (1.0f - progress) / vertices_z_ndc[0] + progress / vertices_z_ndc[1];
}

template<typename T, std::size_t N>
class InplaceVector {
private:
    T data_[N];
    std::size_t size_ = 0;
public:
    constexpr InplaceVector() noexcept = default;
    constexpr InplaceVector(std::initializer_list<T> init) noexcept : size_(init.size()) {
        assert(size_ <= N);
        std::copy(init.begin(), init.end(), std::begin(data_));
    }
    constexpr void push_back(const T& value) noexcept {
        assert(size_ < N);
        data_[size_++] = value;
    }
    constexpr std::size_t size() const noexcept { return size_; }
    constexpr T& operator[](std::size_t index) noexcept { return data_[index]; }
    constexpr const T& operator[](std::size_t index) const noexcept { return data_[index]; }
};

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

class Texture {
    std::size_t width_;
    std::size_t height_;
    std::vector<Colour> data_;
public:
    constexpr Texture(std::size_t w, std::size_t h, std::vector<Colour> data)
        : width_(w), height_(h), data_(std::move(data)) {}
    constexpr Colour sample(FloatType u, FloatType v) const {
        // Convert UV (0-1) to pixel coordinates
        std::size_t ix = static_cast<std::size_t>(Clamp(u * static_cast<FloatType>(width_ - 1), 0.0f, static_cast<FloatType>(width_ - 1)));
        std::size_t iy = static_cast<std::size_t>(Clamp(v * static_cast<FloatType>(height_ - 1), 0.0f, static_cast<FloatType>(height_ - 1)));
        return data_[iy * width_ + ix];
    }
};

struct Material {
    Colour colour;
    std::shared_ptr<Texture> texture;
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
    glm::mat3 orientation_ = glm::mat3(
        glm::vec3(1.0f, 0.0f, 0.0f),  // right
        glm::vec3(0.0f, 1.0f, 0.0f),  // up
        glm::vec3(0.0f, 0.0f, -1.0f)  // forward
    );
    glm::vec3 orbit_target_ = { 0.0f, 0.0f, 0.0f };
    std::int64_t last_orbit_time_ = std::chrono::system_clock::now().time_since_epoch().count();
    Camera() = default;
    glm::vec4 world_to_clip(const glm::vec3& vertex, double aspect_ratio) const noexcept;
    glm::vec3 clip_to_ndc(const glm::vec4& clip) const noexcept;
    void start_orbiting(glm::vec3 target);
    void orbiting();
    void stop_orbiting();
    void rotate(FloatType angle_x, FloatType angle_y);
    void handle_event(const SDL_Event& event);
};

struct Face {
    std::array<glm::vec3, 3> vertices;
    std::array<std::uint8_t, 3> texture_vertices;
    std::array<glm::vec2, 3> texture_coords;  // Actual UV coordinates for this face
    Material material;
};

struct Object {
    std::string name;
    Material material;
    std::vector<Face> faces;
};

class Renderer {
public:
    enum Mode {
        Wireframe,
        Rasterized,
        Raytraced,
    };
private:
    DrawingWindow& window_;
    Mode mode_ = Rasterized;
    std::vector<FloatType> z_buffer_;
    double aspect_ratio_ = 1.0;
public:
    Renderer(DrawingWindow& window) noexcept;
    void clear() noexcept;
    ScreenNdcCoord ndc_to_screen(const glm::vec3& ndc, const glm::vec2& uv, FloatType w) const noexcept;
    void render(const Camera& camera, const Face& face) noexcept;
    void handle_event(const SDL_Event& event) noexcept;
private:
    void wireframe_render(const Camera& camera, const Face& face) noexcept;
    void rasterized_render(const Camera& camera, const Face& face) noexcept;
    void raytraced_render(const Camera& camera, const Face& face) noexcept;
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

class Model {
private:
    std::map<std::string, Material> materials_;
    std::vector<glm::vec3> vertices_;
    std::vector<glm::vec2> texture_coords_;  // vt coordinates from OBJ
    std::vector<Object> objects_;
public:
    Model() noexcept = default;
    void load_file(std::string filename);
    void draw(Renderer& renderer, const Camera& camera) const noexcept;
private:
    void load_materials(std::string filename);
    Texture load_texture(std::string filename);
};

class World {
private:
    std::vector<Model> groups_;
    Camera camera_;
public:
    void load_files(const std::vector<std::string>& filenames);
    void draw(Renderer& renderer) const noexcept;
    void handle_event(const SDL_Event& event) noexcept;
    void orbiting() noexcept;
};