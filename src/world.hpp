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
constexpr auto Clamp(T value, T min, T max) {
    return (value < min) ? min : (value > max) ? max : value;
}

constexpr FloatType ComputeInvZndc(const std::array<FloatType, 3> bary, const std::array<FloatType, 3> vertices_z_ndc) {
    FloatType inv_z = bary[0] / vertices_z_ndc[0] +
                  bary[1] / vertices_z_ndc[1] +
                  bary[2] / vertices_z_ndc[2];
    return inv_z;
}

constexpr FloatType ComputeInvZndc(FloatType progress, const std::array<FloatType, 2> vertices_z_ndc) {
    FloatType inv_z = (1.0f - progress) / vertices_z_ndc[0] + progress / vertices_z_ndc[1];
    return inv_z;
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

// Vertex in clip space with attributes
struct ClipVertex {
    glm::vec4 position_clip;  // Homogeneous clip space coordinates
    Colour colour;
};

struct ScreenNdcCoord {
    FloatType x;
    FloatType y;
    FloatType z_ndc;
};

class Camera {
public:
    static constexpr std::int64_t OrbitInterval = 1'000'000'000 / 60; // 60 FPS
    static constexpr double FOV = 45.0;
    static constexpr double NearPlane = 0.001;
    static constexpr double FarPlane = 100.0;
public:
    glm::vec3 position_ = { 0.0f, 0.0f, 4.0f };
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
    Colour colour;
};

class Object {
    friend class Group;
private:
    std::string name_;
    Colour colour_;
    std::vector<Face> faces_;
public:
    Object(const std::string& name);
    void set_colour(const Colour& colour);
    void add_face(const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3);
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
    ScreenNdcCoord ndc_to_screen(const glm::vec3& ndc) const noexcept;
    void render(const Camera& camera, const Face& face) noexcept;
    void handle_event(const SDL_Event& event) noexcept;
private:
    void wireframe_render(const Camera& camera, const Face& face) noexcept;
    void rasterized_render(const Camera& camera, const Face& face) noexcept;
    void raytraced_render(const Camera& camera, const Face& face) noexcept;
    // Clipping utilities
    static bool inside_plane(const glm::vec4& v, ClipPlane plane) noexcept;
    static FloatType compute_intersection_t(const glm::vec4& v0, const glm::vec4& v1, ClipPlane plane) noexcept;
    static ClipVertex intersect_plane(const ClipVertex& v0, const ClipVertex& v1, ClipPlane plane) noexcept;
    static InplaceVector<ClipVertex, 9> clip_against_plane(const InplaceVector<ClipVertex, 9>& input, ClipPlane plane) noexcept;
    InplaceVector<ClipVertex, 9> clip_triangle(const Camera& camera, const Face& face) noexcept;
};

class Group {
private:
    std::map<std::string, Colour> materials_;
    std::vector<glm::vec3> vertices_;
    std::vector<Object> objects_;
public:
    Group() noexcept = default;
    void load_file(std::string filename);
    void draw(Renderer& renderer, const Camera& camera) const noexcept;
private:
    void load_materials(std::string filename);
};

class World {
private:
    std::vector<Group> groups_;
    Camera camera_;
public:
    void load_files(const std::vector<std::string>& filenames);
    void draw(Renderer& renderer) const noexcept;
    void handle_event(const SDL_Event& event) noexcept;
    void orbiting() noexcept;
};