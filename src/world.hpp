#pragma once
#include <fstream>
#include <vector>
#include <sstream>
#include <map>
#include <stdexcept>
#include <memory>
#include <chrono>
#include <array>
#include "DrawingWindow.h"
#include "Utils.h"

using FloatType = decltype(std::declval<glm::vec3>().x);

struct ScreenNdcCoord {
    FloatType x;
    FloatType y;
    FloatType z_ndc;
};

template<typename T>
constexpr auto Clamp(T value, T min, T max) {
    return (value < min) ? min : (value > max) ? max : value;
}

float ComputeZndc(std::array<FloatType, 3> bary, std::array<FloatType, 3> vertices_z_view);

float ComputeZndc(float progress, std::array<FloatType, 2> vertices_z_view);

struct Colour {
    std::uint8_t red;
    std::uint8_t green;
    std::uint8_t blue;
    constexpr operator std::uint32_t () const {
        return (255 << 24) + (red << 16) + (green << 8) + blue;
    }
};

class Camera {
public:
    static constexpr std::int64_t OrbitInterval = 1'000'000'000 / 60; // 60 FPS
    static constexpr double FOV = 45.0;
    static constexpr double NearPlane = 0.1;
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
    glm::vec3 world_to_ndc(const glm::vec3& vertex, double aspect_ratio) const noexcept;
    void orbiting();
    void set_orbit(glm::vec3 target);
    void stop_orbit();
    void rotate(float angle_x, float angle_y);
    void handle_event(const SDL_Event& event);
};

struct Face {
    std::array<glm::vec3, 3> vertices;
    Colour colour;
};

class Object {
    friend class World;
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
    std::vector<float> z_buffer_;
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
};

class World {
private:
    std::map<std::string, Colour> materials_;
    std::vector<glm::vec3> vertices_;
    std::vector<Object> objects_;
    Camera camera_;
public:
    World() noexcept = default;
    void load_file(std::string filename);
    void draw(Renderer& renderer) const noexcept;
    void handle_event(const SDL_Event& event) noexcept;
private:
    void load_materials(std::string filename);
};
