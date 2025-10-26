#pragma once
#include <fstream>
#include <vector>
#include <sstream>
#include <map>
#include <stdexcept>
#include <memory>
#include <chrono>
#include "DrawingWindow.h"
#include "ModelTriangle.h"
#include "Colour.h"
#include "Utils.h"

class ZBuffer;

float ComputeZndc(glm::vec3 bary, glm::vec3 vertices_z_view);

float ComputeZndc(float progress, glm::vec2 vertices_z_view);

void StrokeLine(DrawingWindow& window, ZBuffer& z_buffer, glm::vec3 from, glm::vec3 to, std::uint32_t colour);

void FillTriangle(DrawingWindow& window, ZBuffer& z_buffer, glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, std::uint32_t colour);

class ZBuffer {
private:
    std::vector<float> buffer_;
    size_t width_ = 0;
    size_t height_ = 0;
public:
    ZBuffer() = default;
    ZBuffer(size_t width, size_t height);
    ZBuffer& reset(size_t width, size_t height);
    bool replace_if_closer(std::int64_t x, std::int64_t y, float z_ndc);
};

class Camera {
public:
    static constexpr std::int64_t OrbitInterval = 1000000000 / 60; // 60 FPS
public:
    glm::vec3 position_ = { 0.0f, 0.0f, 4.0f };
    glm::vec3 forward_ = { 0.0f, 0.0f, -1.0f };
    glm::vec3 up_ = { 0.0f, 1.0f, 0.0f };
    glm::vec3 right_ = glm::normalize(glm::cross(forward_, up_));
    glm::vec3 orbit_target_ = { 0.0f, 0.0f, 0.0f };
    std::int64_t last_orbit_time_ = std::chrono::system_clock::now().time_since_epoch().count();
    double fov = 45.0;
    Camera() = default;
    void orbiting();
    void set_orbit(glm::vec3 target);
    void remove_orbit();
    void rotate(float angle_x, float angle_y);
    void handle_event(const SDL_Event& event);
};

class Face {
public:
    std::array<glm::vec3, 3> vertices_;
    const Colour colour_;
    Face(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3, Colour c) : vertices_{{ v1, v2, v3 }}, colour_(c) {}
    Face(const std::array<glm::vec3, 3>& verts, Colour c) : vertices_(verts), colour_(c) {}
    std::array<glm::vec3, 3> to_ndc(const DrawingWindow& window, const Camera& camera) const;
    std::array<glm::vec3, 3> to_screen(const DrawingWindow& window, const std::array<glm::vec3, 3>& ndc) const;
    void draw(DrawingWindow& window, const Camera& camera, ZBuffer& z_buffer) const;
};

class Object {
private:
    std::string name_;
    Colour colour_;
    std::vector<Face> faces_;
public:
    Object(const std::string& name);
    void set_colour(const Colour& colour);
    void add_face(const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3);
    void draw(DrawingWindow& window, const Camera& camera, ZBuffer& z_buffer) const;
};

class World {
private:
    std::vector<glm::vec3> vertices_;
    std::vector<Object> objects_;
    Camera camera_;
    ZBuffer z_buffer_;
public:
    void LoadFromFile(const std::string& filename);
    void draw(DrawingWindow& window);
    void handle_event(const SDL_Event& event, DrawingWindow& window);
};
