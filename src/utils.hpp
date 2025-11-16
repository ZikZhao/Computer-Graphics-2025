#pragma once
#include <glm/glm.hpp>
#include <array>

using FloatType = decltype(std::declval<glm::vec3>().x);

// ============================================================================
// Math Utilities
// ============================================================================

template<typename T>
constexpr T Clamp(T value, T min, T max) noexcept {
    return (value < min) ? min : (value > max) ? max : value;
}

constexpr FloatType ComputeInvZndc(FloatType progress, std::array<FloatType, 2> vertices_z_ndc) noexcept {
    return (1.0f - progress) / vertices_z_ndc[0] + progress / vertices_z_ndc[1];
}

constexpr FloatType ComputeInvZndc(std::array<FloatType, 3> bary, std::array<FloatType, 3> vertices_z_ndc) noexcept {
    return bary[0] / vertices_z_ndc[0] +
           bary[1] / vertices_z_ndc[1] +
           bary[2] / vertices_z_ndc[2];
}

// ============================================================================
// Ray Tracing Utilities
// ============================================================================

// Möller-Trumbore ray-triangle intersection algorithm
inline bool IntersectRayTriangle(
    const glm::vec3& ray_origin,
    const glm::vec3& ray_dir,
    const glm::vec3& v0,
    const glm::vec3& v1,
    const glm::vec3& v2,
    FloatType& out_t,
    FloatType& out_u,
    FloatType& out_v
) noexcept {
    constexpr FloatType EPSILON = 1e-6f;
    
    glm::vec3 edge1 = v1 - v0;
    glm::vec3 edge2 = v2 - v0;
    glm::vec3 h = glm::cross(ray_dir, edge2);
    FloatType a = glm::dot(edge1, h);
    
    // Ray is parallel to triangle
    if (std::abs(a) < EPSILON) {
        return false;
    }
    
    FloatType f = 1.0f / a;
    glm::vec3 s = ray_origin - v0;
    FloatType u = f * glm::dot(s, h);
    
    if (u < 0.0f || u > 1.0f) {
        return false;
    }
    
    glm::vec3 q = glm::cross(s, edge1);
    FloatType v = f * glm::dot(ray_dir, q);
    
    if (v < 0.0f || u + v > 1.0f) {
        return false;
    }
    
    FloatType t = f * glm::dot(edge2, q);
    
    out_t = t;
    out_u = u;
    out_v = v;
    
    return t > EPSILON;
}
