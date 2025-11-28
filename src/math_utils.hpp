#pragma once
#include <glm/glm.hpp>
#include "utils.hpp"

inline glm::vec3 CalculateNormal(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2) noexcept {
    glm::vec3 edge1 = v1 - v0;
    glm::vec3 edge2 = v2 - v0;
    return glm::normalize(glm::cross(edge1, edge2));
}

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
    if (std::abs(a) < EPSILON) return false;
    FloatType f = 1.0f / a;
    glm::vec3 s = ray_origin - v0;
    FloatType u = f * glm::dot(s, h);
    if (u < 0.0f || u > 1.0f) return false;
    glm::vec3 q = glm::cross(s, edge1);
    FloatType v = f * glm::dot(ray_dir, q);
    if (v < 0.0f || u + v > 1.0f) return false;
    FloatType t = f * glm::dot(edge2, q);
    out_t = t; out_u = u; out_v = v;
    return t > EPSILON;
}

inline glm::vec3 CalculateBarycentric(glm::vec2 v0, glm::vec2 v1, glm::vec2 v2, glm::vec2 p) noexcept {
    glm::vec2 e0 = v1 - v0;
    glm::vec2 e1 = v2 - v0;
    glm::vec2 e2 = p - v0;
    FloatType denominator = e0.x * e1.y - e0.y * e1.x;
    FloatType invDenominator = 1.0f / denominator;
    FloatType weight_v1 = (e2.x * e1.y - e2.y * e1.x) * invDenominator;
    FloatType weight_v2 = (e0.x * e2.y - e0.y * e2.x) * invDenominator;
    FloatType weight_v0 = 1.0f - weight_v1 - weight_v2;
    return glm::vec3(weight_v0, weight_v1, weight_v2);
}

