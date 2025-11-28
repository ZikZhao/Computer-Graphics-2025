#pragma once
#include <glm/glm.hpp>
#include <array>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <fstream>
#include <string>
#include <cstdint>
#include <vector>
#include <thread>
#include <iostream>

using FloatType = decltype(std::declval<glm::vec3>().x);

// ============================================================================
// Math Utilities
// ============================================================================

constexpr FloatType ComputeInvZndc(FloatType progress, std::array<FloatType, 2> vertices_z_ndc) noexcept {
    return (1.0f - progress) / vertices_z_ndc[0] + progress / vertices_z_ndc[1];
}

constexpr FloatType ComputeInvZndc(std::array<FloatType, 3> bary, std::array<FloatType, 3> vertices_z_ndc) noexcept {
    return bary[0] / vertices_z_ndc[0] +
           bary[1] / vertices_z_ndc[1] +
           bary[2] / vertices_z_ndc[2];
}

// Container utilities moved to containers.hpp

// ============================================================================
// Ray Tracing Utilities moved to math_utils.hpp

// ============================================================================
// Low-Discrepancy Sampling (Halton Sequence)
// ============================================================================

inline FloatType Halton(int index, int base) noexcept {
    FloatType result = 0.0f;
    FloatType f = 1.0f / base;
    int i = index;
    while (i > 0) {
        result += f * (i % base);
        i = i / base;
        f = f / base;
    }
    return result;
}

inline glm::vec3 SampleSphereHalton(int index, FloatType radius, const glm::vec3& center) noexcept {
    FloatType u = Halton(index, 2);
    FloatType v = Halton(index, 3);
    
    FloatType theta = 2.0f * std::numbers::pi * u;
    FloatType phi = std::acos(2.0f * v - 1.0f);
    
    FloatType sin_phi = std::sin(phi);
    FloatType x = radius * sin_phi * std::cos(theta);
    FloatType y = radius * sin_phi * std::sin(theta);
    FloatType z = radius * std::cos(phi);
    
    return center + glm::vec3(x, y, z);
}

inline glm::vec3 SampleUnitVectorHalton(int index) noexcept {
    return SampleSphereHalton(index, 1.0f, glm::vec3(0.0f));
}

inline glm::vec3 SampleConeHalton(int index, const glm::vec3& direction, FloatType cone_angle) noexcept {
    FloatType u1 = Halton(index, 2);
    FloatType u2 = Halton(index, 3);
    
    FloatType cos_angle = std::cos(cone_angle);
    FloatType z = cos_angle + (1.0f - cos_angle) * u1;
    FloatType phi = 2.0f * std::numbers::pi * u2;
    
    FloatType sin_theta = std::sqrt(1.0f - z * z);
    glm::vec3 sample_dir(sin_theta * std::cos(phi), sin_theta * std::sin(phi), z);
    
    // Build orthonormal basis around direction
    glm::vec3 up = std::abs(direction.y) < 0.999f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    glm::vec3 right = glm::normalize(glm::cross(up, direction));
    glm::vec3 forward = glm::cross(direction, right);
    
    // Transform sample to world space
    return glm::normalize(sample_dir.x * right + sample_dir.y * forward + sample_dir.z * direction);
}

// ============================================================================
// VideoRecorder moved to video_recorder.hpp/cpp

