#pragma once
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

/// @brief Floating-point type used throughout the renderer.
using FloatType = decltype(std::declval<glm::vec3>().x);

/**
 * @brief Fixed-capacity, stack-allocated vector with inplace storage.
 *
 * Provides a minimal subset of `std::inplace_vector` semantics for C++20.
 * Elements are constructed in a preallocated buffer and capacity is limited
 * to `N`. Intended for hot paths where heap allocation is undesirable.
 */
template <typename T, std::size_t N>
class InplaceVector {
private:
    alignas(T) std::byte data_[N * sizeof(T)];
    std::size_t size_ = 0;

public:
    constexpr InplaceVector() noexcept = default;
    constexpr InplaceVector(auto&&... args) noexcept {
        assert((sizeof...(args)) <= N);
        (emplace_back(std::forward<decltype(args)>(args)), ...);
    }
    constexpr std::size_t size() const noexcept { return size_; }
    constexpr T& operator[](std::size_t index) noexcept {
        return *reinterpret_cast<T*>(&data_[index * sizeof(T)]);
    }
    constexpr const T& operator[](std::size_t index) const noexcept {
        return *reinterpret_cast<const T*>(&data_[index * sizeof(T)]);
    }
    constexpr void push_back(const T& value) noexcept {
        assert(size_ < N);
        new (&data_[size_ * sizeof(T)]) T(value);
        size_++;
    }
    constexpr void emplace_back(T&& value) noexcept {
        assert(size_ < N);
        new (&data_[size_ * sizeof(T)]) T(std::move(value));
        size_++;
    }
};

/**
 * @brief Read a PPM (P6 binary) image file header and return the stream positioned at pixel data.
 * @param path Path to the PPM file.
 * @return Anonymous struct containing width, height, and the file stream positioned at pixel data.
 * @throws std::runtime_error if file cannot be opened or parsed.
 */
inline auto ReadPPM(const std::filesystem::path& path) {
    struct PPMData {
        std::size_t width;
        std::size_t height;
        std::size_t max;
        std::ifstream stream;
    };

    std::ifstream file(path, std::ifstream::binary);
    if (!file.is_open()) throw std::runtime_error("Could not open PPM file: " + path.string());

    std::string magic_number;
    std::getline(file, magic_number);
    if (magic_number != "P6")
        throw std::runtime_error("Invalid PPM format (expected P6): " + path.string());

    std::string line;
    std::getline(file, line);
    while (!line.empty() && line[0] == '#') {
        std::getline(file, line);
    }

    std::istringstream size_stream(line);
    std::size_t width, height;
    if (!(size_stream >> width >> height))
        throw std::runtime_error("Failed to parse PPM dimensions: " + path.string());

    std::getline(file, line);
    std::istringstream max_stream(line);
    std::size_t max;
    if (!(max_stream >> max))
        throw std::runtime_error("Failed to parse PPM max value: " + path.string());

    return PPMData{width, height, max, std::move(file)};
}

/**
 * @brief Write a PPM (P6 binary) image file.
 * @param filename Output file path.
 * @param width Image width in pixels.
 * @param height Image height in pixels.
 * @param pixels Pixel data as ARGB 32-bit values.
 */
inline void WritePPM(
    const std::string& filename,
    std::size_t width,
    std::size_t height,
    std::size_t max,
    const std::vector<std::uint32_t>& pixels
) {
    std::ofstream output_stream(filename, std::ofstream::out);
    output_stream << "P6\n";
    output_stream << width << " " << height << "\n";
    output_stream << max << "\n";

    for (std::size_t i = 0; i < width * height; i++) {
        std::array<char, 3> rgb{
            {static_cast<char>((pixels[i] >> 16) & 0xFF),
             static_cast<char>((pixels[i] >> 8) & 0xFF),
             static_cast<char>((pixels[i] >> 0) & 0xFF)}
        };
        output_stream.write(rgb.data(), 3);
    }
}

/**
 * @brief Computes inverse Z in NDC for a linear interpolation along an edge.
 * @param progress Interpolation factor in [0,1].
 * @param vertices_z_ndc Z components (NDC) for the two edge endpoints.
 * @return 1/z value suitable for depth comparison and perspective-correct interpolation.
 */
constexpr FloatType InvZndc(FloatType progress, std::array<FloatType, 2> vertices_z_ndc) noexcept {
    return (1.0f - progress) / vertices_z_ndc[0] + progress / vertices_z_ndc[1];
}

/**
 * @brief Computes inverse Z in NDC using barycentric weights over a triangle.
 * @param bary Barycentric weights for triangle vertices (sum to 1).
 * @param vertices_z_ndc Z components (NDC) for the three triangle vertices.
 * @return 1/z value for perspective-correct interpolation and depth testing.
 */
constexpr FloatType InvZndc(
    std::array<FloatType, 3> bary, std::array<FloatType, 3> vertices_z_ndc
) noexcept {
    return bary[0] / vertices_z_ndc[0] + bary[1] / vertices_z_ndc[1] + bary[2] / vertices_z_ndc[2];
}

/**
 * @brief Computes a triangle face normal from three vertices.
 * @param v0 First vertex in world space.
 * @param v1 Second vertex in world space.
 * @param v2 Third vertex in world space.
 * @return Unit-length geometric normal (right-hand cross).
 */
inline glm::vec3 FaceNormal(
    const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2
) noexcept {
    glm::vec3 edge1 = v1 - v0;
    glm::vec3 edge2 = v2 - v0;
    return glm::normalize(glm::cross(edge1, edge2));
}

/**
 * @brief Möller–Trumbore ray–triangle intersection.
 * @param ray_origin Ray origin in world space.
 * @param ray_dir Normalized ray direction.
 * @param v0 Triangle vertex 0.
 * @param v1 Triangle vertex 1.
 * @param v2 Triangle vertex 2.
 * @param out_t Intersection distance along the ray.
 * @param out_u Barycentric coordinate for v1.
 * @param out_v Barycentric coordinate for v2.
 * @return True if the ray hits the triangle at t > 0.
 */
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
    out_t = t;
    out_u = u;
    out_v = v;
    return t > EPSILON;
}

/**
 * @brief Computes barycentric coordinates of a point relative to a 2D triangle.
 * @param v0 Vertex 0 in screen or 2D space.
 * @param v1 Vertex 1 in screen or 2D space.
 * @param v2 Vertex 2 in screen or 2D space.
 * @param p Point to evaluate.
 * @return Barycentric weights (w0, w1, w2) that sum to 1.
 */
inline glm::vec3 Barycentric(glm::vec2 v0, glm::vec2 v1, glm::vec2 v2, glm::vec2 p) noexcept {
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

/**
 * @brief Generates a uniform random float in [0,1) using thread-local RNG.
 * @return Random float in [0,1).
 */
inline FloatType Rand() noexcept {
    thread_local std::mt19937 rng(std::random_device{}());
    return std::uniform_real_distribution<FloatType>(0.0f, 1.0f)(rng);
}

/**
 * @brief Generates a uniform random float in [0,1) using PCG hash.
 * @param seed RNG state (updated in-place).
 * @return Random float in [0,1).
 */
constexpr FloatType PCGRandomFloat(std::uint32_t& seed) noexcept {
    seed = seed * 747796405u + 2891336453u;
    std::uint32_t word = ((seed >> ((seed >> 28u) + 4u)) ^ seed) * 277803737u;
    seed = word ^ (word >> 22u);
    return static_cast<FloatType>((seed >> 8) & 0x00FFFFFFu) / 16777216.0f;
}

/**
 * @brief Computes luminance from RGB using ITU-R BT.709 coefficients.
 * @param rgb RGB color (either glm::vec3 or ColourHDR-like with .red/.green/.blue).
 * @return Luminance value.
 */
constexpr FloatType Luminance(const glm::vec3& rgb) noexcept {
    return 0.2126f * rgb.x + 0.7152f * rgb.y + 0.0722f * rgb.z;
}

/**
 * @brief Computes effective absorption coefficient for volumetric media.
 *
 * If sigma_a is non-zero, uses it directly. Otherwise derives from base_color and td
 * (transmission distance) using Beer-Lambert law inversion.
 *
 * @param sigma_a Explicit absorption coefficient (use if length > 0).
 * @param base_color Base color of the medium (used to derive absorption).
 * @param td Transmission distance at which base_color is achieved.
 * @return Effective absorption coefficient (sigma_a).
 */
inline glm::vec3 EffectiveSigmaA(
    const glm::vec3& sigma_a, const glm::vec3& base_color, FloatType td
) noexcept {
    if (glm::length(sigma_a) > 0.0f) {
        return sigma_a;
    }
    if (td > 0.0f) {
        return glm::vec3(
            -std::log(std::max(base_color.r, 0.001f)) / td,
            -std::log(std::max(base_color.g, 0.001f)) / td,
            -std::log(std::max(base_color.b, 0.001f)) / td
        );
    }
    return glm::vec3(0.0f);
}

/**
 * @brief Applies Beer-Lambert absorption over a given distance.
 * @param sigma_a Absorption coefficient per unit distance.
 * @param distance Distance traveled through the medium.
 * @return Transmittance factor (multiply with incoming radiance).
 */
inline glm::vec3 BeerLambert(const glm::vec3& sigma_a, FloatType distance) noexcept {
    return glm::vec3(
        std::exp(-sigma_a.r * distance),
        std::exp(-sigma_a.g * distance),
        std::exp(-sigma_a.b * distance)
    );
}

/**
 * @brief Halton low-discrepancy sequence term.
 * @param index Sequence index (>= 0).
 * @param base Prime base (e.g., 2 or 3).
 * @return Value in [0,1).
 */
constexpr FloatType Halton(int index, int base) noexcept {
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

/**
 * @brief Samples a point on a sphere using Halton sequences.
 * @param index Sample index.
 * @param radius Sphere radius.
 * @param center Sphere center.
 * @return Position on the sphere surface.
 */
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

/**
 * @brief Samples a unit direction uniformly over the sphere.
 * @param index Sample index.
 * @return Unit-length direction vector.
 */
inline glm::vec3 SampleUnitVectorHalton(int index) noexcept {
    return SampleSphereHalton(index, 1.0f, glm::vec3(0.0f));
}

/**
 * @brief Samples a direction within a cone around `direction` using Halton.
 * @param index Sample index.
 * @param direction Cone axis (normalized).
 * @param cone_angle Half-angle of the cone in radians.
 * @return Unit-length direction within the cone.
 */
inline glm::vec3 SampleConeHalton(
    int index, const glm::vec3& direction, FloatType cone_angle
) noexcept {
    FloatType u1 = Halton(index, 2);
    FloatType u2 = Halton(index, 3);

    FloatType cos_angle = std::cos(cone_angle);
    FloatType z = cos_angle + (1.0f - cos_angle) * u1;
    FloatType phi = 2.0f * std::numbers::pi * u2;

    FloatType sin_theta = std::sqrt(1.0f - z * z);
    glm::vec3 sample_dir(sin_theta * std::cos(phi), sin_theta * std::sin(phi), z);

    // Basis construction: pick an 'up' that avoids degeneracy when direction
    // aligns with the world up, then build a stable orthonormal frame to map
    // local samples into world space without introducing skew.
    glm::vec3 up =
        std::abs(direction.y) < 0.999f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    glm::vec3 right = glm::normalize(glm::cross(up, direction));
    glm::vec3 forward = glm::cross(direction, right);

    // Transform: apply the frame so the sample respects the requested cone
    // angle around 'direction' while preserving low-discrepancy properties.
    return glm::normalize(sample_dir.x * right + sample_dir.y * forward + sample_dir.z * direction);
}

/**
 * @brief Maps a point from the unit square [0,1]x[0,1] to a unit disk.
 *
 * Uses Concentric Mapping to preserve area and adjacency, avoiding the
 * distortion at the center that simple polar mapping causes.
 */
inline glm::vec2 SampleDiskConcentric(FloatType u1, FloatType u2) noexcept {
    // Map [0,1] to [-1,1]
    FloatType a = 2.0f * u1 - 1.0f;
    FloatType b = 2.0f * u2 - 1.0f;

    if (a == 0.0f && b == 0.0f) {
        return glm::vec2(0.0f, 0.0f);
    }

    FloatType r, theta;
    if (a * a > b * b) {
        r = a;
        theta = (std::numbers::pi / 4.0f) * (b / a);
    } else {
        r = b;
        theta = (std::numbers::pi / 2.0f) - (std::numbers::pi / 4.0f) * (a / b);
    }

    return glm::vec2(r * std::cos(theta), r * std::sin(theta));
}
