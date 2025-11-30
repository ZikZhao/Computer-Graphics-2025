#pragma once
#include <memory>
#include <vector>

#include "photon_map.hpp"
#include "world.hpp"

/**
 * @class RayTracer
 * @brief The core ray tracing engine.
 *
 * This class implements a path tracer with support for:
 * - Monte Carlo integration for soft shadows (Area Lights)
 * - Depth of Field (DoF)
 * - Global Illumination via Caustics (Photon Mapping)
 * - Dielectric materials (Refraction/Reflection with Fresnel)
 * - Metallic materials (Glossy Reflection)
 * - Volumetric Absorption (Beer's Law)
 */
class RayTracer {
public:
    /**
     * @brief Tracks the current medium the ray is traversing.
     * Used for calculating volumetric absorption (Beer's Law).
     */
    struct MediumState {
        const Material* material = nullptr;       /// Pointer to the material of the medium.
        glm::vec3 entry_point = glm::vec3(0.0f);  /// Point where the ray entered the medium.
        FloatType entry_distance = 0.0f;          /// Distance from camera/origin to entry point.
    };

public:
    /**
     * @brief Clamps the radiance values to avoid fireflies/NaNs.
     * @param c Input color.
     * @param max_luma Maximum allowed component value.
     * @return Clamped color.
     */
    [[nodiscard]] static constexpr ColourHDR ClampRadiance(
        const ColourHDR& c, FloatType max_component
    ) noexcept {
        FloatType r = std::min(c.red, max_component);
        FloatType g = std::min(c.green, max_component);
        FloatType b = std::min(c.blue, max_component);
        return ColourHDR{.red = r, .green = g, .blue = b};
    }

private:
    const World& world_;
    std::unique_ptr<PhotonMap> photon_map_;

public:
    RayTracer(const World& world);

public:
    /**
     * @brief Checks if the photon map has finished building.
     */
    [[nodiscard]] bool is_photon_map_ready() const noexcept {
        return photon_map_ && photon_map_->is_ready();
    }

public:
    /**
     * @brief Renders a single pixel using standard pinhole camera model.
     *
     * @param cam The camera.
     * @param x Pixel x-coordinate.
     * @param y Pixel y-coordinate.
     * @param width Image width.
     * @param height Image height.
     * @param soft_shadows Enable area light sampling for soft shadows.
     * @param use_caustics Enable photon map lookup.
     * @param sample_index Index of the current sample (for accumulation).
     * @param initial_seed Seed for RNG.
     * @return The computed color for this sample.
     */
    [[nodiscard]] ColourHDR render_pixel(
        const Camera& cam,
        int x,
        int y,
        int width,
        int height,
        bool soft_shadows,
        bool use_caustics = false,
        int sample_index = 0,
        std::uint32_t initial_seed = 1u
    ) const noexcept;

    /**
     * @brief Renders a single pixel with Depth of Field (DoF).
     *
     * @param cam The camera.
     * @param x Pixel x-coordinate.
     * @param y Pixel y-coordinate.
     * @param width Image width.
     * @param height Image height.
     * @param focal_distance Distance to the focal plane.
     * @param aperture_size Size of the lens aperture (radius).
     * @param samples Number of samples per pixel for DoF.
     * @param soft_shadows Enable area light sampling.
     * @param use_caustics Enable photon map lookup.
     * @return The averaged color of the samples.
     */
    [[nodiscard]] ColourHDR render_pixel_dof(
        const Camera& cam,
        int x,
        int y,
        int width,
        int height,
        FloatType focal_distance,
        FloatType aperture_size,
        int samples,
        bool soft_shadows,
        bool use_caustics = false
    ) const noexcept;

private:
    /**
     * @brief Recursive ray tracing function.
     *
     * @param ro Ray origin.
     * @param rd Ray direction (normalized).
     * @param depth Current recursion depth.
     * @param medium Current medium state (for absorption).
     * @param soft_shadows Enable soft shadows.
     * @param use_caustics Enable caustics.
     * @param sample_index Current sample index.
     * @param throughput Current path throughput (for Russian Roulette).
     * @param rng RNG state.
     * @return Radiance found along the ray.
     */
    ColourHDR trace_ray(
        const glm::vec3& ro,
        const glm::vec3& rd,
        int depth,
        const MediumState& medium,
        bool soft_shadows,
        bool use_caustics,
        int sample_index,
        const glm::vec3& throughput,
        std::uint32_t& rng
    ) const noexcept;

    /**
     * @brief Intersects the ray with the scene geometry.
     */
    RayTriangleIntersection hit(const glm::vec3& ro, const glm::vec3& rd) const noexcept;

    /**
     * @brief Computes visibility between two points (shadow ray).
     * @return Transmission color (e.g., white if visible, black if blocked, or filtered color if
     * through transparent object).
     */
    glm::vec3 compute_transmittance_bvh(const glm::vec3& point, const glm::vec3& light_pos)
        const noexcept;
};
