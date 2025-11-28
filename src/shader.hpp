#pragma once
#include <optional>
#include "world.hpp"

/**
 * @brief Simple ray definition used by shading routines.
 */
struct Ray {
    glm::vec3 origin;
    glm::vec3 direction;
};

/**
 * @brief Output of a material scatter event.
 */
struct ScatterRecord {
    glm::vec3 attenuation;
    glm::vec3 emission;
    bool is_specular;
    std::optional<Ray> scattered_ray;
    FloatType pdf;
    ScatterRecord() noexcept
        : attenuation(1.0f, 1.0f, 1.0f), emission(0.0f), is_specular(false), scattered_ray(std::nullopt), pdf(1.0f) {}
};

using HitRecord = RayTriangleIntersection;

namespace Shading {
    /**
     * @brief Diffuse (Lambertian) scatter producing a hemisphere sample.
     * @param r_in Incident ray.
     * @param rec Intersection data.
     * @param mat Material parameters.
     * @param scattered Output scatter record.
     * @return True if a valid scattered ray was produced.
     */
    bool ScatterLambertian(const Ray& r_in, const HitRecord& rec, const Material& mat, ScatterRecord& scattered) noexcept;

    /**
     * @brief Perfect specular reflection for metallic surfaces.
     * @param r_in Incident ray.
     * @param rec Intersection data.
     * @param mat Material parameters.
     * @param scattered Output scatter record.
     * @return True if reflection is valid (front-facing).
     */
    bool ScatterMetal(const Ray& r_in, const HitRecord& rec, const Material& mat, ScatterRecord& scattered) noexcept;

    /**
     * @brief Dielectric scatter using refraction/reflection with Fresnel mixing.
     * @param r_in Incident ray.
     * @param rec Intersection data.
     * @param mat Material parameters (ior/tw/td).
     * @param scattered Output scatter record.
     * @return True if either reflected or refracted ray was produced.
     */
    bool ScatterDielectric(const Ray& r_in, const HitRecord& rec, const Material& mat, ScatterRecord& scattered) noexcept;
}
