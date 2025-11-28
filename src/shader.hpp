#pragma once
#include <optional>
#include "world.hpp"

struct Ray {
    glm::vec3 origin;
    glm::vec3 direction;
};

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
    bool ScatterLambertian(const Ray& r_in, const HitRecord& rec, const Material& mat, ScatterRecord& scattered) noexcept;
    bool ScatterMetal(const Ray& r_in, const HitRecord& rec, const Material& mat, ScatterRecord& scattered) noexcept;
    bool ScatterDielectric(const Ray& r_in, const HitRecord& rec, const Material& mat, ScatterRecord& scattered) noexcept;
}
