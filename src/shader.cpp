#include "shader.hpp"

#include <numbers>
#include <random>

static thread_local std::mt19937 rng(std::random_device{}());
static thread_local std::uniform_real_distribution<FloatType> dist(0.0f, 1.0f);

static glm::vec3 random_in_hemisphere(const glm::vec3& normal) noexcept {
    FloatType u1 = dist(rng);
    FloatType u2 = dist(rng);
    FloatType r = std::sqrt(u1);
    FloatType theta = 2.0f * std::numbers::pi * u2;
    FloatType x = r * std::cos(theta);
    FloatType y = r * std::sin(theta);
    FloatType z = std::sqrt(std::max(0.0f, 1.0f - u1));
    glm::vec3 w = normal;
    glm::vec3 a = std::abs(w.x) > 0.9f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    glm::vec3 v = glm::normalize(glm::cross(w, a));
    glm::vec3 u = glm::cross(w, v);
    return glm::normalize(u * x + v * y + w * z);
}

namespace Shading {

bool ScatterLambertian(const Ray& r_in, const HitRecord& rec, const Material& mat, ScatterRecord& scattered) noexcept {
    glm::vec3 scatter_dir = random_in_hemisphere(rec.normal);
    scattered.attenuation = rec.color;
    scattered.emission = glm::vec3(0.0f);
    scattered.is_specular = false;
    // Epsilon origin shift: start the diffuse bounce slightly off the surface
    // to avoid self-intersection due to FP error (shadow acne).
    scattered.scattered_ray = Ray{rec.intersectionPoint + rec.normal * 0.001f, scatter_dir};
    // PDF for cosine-weighted hemisphere sampling. The integrator weights
    // contributions by 1/PDF to keep the Monte Carlo estimator unbiased.
    scattered.pdf = glm::dot(rec.normal, scatter_dir) / std::numbers::pi;
    return true;
}

bool ScatterMetal(const Ray& r_in, const HitRecord& rec, const Material& mat, ScatterRecord& scattered) noexcept {
    glm::vec3 reflected = glm::reflect(r_in.direction, rec.normal);
    scattered.attenuation = rec.color;
    scattered.emission = glm::vec3(0.0f);
    scattered.is_specular = true;
    // Epsilon origin shift: avoid immediate re-hit of the same triangle.
    scattered.scattered_ray = Ray{rec.intersectionPoint + rec.normal * 0.001f, reflected};
    // Specular reflection is a delta distribution; treat PDF as 1 for the
    // chosen path since the sampling is deterministic.
    scattered.pdf = 1.0f;
    return glm::dot(reflected, rec.normal) > 0.0f;
}

static FloatType fresnel_schlick(FloatType cos_theta, FloatType ior_ratio) noexcept {
    FloatType r0 = (1.0f - ior_ratio) / (1.0f + ior_ratio);
    r0 = r0 * r0;
    FloatType cos_term = 1.0f - cos_theta;
    FloatType cos5 = cos_term * cos_term * cos_term * cos_term * cos_term;
    return r0 + (1.0f - r0) * cos5;
}

bool ScatterDielectric(const Ray& r_in, const HitRecord& rec, const Material& mat, ScatterRecord& scattered) noexcept {
    constexpr FloatType epsilon = 0.001f;

    glm::vec3 normal = rec.normal;
    FloatType cos_theta_i = glm::dot(-r_in.direction, normal);
    bool entering = cos_theta_i > 0.0f;

    FloatType ior_from = entering ? 1.0f : mat.ior;
    FloatType ior_to = entering ? mat.ior : 1.0f;
    FloatType ior_ratio = ior_from / ior_to;

    if (!entering) {
        normal = -normal;
        cos_theta_i = -cos_theta_i;
    }

    glm::vec3 refracted = glm::refract(r_in.direction, normal, ior_ratio);
    bool total_internal_reflection = (glm::length(refracted) < 0.0001f);

    if (total_internal_reflection) {
        glm::vec3 reflected = glm::reflect(r_in.direction, normal);
        scattered.attenuation = glm::vec3(1.0f, 1.0f, 1.0f);
        scattered.emission = glm::vec3(0.0f, 0.0f, 0.0f);
        scattered.is_specular = true;
        // Offset along the normal to prevent the reflected ray from
        // self-intersecting the refractive surface.
        scattered.scattered_ray = Ray{rec.intersectionPoint + normal * epsilon, reflected};
        scattered.pdf = 1.0f;
        return true;
    }

    FloatType fresnel = fresnel_schlick(std::abs(cos_theta_i), ior_ratio);

    FloatType reflect_prob = fresnel * (1.0f - mat.tw);
    FloatType refract_prob = (1.0f - fresnel) * mat.tw;

    FloatType total_prob = reflect_prob + refract_prob;
    if (total_prob < 0.01f) {
        return false;  // Absorbed
    }

    reflect_prob /= total_prob;

    if (dist(rng) < reflect_prob) {
        glm::vec3 reflected = glm::reflect(r_in.direction, normal);
        scattered.attenuation = glm::vec3(1.0f, 1.0f, 1.0f);
        scattered.emission = glm::vec3(0.0f, 0.0f, 0.0f);
        scattered.is_specular = true;
        // Offset into the air side.
        scattered.scattered_ray = Ray{rec.intersectionPoint + normal * epsilon, reflected};
        scattered.pdf = reflect_prob;
        return true;
    } else {
        scattered.attenuation = glm::vec3(1.0f, 1.0f, 1.0f);
        scattered.emission = glm::vec3(0.0f, 0.0f, 0.0f);
        scattered.is_specular = true;
        // Offset into the medium side.
        scattered.scattered_ray = Ray{rec.intersectionPoint - normal * epsilon, refracted};
        scattered.pdf = 1.0f - reflect_prob;
        return true;
    }
}

}  // namespace Shading
