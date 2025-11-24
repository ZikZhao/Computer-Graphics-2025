#include "shader.hpp"
#include <random>
#include <numbers>

// Random number generator for scattering
static thread_local std::mt19937 rng(std::random_device{}());
static thread_local std::uniform_real_distribution<FloatType> dist(0.0f, 1.0f);

// Generate random direction in hemisphere around normal
glm::vec3 LambertianShader::random_in_hemisphere(const glm::vec3& normal) noexcept {
    // Cosine-weighted hemisphere sampling
    FloatType u1 = dist(rng);
    FloatType u2 = dist(rng);
    
    FloatType r = std::sqrt(u1);
    FloatType theta = 2.0f * std::numbers::pi * u2;
    
    FloatType x = r * std::cos(theta);
    FloatType y = r * std::sin(theta);
    FloatType z = std::sqrt(std::max(0.0f, 1.0f - u1));
    
    // Build orthonormal basis around normal
    glm::vec3 w = normal;
    glm::vec3 a = std::abs(w.x) > 0.9f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    glm::vec3 v = glm::normalize(glm::cross(w, a));
    glm::vec3 u = glm::cross(w, v);
    
    return glm::normalize(u * x + v * y + w * z);
}

bool LambertianShader::scatter(
    const Ray& r_in,
    const HitRecord& rec,
    const Material& mat,
    ScatterRecord& scattered
) const {
    // Lambertian diffuse: scatter in random direction
    glm::vec3 scatter_dir = random_in_hemisphere(rec.normal);
    
    scattered.attenuation = rec.color;  // Use surface color
    scattered.emission = glm::vec3(0.0f, 0.0f, 0.0f);
    scattered.is_specular = false;
    scattered.scattered_ray = Ray{rec.intersectionPoint + rec.normal * 0.001f, scatter_dir};
    scattered.pdf = glm::dot(rec.normal, scatter_dir) / std::numbers::pi;
    
    return true;
}

bool MetalShader::scatter(
    const Ray& r_in,
    const HitRecord& rec,
    const Material& mat,
    ScatterRecord& scattered
) const {
    // Perfect specular reflection
    glm::vec3 reflected = glm::reflect(r_in.direction, rec.normal);
    
    scattered.attenuation = rec.color;  // Metals tint reflections
    scattered.emission = glm::vec3(0.0f, 0.0f, 0.0f);
    scattered.is_specular = true;
    scattered.scattered_ray = Ray{rec.intersectionPoint + rec.normal * 0.001f, reflected};
    scattered.pdf = 1.0f;
    
    // Only scatter if reflection is in same hemisphere as normal
    return glm::dot(reflected, rec.normal) > 0.0f;
}

FloatType DielectricShader::fresnel_schlick(FloatType cos_theta, FloatType ior_ratio) noexcept {
    FloatType r0 = (1.0f - ior_ratio) / (1.0f + ior_ratio);
    r0 = r0 * r0;
    FloatType cos_term = 1.0f - cos_theta;
    FloatType cos5 = cos_term * cos_term * cos_term * cos_term * cos_term;
    return r0 + (1.0f - r0) * cos5;
}

bool DielectricShader::scatter(
    const Ray& r_in,
    const HitRecord& rec,
    const Material& mat,
    ScatterRecord& scattered
) const {
    constexpr FloatType epsilon = 0.001f;
    
    // Determine if entering or exiting
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
    
    // Calculate refraction
    glm::vec3 refracted = glm::refract(r_in.direction, normal, ior_ratio);
    bool total_internal_reflection = (glm::length(refracted) < 0.0001f);
    
    if (total_internal_reflection) {
        // Total internal reflection
        glm::vec3 reflected = glm::reflect(r_in.direction, normal);
        scattered.attenuation = glm::vec3(1.0f, 1.0f, 1.0f);
        scattered.emission = glm::vec3(0.0f, 0.0f, 0.0f);
        scattered.is_specular = true;
        scattered.scattered_ray = Ray{rec.intersectionPoint + normal * epsilon, reflected};
        scattered.pdf = 1.0f;
        return true;
    }
    
    // Calculate Fresnel
    FloatType fresnel = fresnel_schlick(std::abs(cos_theta_i), ior_ratio);
    
    // Mix reflection and refraction based on Fresnel and transparency weight
    FloatType reflect_prob = fresnel * (1.0f - mat.tw);
    FloatType refract_prob = (1.0f - fresnel) * mat.tw;
    
    // Normalize probabilities
    FloatType total_prob = reflect_prob + refract_prob;
    if (total_prob < 0.01f) {
        return false;  // Absorbed
    }
    
    reflect_prob /= total_prob;
    
    // Stochastically choose reflection or refraction
    if (dist(rng) < reflect_prob) {
        // Reflection
        glm::vec3 reflected = glm::reflect(r_in.direction, normal);
        scattered.attenuation = glm::vec3(1.0f, 1.0f, 1.0f);
        scattered.emission = glm::vec3(0.0f, 0.0f, 0.0f);
        scattered.is_specular = true;
        scattered.scattered_ray = Ray{rec.intersectionPoint + normal * epsilon, reflected};
        scattered.pdf = reflect_prob;
        return true;
    } else {
        // Refraction
        scattered.attenuation = glm::vec3(1.0f, 1.0f, 1.0f);
        scattered.emission = glm::vec3(0.0f, 0.0f, 0.0f);
        scattered.is_specular = true;
        scattered.scattered_ray = Ray{rec.intersectionPoint - normal * epsilon, refracted};
        scattered.pdf = 1.0f - reflect_prob;
        return true;
    }
}
