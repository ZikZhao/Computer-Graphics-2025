#pragma once
#include "world.hpp"
#include <optional>

// Ray structure for shader computations
struct Ray {
    glm::vec3 origin;
    glm::vec3 direction;
};

// Scatter result - tells RayTracer what to do next
struct ScatterRecord {
    glm::vec3 attenuation;           // Color attenuation (albedo * transmittance)
    glm::vec3 emission;              // Self-emission (for lights)
    bool is_specular;                // Whether this is a specular reflection
    std::optional<Ray> scattered_ray; // Next ray to trace (none = absorbed)
    FloatType pdf;                   // Probability density for importance sampling
    
    ScatterRecord() noexcept 
        : attenuation(1.0f, 1.0f, 1.0f),
          emission(0.0f, 0.0f, 0.0f),
          is_specular(false),
          scattered_ray(std::nullopt),
          pdf(1.0f) {}
};

// Hit record alias
using HitRecord = RayTriangleIntersection;

// Abstract Shader base class
class Shader {
public:
    virtual ~Shader() = default;
    
    // Pure virtual: compute scatter based on material interaction
    // Returns false if ray is absorbed, true if scattered
    virtual bool scatter(
        const Ray& r_in,
        const HitRecord& rec,
        const Material& mat,
        ScatterRecord& scattered
    ) const = 0;
};

// Lambertian (diffuse) shader
class LambertianShader : public Shader {
public:
    bool scatter(
        const Ray& r_in,
        const HitRecord& rec,
        const Material& mat,
        ScatterRecord& scattered
    ) const override;
    
private:
    // Generate random direction in hemisphere around normal
    static glm::vec3 random_in_hemisphere(const glm::vec3& normal) noexcept;
};

// Metal (reflective) shader
class MetalShader : public Shader {
public:
    bool scatter(
        const Ray& r_in,
        const HitRecord& rec,
        const Material& mat,
        ScatterRecord& scattered
    ) const override;
};

// Dielectric (glass/transparent) shader
class DielectricShader : public Shader {
public:
    bool scatter(
        const Ray& r_in,
        const HitRecord& rec,
        const Material& mat,
        ScatterRecord& scattered
    ) const override;
    
private:
    // Fresnel reflectance using Schlick's approximation
    static FloatType fresnel_schlick(FloatType cos_theta, FloatType ior_ratio) noexcept;
};
