#pragma once
#include "world.hpp"
#include "shader.hpp"
#include <memory>
#include <vector>

class RayTracer {
public:
    // BVH structures (moved from Renderer)
    struct AABB {
        glm::vec3 min;
        glm::vec3 max;
    };
    
    struct BVHNode {
        AABB box;
        int left;
        int right;
        int start;
        int count;
    };
    
    // Medium tracking for absorption
    struct MediumState {
        const Material* material = nullptr;  // nullptr = air/vacuum
        glm::vec3 entry_point = glm::vec3(0.0f);
        FloatType entry_distance = 0.0f;
    };

private:
    const World& world_;
    
    // BVH data (immutable after construction)
    const std::vector<int> bvh_tri_indices_;
    const std::vector<BVHNode> bvh_nodes_;
    
    // Shader instances (stateless singletons)
    std::unique_ptr<Shader> shader_lambertian_;
    std::unique_ptr<Shader> shader_metal_;
    std::unique_ptr<Shader> shader_dielectric_;
    
public:
    explicit RayTracer(const World& world);
    
    // Core rendering entry point
    ColourHDR render_pixel(const Camera& cam, int x, int y, int width, int height, 
                           bool soft_shadows, FloatType light_intensity) const noexcept;
    
    // Depth of field rendering
    ColourHDR render_pixel_dof(const Camera& cam, int x, int y, int width, int height,
                               FloatType focal_distance, FloatType aperture_size, int samples,
                               bool soft_shadows, FloatType light_intensity) const noexcept;
    
private:
    // Ray tracing core (no longer contains material logic)
    ColourHDR trace_ray(const glm::vec3& ro, const glm::vec3& rd, int depth, 
                        const MediumState& medium, bool soft_shadows, FloatType light_intensity) const noexcept;
    
    // BVH intersection
    HitRecord hit(const glm::vec3& ro, const glm::vec3& rd) const noexcept;
    
    // Shadow and lighting helpers
    bool is_in_shadow_bvh(const glm::vec3& point, const glm::vec3& light_pos) const noexcept;
    glm::vec3 compute_transmittance_bvh(const glm::vec3& point, const glm::vec3& light_pos) const noexcept;
    FloatType compute_soft_shadow(const glm::vec3& point, const glm::vec3& light_center, 
                                   FloatType light_radius, int num_samples) const noexcept;
    
    // Lighting calculations
    static FloatType compute_lambertian_lighting(const glm::vec3& normal, const glm::vec3& to_light, 
                                                  FloatType distance, FloatType intensity) noexcept;
    static FloatType compute_specular_lighting(const glm::vec3& normal, const glm::vec3& to_light, 
                                                const glm::vec3& to_camera, FloatType distance, 
                                                FloatType intensity, FloatType shininess) noexcept;
    
    // BVH construction and traversal
    static std::pair<std::vector<int>, std::vector<BVHNode>> build_bvh(const std::vector<Face>& faces) noexcept;
    static bool intersect_aabb(const glm::vec3& ro, const glm::vec3& rd, const AABB& box, FloatType tmax) noexcept;
    
    // Sampling utilities
    static glm::vec3 sample_sphere_halton(int index, FloatType radius, const glm::vec3& center) noexcept;
    static FloatType halton(int index, int base) noexcept;
};
