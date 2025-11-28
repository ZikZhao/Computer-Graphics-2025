#pragma once
#include <memory>
#include <vector>
#include "shader.hpp"
#include "photon_map.hpp"
#include "world.hpp"

class RayTracer {
public:
    // Medium tracking for absorption
    struct MediumState {
        const Material* material = nullptr;
        glm::vec3 entry_point = glm::vec3(0.0f);
        FloatType entry_distance = 0.0f;
    };

public:
    static uint32_t PcgHash(uint32_t v) noexcept;
    static FloatType RandFloat(uint32_t& seed) noexcept;
    static FloatType ComputeLambertianLighting(const glm::vec3& normal, const glm::vec3& to_light,
                                               FloatType distance, FloatType intensity) noexcept;
    static FloatType ComputeSpecularLighting(const glm::vec3& normal, const glm::vec3& to_light,
                                             const glm::vec3& to_camera, FloatType distance,
                                             FloatType intensity, FloatType shininess) noexcept;
    static glm::vec3 SampleSphereHalton(int index, FloatType radius, const glm::vec3& center) noexcept;
    static FloatType Halton(int index, int base) noexcept;
    static ColourHDR ClampRadiance(const ColourHDR& c, FloatType max_luma) noexcept;

private:
    const World& world_;
    // Photon map for caustics
    std::unique_ptr<PhotonMap> photon_map_;

public:
    explicit RayTracer(const World& world);
    ColourHDR render_pixel(const Camera& cam, int x, int y, int width, int height,
                           bool soft_shadows, FloatType light_intensity, bool use_caustics = false, int sample_index = 0, uint32_t initial_seed = 1u) const noexcept;
    ColourHDR render_pixel_dof(const Camera& cam, int x, int y, int width, int height,
                               FloatType focal_distance, FloatType aperture_size, int samples,
                               bool soft_shadows, FloatType light_intensity, bool use_caustics = false) const noexcept;
    bool is_photon_map_ready() const noexcept { return photon_map_ && photon_map_->is_ready(); }

private:
    ColourHDR trace_ray(const glm::vec3& ro, const glm::vec3& rd, int depth,
                        const MediumState& medium, bool soft_shadows, FloatType light_intensity, bool use_caustics, int sample_index, const glm::vec3& throughput, uint32_t& rng) const noexcept;
    HitRecord hit(const glm::vec3& ro, const glm::vec3& rd) const noexcept;
    bool is_in_shadow_bvh(const glm::vec3& point, const glm::vec3& light_pos) const noexcept;
    glm::vec3 compute_transmittance_bvh(const glm::vec3& point, const glm::vec3& light_pos) const noexcept;
    FloatType compute_soft_shadow(const glm::vec3& point, const glm::vec3& light_center,
                                  FloatType light_radius, int num_samples, int start_index) const noexcept;
};
