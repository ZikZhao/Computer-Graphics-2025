#include <numeric>
#include <algorithm>
#include <functional>
#include <limits>
#include <numbers>
#include <iostream>
#include "photon_map.hpp"
#include "raytracer.hpp"

constexpr uint32_t RayTracer::PcgHash(uint32_t v) noexcept {
    uint32_t state = v * 747796405u + 2891336453u;
    uint32_t word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return word ^ (word >> 22u);
}

FloatType RayTracer::RandFloat(uint32_t& seed) noexcept {
    seed = PcgHash(seed);
    return static_cast<FloatType>((seed >> 8) & 0x00FFFFFFu) / 16777216.0f;
}

RayTracer::RayTracer(const World& world)
    : world_(world) {
    // Initialize photon map (starts background thread automatically)
    std::cout << "RayTracer: Initializing photon map for caustics..." << std::endl;
    photon_map_ = std::make_unique<PhotonMap>(world);
    std::cout << "RayTracer: Photon map initialized (computing in background)" << std::endl;
}

ColourHDR RayTracer::render_pixel(const Camera& cam, int x, int y, int width, int height,
                                 bool soft_shadows, bool use_caustics, int sample_index, uint32_t initial_seed) const noexcept {
    FloatType jitter_x = RandFloat(initial_seed) - 0.5f;
    FloatType jitter_y = RandFloat(initial_seed) - 0.5f;
    FloatType u = (static_cast<FloatType>(x) + 0.5f + jitter_x) / static_cast<FloatType>(width);
    FloatType v = (static_cast<FloatType>(y) + 0.5f + jitter_y) / static_cast<FloatType>(height);
    auto [ray_origin, ray_dir] = cam.generate_ray_uv(u, v, width, height, static_cast<double>(width) / height);
    return trace_ray(ray_origin, ray_dir, 0, MediumState{}, soft_shadows, use_caustics, sample_index, glm::vec3(1.0f), initial_seed);
}

static glm::vec2 SampleDiskConcentric(FloatType u1, FloatType u2) noexcept {
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

ColourHDR RayTracer::render_pixel_dof(const Camera& cam, int x, int y, int width, int height,
                                      FloatType focal_distance, FloatType aperture_size, int samples,
                                      bool soft_shadows, bool use_caustics) const noexcept {
    ColourHDR accumulated_color(0.0f, 0.0f, 0.0f);
    double aspect_ratio = static_cast<double>(width) / height;
    
    for (int sample = 0; sample < samples; ++sample) {
        uint32_t lens_seed = static_cast<uint32_t>((y * width + x) + sample * 747796405u) | 1u;
        FloatType u1 = RandFloat(lens_seed);
        FloatType u2 = RandFloat(lens_seed);
        glm::vec2 lens_sample = SampleDiskConcentric(u1, u2) * aperture_size;
        
        uint32_t jitter_seed = static_cast<uint32_t>((y * width + x) + sample * 1597334677u) | 1u;
        FloatType u0 = (static_cast<FloatType>(x) + RandFloat(jitter_seed)) / static_cast<FloatType>(width);
        FloatType v0 = (static_cast<FloatType>(y) + RandFloat(jitter_seed)) / static_cast<FloatType>(height);
        auto [center_origin, center_dir] = cam.generate_ray_uv(u0, v0, width, height, aspect_ratio);
        glm::vec3 focal_point = center_origin + center_dir * focal_distance;
        
        glm::mat3 cam_orientation = cam.orientation();
        glm::vec3 lens_offset = cam_orientation[0] * lens_sample.x + cam_orientation[1] * lens_sample.y;
        glm::vec3 ray_origin = center_origin + lens_offset;
        glm::vec3 ray_dir = glm::normalize(focal_point - ray_origin);
        
        uint32_t seed = static_cast<uint32_t>((y * width + x) + sample * 2891336453u) | 1u;
        accumulated_color = accumulated_color + trace_ray(ray_origin, ray_dir, 0, MediumState{}, soft_shadows, use_caustics, 0, glm::vec3(1.0f), seed);
    }
    
    return ColourHDR{
        .red = accumulated_color.red / static_cast<FloatType>(samples),
        .green = accumulated_color.green / static_cast<FloatType>(samples),
        .blue = accumulated_color.blue / static_cast<FloatType>(samples)
    };
}


ColourHDR RayTracer::trace_ray(const glm::vec3& ray_origin, const glm::vec3& ray_dir, int depth,
                               const MediumState& medium, bool soft_shadows, bool use_caustics, int sample_index, const glm::vec3& throughput, uint32_t& rng) const noexcept {
    constexpr int ABS_MAX_DEPTH = 20;
    if (depth >= ABS_MAX_DEPTH) {
        if (world_.env_map().is_loaded()) {
            return world_.env_map().sample(ray_dir);
        }
        return ColourHDR{ .red = 0.0f, .green = 0.0f, .blue = 0.0f };
    }
    
    
    HitRecord intersection = hit(ray_origin, ray_dir);
    
    // Apply medium absorption if inside transparent medium
    ColourHDR medium_absorption{1.0f, 1.0f, 1.0f};
    if (medium.material != nullptr && intersection.triangleIndex != static_cast<std::size_t>(-1)) {
        FloatType distance_in_medium = intersection.distanceFromCamera;
        glm::vec3 effective_sigma_a;
        
        if (glm::length(medium.material->sigma_a) > 0.0f) {
            effective_sigma_a = medium.material->sigma_a;
        } else if (medium.material->td > 0.0f) {
            effective_sigma_a = glm::vec3(
                -std::log(std::max(medium.material->base_color.r, 0.001f)) / medium.material->td,
                -std::log(std::max(medium.material->base_color.g, 0.001f)) / medium.material->td,
                -std::log(std::max(medium.material->base_color.b, 0.001f)) / medium.material->td
            );
        } else {
            effective_sigma_a = glm::vec3(0.0f);
        }
        
        medium_absorption = ColourHDR(
            std::exp(-effective_sigma_a.r * distance_in_medium),
            std::exp(-effective_sigma_a.g * distance_in_medium),
            std::exp(-effective_sigma_a.b * distance_in_medium)
        );
    }
    
    if (intersection.triangleIndex == static_cast<std::size_t>(-1)) {
        ColourHDR env_color;
        if (world_.env_map().is_loaded()) {
            env_color = world_.env_map().sample(ray_dir);
        } else {
            env_color = ColourHDR{ .red = 0.0f, .green = 0.0f, .blue = 0.0f };
        }
        {
            ColourHDR out = env_color * medium_absorption;
            if (depth > 0) out = ClampRadiance(out, 10.0f);
            return out;
        }
    }
    
    const std::vector<Face>& all_faces = world_.all_faces();
    const Face& face = all_faces[intersection.triangleIndex];

    {
        glm::vec3 Le = face.material.emission;
        if (glm::length(Le) > 1e-6f) {
            ColourHDR out = ColourHDR{ .red = Le.r, .green = Le.g, .blue = Le.b } * medium_absorption;
            if (depth > 0) out = ClampRadiance(out, 10.0f);
            return out;
        }
    }
    const auto& area_lights = world_.area_lights();
    
    if (face.material.tw > 0.0f) {
        constexpr FloatType epsilon = 0.001f;
        glm::vec3 normal = intersection.normal;
        if (glm::length(normal) < 0.001f) normal = face.face_normal;
        FloatType cos_theta_i = glm::dot(-ray_dir, normal);
        bool entering = cos_theta_i > 0.0f;
        
        FloatType ior_from = entering ? 1.0f : face.material.ior;
        FloatType ior_to = entering ? face.material.ior : 1.0f;
        FloatType ior_ratio = ior_from / ior_to;
        
        if (!entering) {
            normal = -normal;
            cos_theta_i = -cos_theta_i;
        }
        
        glm::vec3 refracted_dir = glm::refract(ray_dir, normal, ior_ratio);
        bool total_internal_reflection = (glm::length(refracted_dir) < 0.0001f);
        
        if (total_internal_reflection) {
            glm::vec3 reflected_dir = glm::reflect(ray_dir, normal);
            // Epsilon origin shift: nudge the spawn point along the geometric
            // normal to avoid self-intersections caused by floating-point
            // imprecision (shadow acne / z-fighting).
            glm::vec3 offset_origin = intersection.intersectionPoint + normal * epsilon;
            glm::vec3 next_tp = throughput;
            ColourHDR reflected = trace_ray(offset_origin, reflected_dir, depth + 1, medium, soft_shadows, use_caustics, sample_index, next_tp, rng);
            ColourHDR out = reflected * medium_absorption;
            if (depth > 0) out = ClampRadiance(out, 10.0f);
            return out;
        }
        
        // Fresnel
        FloatType r0 = (1.0f - ior_ratio) / (1.0f + ior_ratio);
        r0 = r0 * r0;
        FloatType cos_term = 1.0f - std::abs(cos_theta_i);
        FloatType cos5 = cos_term * cos_term * cos_term * cos_term * cos_term;
        FloatType fresnel = r0 + (1.0f - r0) * cos5;
        
        FloatType reflect_weight = fresnel * (1.0f - face.material.tw);
        FloatType refract_weight = (1.0f - fresnel) * face.material.tw;
        
        ColourHDR result_color{0.0f, 0.0f, 0.0f};
        
        if (reflect_weight > 0.01f) {
            glm::vec3 reflected_dir = glm::reflect(ray_dir, normal);
            // Epsilon origin shift: prevent the reflected ray from re-hitting
            // the same surface due to numerical error.
            glm::vec3 offset_origin = intersection.intersectionPoint + normal * epsilon;
            glm::vec3 next_tp = throughput;
            ColourHDR reflected_color = trace_ray(offset_origin, reflected_dir, depth + 1, medium, soft_shadows, use_caustics, sample_index, next_tp, rng);
            result_color = result_color + reflected_color * reflect_weight;
        }
        
        if (refract_weight > 0.01f) {
            // For transmitted rays, offset opposite the surface normal to start
            // the ray just inside the medium.
            glm::vec3 offset_origin = intersection.intersectionPoint - normal * epsilon;
            MediumState new_medium;
            if (entering) {
                new_medium.material = &face.material;
                new_medium.entry_point = intersection.intersectionPoint;
                new_medium.entry_distance = 0.0f;
            } else {
                new_medium.material = nullptr;
                new_medium.entry_point = glm::vec3(0.0f);
                new_medium.entry_distance = 0.0f;
            }
            glm::vec3 next_tp = throughput;
            ColourHDR refracted_color = trace_ray(offset_origin, refracted_dir, depth + 1, new_medium, soft_shadows, use_caustics, sample_index, next_tp, rng);
            result_color = result_color + refracted_color * refract_weight;
        }
        
        FloatType direct_weight = 1.0f - reflect_weight - refract_weight;
        if (direct_weight > 0.01f) {
            ColourHDR hdr_colour = ColourHDR{intersection.color.r, intersection.color.g, intersection.color.b};
            FloatType ambient = 0.025f;
            result_color = result_color + hdr_colour * ambient * direct_weight;
        }
        
        {
            ColourHDR out = result_color * medium_absorption;
            if (depth > 0) out = ClampRadiance(out, 10.0f);
            return out;
        }
    }
    
    glm::vec3 to_camera_hit = -ray_dir;
    
    ColourHDR shadow_color;
    bool backface_view_gate = false;
    {
        backface_view_gate = false;
        shadow_color = ColourHDR{ .red = 1.0f, .green = 1.0f, .blue = 1.0f };
    }
    
    ColourHDR hdr_colour = ColourHDR{ .red = intersection.color.r, .green = intersection.color.g, .blue = intersection.color.b };
    FloatType ambient = 0.025f;
    FloatType specular = 0.0f;
    ColourHDR diffuse_component{0.0f, 0.0f, 0.0f};
    ColourHDR specular_component{0.0f, 0.0f, 0.0f};
    
    FloatType w = 1.0f - intersection.u - intersection.v;
    if (!backface_view_gate) {
        glm::vec3 n_shade = (face.material.shading == Material::Shading::FLAT)
            ? face.face_normal
            : [&](){
                auto fetch_normal = [&](int idx) -> glm::vec3 {
                    std::uint32_t ni = face.vn_indices[idx];
                    if (ni != std::numeric_limits<std::uint32_t>::max() && ni < world_.all_vertex_normals().size()) return world_.all_vertex_normals()[ni];
                    std::uint32_t vi = face.v_indices[idx];
                    if (vi < world_.all_vertex_normals_by_vertex().size() && glm::length(world_.all_vertex_normals_by_vertex()[vi]) > 0.001f) return world_.all_vertex_normals_by_vertex()[vi];
                    return face.face_normal;
                };
                glm::vec3 n0 = fetch_normal(0);
                glm::vec3 n1 = fetch_normal(1);
                glm::vec3 n2 = fetch_normal(2);
                return glm::normalize(w * n0 + intersection.u * n1 + intersection.v * n2);
            }();
        specular = 0.0f;
        if (!area_lights.empty()) {
            glm::vec3 diffuse_rgb_accum(0.0f);
            for (const Face* lf : area_lights) {
                glm::vec3 e0 = world_.all_vertices()[lf->v_indices[1]] - world_.all_vertices()[lf->v_indices[0]];
                glm::vec3 e1 = world_.all_vertices()[lf->v_indices[2]] - world_.all_vertices()[lf->v_indices[0]];
                FloatType area = 0.5f * glm::length(glm::cross(e0, e1));
                if (area < 1e-6f) continue;
                FloatType u1 = RandFloat(rng);
                FloatType u2 = RandFloat(rng);
                FloatType su = std::sqrt(u1);
                FloatType b0 = 1.0f - su;
                FloatType b1 = su * (1.0f - u2);
                FloatType b2 = su * u2;
                glm::vec3 light_p = world_.all_vertices()[lf->v_indices[0]] + b1 * e0 + b2 * e1;
                glm::vec3 shadow_origin = intersection.intersectionPoint + n_shade * 1e-4f;
                glm::vec3 to_light = light_p - shadow_origin;
                if (glm::dot(face.face_normal, to_light) <= 0.0f) continue;
                FloatType dist = glm::length(to_light);
                if (dist < 1e-4f) continue;
                glm::vec3 L = glm::normalize(to_light);
                glm::vec3 n_geom = face.face_normal;
                if (glm::dot(n_geom, L) < 0.0f) n_geom = -n_geom;
                if (glm::dot(n_geom, to_camera_hit) < 0.0f) continue;
                glm::vec3 n_light = glm::normalize(lf->face_normal);
                FloatType cos_surf = std::max(0.0f, glm::dot(n_shade, L));
                FloatType cos_light = std::max(0.0f, glm::dot(n_light, -L));
                glm::vec3 transmittance = compute_transmittance_bvh(shadow_origin, light_p);
                glm::vec3 vis = transmittance;
                glm::vec3 Le = lf->material.emission;
                FloatType G = (cos_surf * cos_light) / (dist * dist + 1e-6f);
                // Monte Carlo weighting: we sample points uniformly on the area
                // light (PDF = 1/area). Multiplying by 'area' is the 1/PDF term
                // that keeps the estimator unbiased. 'G' encodes geometry terms.
                FloatType GA = G * area;
                glm::vec3 albedo = glm::vec3(hdr_colour.red, hdr_colour.green, hdr_colour.blue);
                FloatType inv_pi = 1.0f / static_cast<FloatType>(std::numbers::pi);
                diffuse_rgb_accum += (albedo * ((Le * vis) * (GA * inv_pi)));
                glm::vec3 halfway = glm::normalize(L + to_camera_hit);
                FloatType cos_alpha = std::max(0.0f, glm::dot(n_shade, halfway));
                // Luminance coefficients per ITU-R BT.709 (sRGB/Rec.709).
                FloatType le_lum = 0.2126f * Le.r + 0.7152f * Le.g + 0.0722f * Le.b;
                FloatType vis_lum = 0.2126f * vis.r + 0.7152f * vis.g + 0.0722f * vis.b;
                specular += le_lum * std::pow(cos_alpha, face.material.shininess) * area * vis_lum / (dist * dist + 1e-6f);
            }
                    diffuse_component = ColourHDR{ .red = diffuse_rgb_accum.r, .green = diffuse_rgb_accum.g, .blue = diffuse_rgb_accum.b };
        } else {
                    diffuse_component = ColourHDR{ .red = 0.0f, .green = 0.0f, .blue = 0.0f };
        }
    }
    
    ColourHDR ambient_component = hdr_colour * ambient;
    if (area_lights.empty()) {
        diffuse_component = ColourHDR{ .red = 0.0f, .green = 0.0f, .blue = 0.0f };
    }
    specular_component = ColourHDR{ .red = specular, .green = specular, .blue = specular };
    
    ColourHDR direct_lighting;
    if (face.material.metallic > 0.0f) {
        direct_lighting = ambient_component + diffuse_component + hdr_colour * specular_component.red;
    } else {
        direct_lighting = ambient_component + diffuse_component + specular_component;
    }
    
    if (face.material.metallic > 0.0f) {
        glm::vec3 use_normal = (face.material.shading == Material::Shading::FLAT)
            ? face.face_normal
            : intersection.normal;
        if (glm::dot(use_normal, -ray_dir) < 0.0f) {
            use_normal = -use_normal;
        }
        glm::vec3 reflected_dir = glm::reflect(ray_dir, use_normal);
        constexpr FloatType epsilon = 0.001f;
        // Epsilon origin shift: start the glossy reflection slightly above the
        // surface to avoid self-hit.
        glm::vec3 offset_origin = intersection.intersectionPoint + use_normal * epsilon;
        glm::vec3 next_tp = throughput * glm::vec3(hdr_colour.red, hdr_colour.green, hdr_colour.blue);
        FloatType p = std::max(next_tp.x, std::max(next_tp.y, next_tp.z));
        p = std::clamp(p, 0.05f, 1.0f);
        // Russian roulette: divide by survival probability 'p' so the expected
        // contribution remains unchanged (unbiased estimator).
        glm::vec3 compensated_tp = next_tp / p;
        ColourHDR reflected_color = trace_ray(offset_origin, reflected_dir, depth + 1, medium, soft_shadows, use_caustics, sample_index, compensated_tp, rng);
        ColourHDR metallic_reflection = ColourHDR{
            .red   = reflected_color.red   * hdr_colour.red,
            .green = reflected_color.green * hdr_colour.green,
            .blue  = reflected_color.blue  * hdr_colour.blue
        };
        {
            ColourHDR out = direct_lighting * (1.0f - face.material.metallic) + metallic_reflection * face.material.metallic;
            if (depth > 0) out = ClampRadiance(out, 10.0f);
            return out;
        }
    }
    
    ColourHDR caustics_contribution(0.0f, 0.0f, 0.0f);
    if (use_caustics && photon_map_ && photon_map_->is_ready()) {
        // Estimate caustic radiance at this point
        caustics_contribution = photon_map_->estimate_caustic(&face, intersection.intersectionPoint, intersection.normal, PhotonMap::CausticSearchRadius);
        
        // Modulate caustics by surface albedo (diffuse color) to get final color
        caustics_contribution = ColourHDR{
            .red   = caustics_contribution.red   * hdr_colour.red,
            .green = caustics_contribution.green * hdr_colour.green,
            .blue  = caustics_contribution.blue  * hdr_colour.blue
        };
        
        // Scale caustics intensity to 50% of current value
        caustics_contribution = caustics_contribution * 0.25f;
    }
    
    {
        ColourHDR out = (direct_lighting + caustics_contribution) * medium_absorption;
        if (depth > 0) out = ClampRadiance(out, 10.0f);
        return out;
    }
}

HitRecord RayTracer::hit(const glm::vec3& ro, const glm::vec3& rd) const noexcept {
    return world_.accelerator().intersect(ro, rd, world_.all_faces());
}

glm::vec3 RayTracer::compute_transmittance_bvh(const glm::vec3& point, const glm::vec3& light_pos) const noexcept {
    return world_.accelerator().transmittance(point, light_pos, world_.all_faces());
}

glm::vec3 RayTracer::SampleSphereHalton(int index, FloatType radius, const glm::vec3& center) noexcept {
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

constexpr FloatType RayTracer::Halton(int index, int base) noexcept {
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

constexpr ColourHDR RayTracer::ClampRadiance(const ColourHDR& c, FloatType max_component) noexcept {
    FloatType r = std::min(c.red, max_component);
    FloatType g = std::min(c.green, max_component);
    FloatType b = std::min(c.blue, max_component);
    return ColourHDR{ .red = r, .green = g, .blue = b };
}
