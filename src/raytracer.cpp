#include "raytracer.hpp"

#include <algorithm>
#include <functional>
#include <iostream>
#include <limits>
#include <numbers>
#include <numeric>

#include "photon_map.hpp"

RayTracer::RayTracer(const World& world) : world_(world) {
    photon_map_ = std::make_unique<PhotonMap>(world);
}

ColourHDR RayTracer::render_pixel(
    const Camera& cam,
    int x,
    int y,
    int width,
    int height,
    bool use_caustics,
    std::uint32_t initial_seed
) const noexcept {
    // Anti-aliasing: Jitter the pixel coordinate to sample within the pixel area
    FloatType jitter_x = RandFloat(initial_seed) - 0.5f;
    FloatType jitter_y = RandFloat(initial_seed) - 0.5f;
    FloatType u = (static_cast<FloatType>(x) + 0.5f + jitter_x) / static_cast<FloatType>(width);
    FloatType v = (static_cast<FloatType>(y) + 0.5f + jitter_y) / static_cast<FloatType>(height);

    auto [ray_origin, ray_dir] = cam.generate_ray_uv(u, v, width, height);

    return trace_ray(
        ray_origin, ray_dir, 0, MediumState{}, use_caustics, glm::vec3(1.0f), initial_seed
    );
}

ColourHDR RayTracer::render_pixel_normal(const Camera& cam, int x, int y, int width, int height)
    const noexcept {
    FloatType u = (static_cast<FloatType>(x) + 0.5f) / static_cast<FloatType>(width);
    FloatType v = (static_cast<FloatType>(y) + 0.5f) / static_cast<FloatType>(height);

    auto [ray_origin, ray_dir] = cam.generate_ray_uv(u, v, width, height);

    RayTriangleIntersection intersection = hit(ray_origin, ray_dir);

    if (intersection.triangleIndex != static_cast<std::size_t>(-1)) {
        // Map normal from [-1, 1] to [0, 1] for color display
        glm::vec3 n = intersection.normal * 0.5f + 0.5f;
        return ColourHDR{n.x, n.y, n.z};
    }

    // Miss: return environment or black
    if (world_.env_map_.is_loaded()) {
        return world_.env_map_.sample(ray_dir);
    }
    return ColourHDR{0.0f, 0.0f, 0.0f};
}

ColourHDR RayTracer::render_pixel_dof(
    const Camera& cam,
    int x,
    int y,
    int width,
    int height,
    FloatType focal_distance,
    FloatType aperture_size,
    int samples,
    bool use_caustics
) const noexcept {
    ColourHDR accumulated_color(0.0f, 0.0f, 0.0f);

    for (int sample = 0; sample < samples; ++sample) {
        // Sample thin lens aperture (concentric disk)
        std::uint32_t lens_seed =
            static_cast<std::uint32_t>((y * width + x) + sample * 747796405u) | 1u;
        FloatType u1 = RandFloat(lens_seed);
        FloatType u2 = RandFloat(lens_seed);
        glm::vec2 lens_sample = SampleDiskConcentric(u1, u2) * aperture_size;

        // Jitter within the pixel for anti-aliasing
        std::uint32_t jitter_seed =
            static_cast<std::uint32_t>((y * width + x) + sample * 1597334677u) | 1u;
        FloatType u0 =
            (static_cast<FloatType>(x) + RandFloat(jitter_seed)) / static_cast<FloatType>(width);
        FloatType v0 =
            (static_cast<FloatType>(y) + RandFloat(jitter_seed)) / static_cast<FloatType>(height);

        // Primary ray through the pinhole
        auto [center_origin, center_dir] = cam.generate_ray_uv(u0, v0, width, height);

        // Focal plane target for the lens sample
        glm::vec3 focal_point = center_origin + center_dir * focal_distance;

        // Offset origin on the lens and aim at focal point
        glm::mat3 cam_orientation = cam.orientation();
        glm::vec3 lens_offset =
            cam_orientation[0] * lens_sample.x + cam_orientation[1] * lens_sample.y;
        glm::vec3 ray_origin = center_origin + lens_offset;

        // New ray direction toward focal target
        glm::vec3 ray_dir = glm::normalize(focal_point - ray_origin);

        std::uint32_t seed =
            static_cast<std::uint32_t>((y * width + x) + sample * 2891336453u) | 1u;
        accumulated_color =
            accumulated_color +
            trace_ray(ray_origin, ray_dir, 0, MediumState{}, use_caustics, glm::vec3(1.0f), seed);
    }

    return ColourHDR{
        .red = accumulated_color.red / static_cast<FloatType>(samples),
        .green = accumulated_color.green / static_cast<FloatType>(samples),
        .blue = accumulated_color.blue / static_cast<FloatType>(samples)
    };
}

ColourHDR RayTracer::trace_ray(
    const glm::vec3& ray_origin,
    const glm::vec3& ray_dir,
    int depth,
    const MediumState& medium,
    bool use_caustics,
    const glm::vec3& throughput,
    std::uint32_t& rng
) const noexcept {
    constexpr int ABS_MAX_DEPTH = 64;
    if (depth >= ABS_MAX_DEPTH) {
        return ColourHDR{.red = 0.0f, .green = 0.0f, .blue = 0.0f};
    }

    // Scene intersection
    RayTriangleIntersection intersection = hit(ray_origin, ray_dir);

    // Medium absorption (Beer's Law): attenuate radiance by traveled distance
    ColourHDR medium_absorption{1.0f, 1.0f, 1.0f};
    if (medium.material != nullptr && intersection.triangleIndex != static_cast<std::size_t>(-1)) {
        FloatType distance_in_medium = intersection.distanceFromCamera;
        glm::vec3 effective_sigma_a;

        if (glm::length(medium.material->sigma_a) > 0.0f) {
            effective_sigma_a = medium.material->sigma_a;
        } else if (medium.material->td > 0.0f) {
            // Derive absorption from transmission distance (td) and base color
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

    // Environment mapping
    if (intersection.triangleIndex == static_cast<std::size_t>(-1)) {
        ColourHDR env_color;
        if (world_.env_map_.is_loaded()) {
            env_color = world_.env_map_.sample(ray_dir);
        } else {
            env_color = ColourHDR{.red = 0.0f, .green = 0.0f, .blue = 0.0f};
        }
        ColourHDR out = env_color * medium_absorption;
        if (depth > 0) out = ClampRadiance(out, 10.0f);
        return out;
    }

    const std::vector<Face>& all_faces = world_.all_faces_;
    const Face& face = all_faces[intersection.triangleIndex];

    // Emissive contribution from area lights
    glm::vec3 Le = face.material.emission;
    if (glm::length(Le) > 1e-6f) {
        ColourHDR out = ColourHDR{.red = Le.r, .green = Le.g, .blue = Le.b} * medium_absorption;
        if (depth > 0) out = ClampRadiance(out, 10.0f);
        return out;
    }
    const auto& area_lights = world_.emissive_faces_;

    // Dielectric surface: refraction/reflection with Fresnel (Schlick)
    if (face.material.tw > 0.0f) {
        constexpr FloatType epsilon = 0.001f;
        glm::vec3 normal = intersection.normal;
        if (glm::length(normal) < 0.001f) normal = face.face_normal;

        FloatType cos_theta_i = glm::dot(-ray_dir, normal);
        bool entering = cos_theta_i > 0.0f;

        // Determine IOR (Index of Refraction) ratio
        FloatType ior_from = entering ? 1.0f : face.material.ior;
        FloatType ior_to = entering ? face.material.ior : 1.0f;
        FloatType ior_ratio = ior_from / ior_to;

        if (!entering) {
            normal = -normal;
            cos_theta_i = -cos_theta_i;
        }

        // Refract
        glm::vec3 refracted_dir = glm::refract(ray_dir, normal, ior_ratio);
        bool total_internal_reflection = (glm::length(refracted_dir) < 0.0001f);

        if (total_internal_reflection) {
            // Total internal reflection: 100% reflective branch
            glm::vec3 reflected_dir = glm::reflect(ray_dir, normal);
            glm::vec3 offset_origin = intersection.intersectionPoint + normal * epsilon;
            glm::vec3 next_tp = throughput;
            ColourHDR reflected = trace_ray(
                offset_origin, reflected_dir, depth + 1, medium, use_caustics, next_tp, rng
            );
            ColourHDR out = reflected * medium_absorption;
            if (depth > 0) out = ClampRadiance(out, 10.0f);
            return out;
        }

        // Fresnel (Schlick's approximation)
        FloatType r0 = (1.0f - ior_ratio) / (1.0f + ior_ratio);
        r0 = r0 * r0;
        FloatType cos_term = 1.0f - std::abs(cos_theta_i);
        FloatType cos5 = cos_term * cos_term * cos_term * cos_term * cos_term;
        FloatType fresnel = r0 + (1.0f - r0) * cos5;

        FloatType reflect_weight = fresnel * (1.0f - face.material.tw);
        FloatType refract_weight = (1.0f - fresnel) * face.material.tw;

        ColourHDR result_color{0.0f, 0.0f, 0.0f};

        // Recursive reflection branch
        if (reflect_weight > 0.01f) {
            glm::vec3 reflected_dir = glm::reflect(ray_dir, normal);
            glm::vec3 offset_origin = intersection.intersectionPoint + normal * epsilon;
            glm::vec3 next_tp = throughput;
            ColourHDR reflected_color = trace_ray(
                offset_origin, reflected_dir, depth + 1, medium, use_caustics, next_tp, rng
            );
            result_color = result_color + reflected_color * reflect_weight;
        }

        // Recursive refraction branch
        if (refract_weight > 0.01f) {
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
            ColourHDR refracted_color = trace_ray(
                offset_origin, refracted_dir, depth + 1, new_medium, use_caustics, next_tp, rng
            );
            result_color = result_color + refracted_color * refract_weight;
        }

        // Residual direct term when not fully reflective/refractive
        FloatType direct_weight = 1.0f - reflect_weight - refract_weight;
        if (direct_weight > 0.01f) {
            ColourHDR hdr_colour =
                ColourHDR{intersection.color.r, intersection.color.g, intersection.color.b};
            FloatType ambient = 0.025f;
            result_color = result_color + hdr_colour * ambient * direct_weight;
        }

        ColourHDR out = result_color * medium_absorption;
        if (depth > 0) out = ClampRadiance(out, 10.0f);
        return out;
    }

    // Direct lighting: area light sampling with shadow transmittance
    glm::vec3 to_camera_hit = -ray_dir;

    ColourHDR shadow_color;
    bool backface_view_gate = false;
    backface_view_gate = false;
    shadow_color = ColourHDR{.red = 1.0f, .green = 1.0f, .blue = 1.0f};

    ColourHDR hdr_colour = ColourHDR{
        .red = intersection.color.r, .green = intersection.color.g, .blue = intersection.color.b
    };
    FloatType ambient = 0.025f;
    FloatType specular = 0.0f;
    ColourHDR diffuse_component{0.0f, 0.0f, 0.0f};
    ColourHDR specular_component{0.0f, 0.0f, 0.0f};

    FloatType w = 1.0f - intersection.u - intersection.v;
    if (!backface_view_gate) {
        // Use the normal from intersection which already includes:
        // 1. Interpolated vertex normals (for smooth surfaces)
        // 2. Normal mapping perturbation (if normal map is present)
        // This applies regardless of shading mode when a normal map exists
        glm::vec3 n_shade = intersection.normal;

        specular = 0.0f;
        if (!area_lights.empty()) {
            glm::vec3 diffuse_rgb_accum(0.0f);
            // Monte Carlo area-light sampling
            for (const Face* lf : area_lights) {
                // Sample a point on the emitter
                glm::vec3 e0 =
                    world_.all_vertices_[lf->v_indices[1]] - world_.all_vertices_[lf->v_indices[0]];
                glm::vec3 e1 =
                    world_.all_vertices_[lf->v_indices[2]] - world_.all_vertices_[lf->v_indices[0]];
                FloatType area = 0.5f * glm::length(glm::cross(e0, e1));
                if (area < 1e-6f) continue;

                FloatType u1 = RandFloat(rng);
                FloatType u2 = RandFloat(rng);
                FloatType su = std::sqrt(u1);
                FloatType b0 = 1.0f - su;
                FloatType b1 = su * (1.0f - u2);
                FloatType b2 = su * u2;
                glm::vec3 light_p = world_.all_vertices_[lf->v_indices[0]] + b1 * e0 + b2 * e1;

                // Shadow ray toward sampled point
                glm::vec3 shadow_origin = intersection.intersectionPoint + n_shade * 1e-4f;
                glm::vec3 to_light = light_p - shadow_origin;

                if (glm::dot(face.face_normal, to_light) <= 0.0f)
                    continue;  // Light is behind surface
                FloatType dist = glm::length(to_light);
                if (dist < 1e-4f) continue;
                glm::vec3 L = glm::normalize(to_light);

                glm::vec3 n_geom = face.face_normal;
                if (glm::dot(n_geom, L) < 0.0f) n_geom = -n_geom;
                if (glm::dot(n_geom, to_camera_hit) < 0.0f) continue;

                glm::vec3 n_light = glm::normalize(lf->face_normal);
                FloatType cos_surf = std::max(0.0f, glm::dot(n_shade, L));
                FloatType cos_light = std::max(0.0f, glm::dot(n_light, -L));

                // Visibility via BVH transmittance
                glm::vec3 transmittance = compute_transmittance_bvh(shadow_origin, light_p);
                glm::vec3 vis = transmittance;

                glm::vec3 Le = lf->material.emission;
                FloatType G = (cos_surf * cos_light) / (dist * dist + 1e-6f);
                FloatType GA = G * area;  // PDF = 1/area, so weight = area

                glm::vec3 albedo = glm::vec3(hdr_colour.red, hdr_colour.green, hdr_colour.blue);
                FloatType inv_pi = 1.0f / static_cast<FloatType>(std::numbers::pi);

                // Diffuse accumulation (Lambertian BRDF)
                diffuse_rgb_accum += (albedo * ((Le * vis) * (GA * inv_pi)));

                // Specular accumulation (Blinn-Phong)
                glm::vec3 halfway = glm::normalize(L + to_camera_hit);
                FloatType cos_alpha = std::max(0.0f, glm::dot(n_shade, halfway));
                FloatType le_lum = 0.2126f * Le.r + 0.7152f * Le.g + 0.0722f * Le.b;
                FloatType vis_lum = 0.2126f * vis.r + 0.7152f * vis.g + 0.0722f * vis.b;
                specular += le_lum * std::pow(cos_alpha, face.material.shininess) * area * vis_lum /
                            (dist * dist + 1e-6f);
            }
            diffuse_component = ColourHDR{
                .red = diffuse_rgb_accum.r,
                .green = diffuse_rgb_accum.g,
                .blue = diffuse_rgb_accum.b
            };
        } else {
            diffuse_component = ColourHDR{.red = 0.0f, .green = 0.0f, .blue = 0.0f};
        }
    }

    ColourHDR ambient_component = hdr_colour * ambient;
    if (area_lights.empty()) {
        diffuse_component = ColourHDR{.red = 0.0f, .green = 0.0f, .blue = 0.0f};
    }
    specular_component = ColourHDR{.red = specular, .green = specular, .blue = specular};

    ColourHDR direct_lighting;
    if (face.material.metallic > 0.0f) {
        // Metals absorb diffuse component, reflect specular
        direct_lighting =
            ambient_component + diffuse_component + hdr_colour * specular_component.red;
    } else {
        direct_lighting = ambient_component + diffuse_component + specular_component;
    }

    // Glossy reflection for metallic surfaces with Russian Roulette
    if (face.material.metallic > 0.0f) {
        // Use normal-mapped normal for reflection if available (intersection.normal already
        // contains the perturbed normal from normal mapping), otherwise fall back based on shading
        glm::vec3 use_normal = intersection.normal;
        // Only use flat face normal if no normal mapping was applied and shading is FLAT
        if (!face.material.normal_map && face.material.shading == Material::Shading::FLAT) {
            use_normal = face.face_normal;
        }
        // Only flip normal for non-normal-mapped surfaces (normal maps handle their own
        // orientation)
        if (!face.material.normal_map && glm::dot(use_normal, -ray_dir) < 0.0f) {
            use_normal = -use_normal;
        }
        glm::vec3 reflected_dir = glm::reflect(ray_dir, use_normal);
        constexpr FloatType epsilon = 0.001f;

        // Epsilon origin shift: avoid self-hit on the same triangle
        glm::vec3 offset_origin = intersection.intersectionPoint + use_normal * epsilon;
        glm::vec3 next_tp =
            throughput * glm::vec3(hdr_colour.red, hdr_colour.green, hdr_colour.blue);

        // Russian Roulette for path termination
        FloatType p = std::max(next_tp.x, std::max(next_tp.y, next_tp.z));
        p = std::clamp(p, 0.05f, 1.0f);
        glm::vec3 compensated_tp = next_tp / p;

        ColourHDR reflected_color = trace_ray(
            offset_origin, reflected_dir, depth + 1, medium, use_caustics, compensated_tp, rng
        );

        // Schlick's Approximation for Metallic Fresnel
        // F(θ) = F0 + (1.0 - F0)(1.0 - cosθ)^5
        // For metals, F0 is the surface albedo (hdr_colour)
        FloatType cos_theta = std::max(0.0f, glm::dot(-ray_dir, use_normal));
        ColourHDR F0 = hdr_colour;
        FloatType factor = std::pow(1.0f - cos_theta, 5.0f);
        ColourHDR F = ColourHDR{
            .red = F0.red + (1.0f - F0.red) * factor,
            .green = F0.green + (1.0f - F0.green) * factor,
            .blue = F0.blue + (1.0f - F0.blue) * factor
        };

        // The reflection is tinted by F, not just F0
        ColourHDR metallic_reflection = ColourHDR{
            .red = reflected_color.red * F.red,
            .green = reflected_color.green * F.green,
            .blue = reflected_color.blue * F.blue
        };
        ColourHDR out = direct_lighting * (1.0f - face.material.metallic) +
                        metallic_reflection * face.material.metallic;
        if (depth > 0) out = ClampRadiance(out, 10.0f);
        return out;
    }

    // Caustics (global illumination via photon map)
    ColourHDR caustics_contribution(0.0f, 0.0f, 0.0f);
    if (use_caustics && photon_map_ && photon_map_->is_ready()) {
        // Estimate caustic radiance at this point
        caustics_contribution = photon_map_->estimate_caustic(
            intersection.intersectionPoint, intersection.normal, PhotonMap::CausticSearchRadius
        );

        // Modulate caustics by surface albedo (diffuse color)
        caustics_contribution = ColourHDR{
            .red = caustics_contribution.red * hdr_colour.red,
            .green = caustics_contribution.green * hdr_colour.green,
            .blue = caustics_contribution.blue * hdr_colour.blue
        };

        // Tame intensity for stability
        // caustics_contribution = caustics_contribution * 0.25f;
    }

    ColourHDR out = (direct_lighting + caustics_contribution) * medium_absorption;
    if (depth > 0) out = ClampRadiance(out, 10.0f);
    return out;
}

RayTriangleIntersection RayTracer::hit(const glm::vec3& ro, const glm::vec3& rd) const noexcept {
    return world_.accelerator_.intersect(ro, rd, world_.all_faces_);
}

glm::vec3 RayTracer::compute_transmittance_bvh(const glm::vec3& point, const glm::vec3& light_pos)
    const noexcept {
    return world_.accelerator_.transmittance(point, light_pos, world_.all_faces_);
}
