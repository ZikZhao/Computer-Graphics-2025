#include "raytracer.hpp"
#include "photon_map.hpp"
#include <numeric>
#include <algorithm>
#include <functional>
#include <limits>
#include <numbers>
#include <iostream>

// DEBUG: Set to true to only show caustics (black background)
bool RayTracer::debug_visualize_caustics_only = false;

RayTracer::RayTracer(const World& world)
    : world_(world),
      bvh_tri_indices_([]() { return std::vector<int>(); }()),
      bvh_nodes_([]() { return std::vector<BVHNode>(); }()) {
    // Build BVH once
    auto [indices, nodes] = build_bvh(world.all_faces());
    const_cast<std::vector<int>&>(bvh_tri_indices_) = std::move(indices);
    const_cast<std::vector<BVHNode>&>(bvh_nodes_) = std::move(nodes);
    
    // Create shader instances
    shader_lambertian_ = std::make_unique<LambertianShader>();
    shader_metal_ = std::make_unique<MetalShader>();
    shader_dielectric_ = std::make_unique<DielectricShader>();
    
    // Initialize photon map (starts background thread automatically)
    std::cout << "RayTracer: Initializing photon map for caustics..." << std::endl;
    photon_map_ = std::make_unique<PhotonMap>(world);
    std::cout << "RayTracer: Photon map initialized (computing in background)" << std::endl;
}

ColourHDR RayTracer::render_pixel(const Camera& cam, int x, int y, int width, int height,
                                  bool soft_shadows, FloatType light_intensity, bool use_caustics) const noexcept {
    auto [ray_origin, ray_dir] = cam.generate_ray(x, y, width, height, 
                                                    static_cast<double>(width) / height);
    return trace_ray(ray_origin, ray_dir, 0, MediumState{}, soft_shadows, light_intensity, use_caustics);
}

// Helper for DOF lens sampling
static glm::vec2 sample_disk_concentric(FloatType u1, FloatType u2) noexcept {
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
                                      bool soft_shadows, FloatType light_intensity, bool use_caustics) const noexcept {
    ColourHDR accumulated_color(0.0f, 0.0f, 0.0f);
    double aspect_ratio = static_cast<double>(width) / height;
    
    for (int sample = 0; sample < samples; ++sample) {
        FloatType u1 = halton(sample, 2);
        FloatType u2 = halton(sample, 3);
        glm::vec2 lens_sample = sample_disk_concentric(u1, u2) * aperture_size;
        
        auto [center_origin, center_dir] = cam.generate_ray(x, y, width, height, aspect_ratio);
        glm::vec3 focal_point = center_origin + center_dir * focal_distance;
        
        glm::mat3 cam_orientation = cam.orientation();
        glm::vec3 lens_offset = cam_orientation[0] * lens_sample.x + cam_orientation[1] * lens_sample.y;
        glm::vec3 ray_origin = center_origin + lens_offset;
        glm::vec3 ray_dir = glm::normalize(focal_point - ray_origin);
        
        accumulated_color = accumulated_color + trace_ray(ray_origin, ray_dir, 0, MediumState{}, soft_shadows, light_intensity, use_caustics);
    }
    
    return ColourHDR(
        accumulated_color.red / static_cast<FloatType>(samples),
        accumulated_color.green / static_cast<FloatType>(samples),
        accumulated_color.blue / static_cast<FloatType>(samples)
    );
}

HitRecord RayTracer::hit(const glm::vec3& ro, const glm::vec3& rd) const noexcept {
    HitRecord closest;
    closest.distanceFromCamera = std::numeric_limits<FloatType>::infinity();
    closest.triangleIndex = static_cast<std::size_t>(-1);
    
    if (bvh_nodes_.empty()) return closest;
    
    const std::vector<Face>& faces = world_.all_faces();
    std::vector<int> stack;
    stack.reserve(64);
    stack.push_back(0);
    
    while (!stack.empty()) {
        int ni = stack.back();
        stack.pop_back();
        const BVHNode& n = bvh_nodes_[ni];
        if (!intersect_aabb(ro, rd, n.box, closest.distanceFromCamera)) continue;
        
        if (n.count == 0) {
            stack.push_back(n.left);
            stack.push_back(n.right);
        } else {
            for (int i = 0; i < n.count; ++i) {
                int tri_index = bvh_tri_indices_[n.start + i];
                const Face& face = faces[tri_index];
                FloatType t, u, v;
                
                if (IntersectRayTriangle(ro, rd, face.vertices[0], face.vertices[1], face.vertices[2], t, u, v) 
                    && t < closest.distanceFromCamera) {
                    closest.distanceFromCamera = t;
                    closest.intersectionPoint = ro + rd * t;
                    closest.triangleIndex = tri_index;
                    closest.u = u;
                    closest.v = v;
                    
                    FloatType w = 1.0f - u - v;
                    glm::vec3 interpolated_normal = glm::normalize(
                        w * face.vertex_normals[0] + u * face.vertex_normals[1] + v * face.vertex_normals[2]
                    );
                    
                    if (glm::dot(interpolated_normal, -rd) < 0.0f) {
                        interpolated_normal = -interpolated_normal;
                    }
                    closest.normal = interpolated_normal;
                    
                    glm::vec2 uv_coord = face.texture_coords[0] * w + face.texture_coords[1] * u + face.texture_coords[2] * v;
                    if (face.material.texture) {
                        Colour tex_sample = face.material.texture->sample(uv_coord.x, uv_coord.y);
                        closest.color = glm::vec3(
                            (tex_sample.red / 255.0f) * face.material.base_color.r,
                            (tex_sample.green / 255.0f) * face.material.base_color.g,
                            (tex_sample.blue / 255.0f) * face.material.base_color.b
                        );
                    } else {
                        closest.color = face.material.base_color;
                    }
                }
            }
        }
    }
    
    return closest;
}

ColourHDR RayTracer::trace_ray(const glm::vec3& ray_origin, const glm::vec3& ray_dir, int depth,
                               const MediumState& medium, bool soft_shadows, FloatType light_intensity, bool use_caustics) const noexcept {
    constexpr int MAX_DEPTH = 5;
    
    if (depth >= MAX_DEPTH) {
        if (world_.env_map().is_loaded()) {
            return world_.env_map().sample(ray_dir);
        }
        return ColourHDR(0.0f, 0.0f, 0.0f);
    }
    
    HitRecord intersection = hit(ray_origin, ray_dir);
    
    // Apply medium absorption if inside transparent medium
    ColourHDR medium_absorption(1.0f, 1.0f, 1.0f);
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
        // DEBUG MODE: return black for background
        if (debug_visualize_caustics_only) {
            return ColourHDR(0.0f, 0.0f, 0.0f);
        }
        
        ColourHDR env_color;
        if (world_.env_map().is_loaded()) {
            env_color = world_.env_map().sample(ray_dir);
        } else {
            env_color = ColourHDR(0.0f, 0.0f, 0.0f);
        }
        return env_color * medium_absorption;
    }
    
    const std::vector<Face>& all_faces = world_.all_faces();
    const Face& face = all_faces[intersection.triangleIndex];
    const glm::vec3& light_pos = world_.light_position();
    
    // SHADER DISPATCH: Choose shader based on material properties
    // TODO: Eventually use shader system for pure path tracing
    // For now, use the existing lighting model for compatibility
    
    // Check if material is transparent
    if (face.material.tw > 0.0f) {
        constexpr FloatType epsilon = 0.001f;
        glm::vec3 normal = intersection.normal;
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
            glm::vec3 offset_origin = intersection.intersectionPoint + normal * epsilon;
            ColourHDR reflected = trace_ray(offset_origin, reflected_dir, depth + 1, medium, soft_shadows, light_intensity, use_caustics);
            return reflected * medium_absorption;
        }
        
        // Fresnel
        FloatType r0 = (1.0f - ior_ratio) / (1.0f + ior_ratio);
        r0 = r0 * r0;
        FloatType cos_term = 1.0f - std::abs(cos_theta_i);
        FloatType cos5 = cos_term * cos_term * cos_term * cos_term * cos_term;
        FloatType fresnel = r0 + (1.0f - r0) * cos5;
        
        FloatType reflect_weight = fresnel * (1.0f - face.material.tw);
        FloatType refract_weight = (1.0f - fresnel) * face.material.tw;
        
        ColourHDR result_color(0.0f, 0.0f, 0.0f);
        
        if (reflect_weight > 0.01f) {
            glm::vec3 reflected_dir = glm::reflect(ray_dir, normal);
            glm::vec3 offset_origin = intersection.intersectionPoint + normal * epsilon;
            ColourHDR reflected_color = trace_ray(offset_origin, reflected_dir, depth + 1, medium, soft_shadows, light_intensity, use_caustics);
            result_color = result_color + reflected_color * reflect_weight;
        }
        
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
            ColourHDR refracted_color = trace_ray(offset_origin, refracted_dir, depth + 1, new_medium, soft_shadows, light_intensity, use_caustics);
            result_color = result_color + refracted_color * refract_weight;
        }
        
        FloatType direct_weight = 1.0f - reflect_weight - refract_weight;
        if (direct_weight > 0.01f) {
            ColourHDR hdr_colour = ColourHDR(intersection.color.r, intersection.color.g, intersection.color.b);
            FloatType ambient = 0.025f;
            result_color = result_color + hdr_colour * ambient * direct_weight;
        }
        
        return result_color * medium_absorption;
    }
    
    // Standard lighting for opaque materials
    glm::vec3 to_light_hit = light_pos - intersection.intersectionPoint;
    FloatType dist_light_hit = glm::length(to_light_hit);
    glm::vec3 to_camera_hit = -ray_dir;
    
    ColourHDR shadow_color;
    bool backface_view_gate = false;
    {
        glm::vec3 light_dir = glm::normalize(to_light_hit);
        glm::vec3 n_to_light = face.face_normal;
        if (glm::dot(n_to_light, light_dir) < 0.0f) {
            n_to_light = -n_to_light;
        }
        glm::vec3 v_cam_point = glm::normalize(ray_dir);
        FloatType nl_view_dot = glm::dot(n_to_light, v_cam_point);
        if (nl_view_dot > 0.0f) {
            backface_view_gate = true;
            shadow_color = ColourHDR(1.0f, 1.0f, 1.0f);
        } else {
            if (soft_shadows) {
                constexpr int shadow_samples = 16;
                FloatType light_radius = world_.light_radius();
                FloatType shadow_factor = compute_soft_shadow(intersection.intersectionPoint, light_pos, light_radius, shadow_samples);
                shadow_color = ColourHDR(shadow_factor, shadow_factor, shadow_factor);
            } else {
                glm::vec3 transmittance = compute_transmittance_bvh(intersection.intersectionPoint, light_pos);
                shadow_color = ColourHDR(transmittance.r, transmittance.g, transmittance.b);
            }
        }
    }
    
    ColourHDR hdr_colour = ColourHDR(intersection.color.r, intersection.color.g, intersection.color.b);
    FloatType ambient = 0.025f;
    FloatType lambertian = 0.0f;
    FloatType specular = 0.0f;
    
    FloatType w = 1.0f - intersection.u - intersection.v;
    if (!backface_view_gate) {
        if (face.material.shading == Material::Shading::Flat) {
            lambertian = compute_lambertian_lighting(face.face_normal, to_light_hit, dist_light_hit, light_intensity);
            if (lambertian > 0.0f) {
                specular = compute_specular_lighting(face.face_normal, to_light_hit, to_camera_hit, dist_light_hit, light_intensity, face.material.shininess);
            }
        } else if (face.material.shading == Material::Shading::Gouraud) {
            FloatType lam_v[3] = {0.0f, 0.0f, 0.0f};
            FloatType spec_v[3] = {0.0f, 0.0f, 0.0f};
            for (int k = 0; k < 3; ++k) {
                glm::vec3 to_light_v = light_pos - face.vertices[k];
                FloatType dist_v = glm::length(to_light_v);
                glm::vec3 to_camera_v = -ray_dir;
                lam_v[k] = compute_lambertian_lighting(face.vertex_normals[k], to_light_v, dist_v, light_intensity);
                if (lam_v[k] > 0.0f) {
                    spec_v[k] = compute_specular_lighting(face.vertex_normals[k], to_light_v, to_camera_v, dist_v, light_intensity, face.material.shininess);
                }
            }
            lambertian = w * lam_v[0] + intersection.u * lam_v[1] + intersection.v * lam_v[2];
            specular = w * spec_v[0] + intersection.u * spec_v[1] + intersection.v * spec_v[2];
        } else {
            glm::vec3 n_interp = glm::normalize(w * face.vertex_normals[0] + intersection.u * face.vertex_normals[1] + intersection.v * face.vertex_normals[2]);
            lambertian = compute_lambertian_lighting(n_interp, to_light_hit, dist_light_hit, light_intensity);
            if (lambertian > 0.0f) {
                specular = compute_specular_lighting(n_interp, to_light_hit, to_camera_hit, dist_light_hit, light_intensity, face.material.shininess);
            }
        }
    }
    
    // Separate ambient (not affected by shadows) and direct lighting (affected by shadows)
    ColourHDR ambient_component = hdr_colour * ambient;
    ColourHDR diffuse_component = hdr_colour * lambertian * shadow_color;
    ColourHDR specular_component = ColourHDR(specular, specular, specular) * shadow_color;
    
    ColourHDR direct_lighting;
    if (face.material.metallic > 0.0f) {
        direct_lighting = ambient_component + diffuse_component + hdr_colour * specular_component.red;
    } else {
        direct_lighting = ambient_component + diffuse_component + specular_component;
    }
    
    if (face.material.metallic > 0.0f) {
        glm::vec3 reflected_dir = glm::reflect(ray_dir, intersection.normal);
        constexpr FloatType epsilon = 0.001f;
        glm::vec3 offset_origin = intersection.intersectionPoint + intersection.normal * epsilon;
        ColourHDR reflected_color = trace_ray(offset_origin, reflected_dir, depth + 1, medium, soft_shadows, light_intensity, use_caustics);
        ColourHDR metallic_reflection = ColourHDR(
            reflected_color.red * hdr_colour.red,
            reflected_color.green * hdr_colour.green,
            reflected_color.blue * hdr_colour.blue
        );
        return direct_lighting * (1.0f - face.material.metallic * 0.8f) + metallic_reflection * face.material.metallic;
    }
    
    // Add caustics contribution if enabled
    ColourHDR caustics_contribution(0.0f, 0.0f, 0.0f);
    if (use_caustics && photon_map_ && photon_map_->is_ready()) {
        // Estimate caustic radiance at this point
        caustics_contribution = photon_map_->estimate_caustic(&face, intersection.intersectionPoint, intersection.normal, PhotonMap::CAUSTIC_SEARCH_RADIUS);
        
        // DEBUG MODE: Show only caustics RAW COLOR (no material modulation) for verification
        if (debug_visualize_caustics_only) {
            // Boost by 40x and show raw photon color (before material modulation)
            return caustics_contribution * 40.0f;
        }
        
        // Modulate caustics by surface albedo (diffuse color) to get final color
        caustics_contribution = ColourHDR(
            caustics_contribution.red * hdr_colour.red,
            caustics_contribution.green * hdr_colour.green,
            caustics_contribution.blue * hdr_colour.blue
        );
        
        // Scale down to 12.5% to reduce brightness
        caustics_contribution = caustics_contribution * 0.025f;
    }
    
    // DEBUG MODE: Non-caustic surfaces return black
    if (debug_visualize_caustics_only) {
        return ColourHDR(0.0f, 0.0f, 0.0f);
    }
    
    return (direct_lighting + caustics_contribution) * medium_absorption;
}

// Shadow and lighting helpers
glm::vec3 RayTracer::compute_transmittance_bvh(const glm::vec3& point, const glm::vec3& light_pos) const noexcept {
    glm::vec3 to_light = light_pos - point;
    FloatType light_distance = glm::length(to_light);
    glm::vec3 light_dir = to_light / light_distance;
    
    if (bvh_nodes_.empty()) return glm::vec3(1.0f, 1.0f, 1.0f);
    
    glm::vec3 transmittance(1.0f, 1.0f, 1.0f);
    const std::vector<Face>& faces = world_.all_faces();
    std::vector<int> stack;
    stack.reserve(64);
    stack.push_back(0);
    constexpr FloatType min_t = 0.001f;
    
    struct Intersection {
        FloatType t;
        const Face* face;
        FloatType u, v;
    };
    std::vector<Intersection> intersections;
    
    while (!stack.empty()) {
        int ni = stack.back();
        stack.pop_back();
        const BVHNode& n = bvh_nodes_[ni];
        if (!intersect_aabb(point, light_dir, n.box, light_distance)) continue;
        if (n.count == 0) {
            stack.push_back(n.left);
            stack.push_back(n.right);
        } else {
            for (int i = 0; i < n.count; ++i) {
                int tri_index = bvh_tri_indices_[n.start + i];
                const Face& face = faces[tri_index];
                FloatType t, u, v;
                bool hit = IntersectRayTriangle(point, light_dir, face.vertices[0], face.vertices[1], face.vertices[2], t, u, v);
                if (hit && t > min_t && t < light_distance) {
                    intersections.push_back({t, &face, u, v});
                }
            }
        }
    }
    
    std::sort(intersections.begin(), intersections.end(), 
              [](const Intersection& a, const Intersection& b) { return a.t < b.t; });
    
    for (size_t i = 0; i < intersections.size(); ++i) {
        const auto& isect = intersections[i];
        const Material& mat = isect.face->material;
        
        if (mat.tw < 0.01f) {
            return glm::vec3(0.0f, 0.0f, 0.0f);
        }
        
        if (i + 1 < intersections.size()) {
            const auto& next_isect = intersections[i + 1];
            FloatType distance_in_medium = next_isect.t - isect.t;
            
            glm::vec3 effective_sigma_a;
            if (glm::length(mat.sigma_a) > 0.0f) {
                effective_sigma_a = mat.sigma_a;
            } else if (mat.td > 0.0f) {
                effective_sigma_a = glm::vec3(
                    -std::log(std::max(mat.base_color.r, 0.001f)) / mat.td,
                    -std::log(std::max(mat.base_color.g, 0.001f)) / mat.td,
                    -std::log(std::max(mat.base_color.b, 0.001f)) / mat.td
                );
            } else {
                effective_sigma_a = glm::vec3(0.0f);
            }
            
            glm::vec3 absorption = glm::vec3(
                std::exp(-effective_sigma_a.r * distance_in_medium),
                std::exp(-effective_sigma_a.g * distance_in_medium),
                std::exp(-effective_sigma_a.b * distance_in_medium)
            );
            
            transmittance = transmittance * absorption;
            ++i;
        }
        
        if (transmittance.r < 0.001f && transmittance.g < 0.001f && transmittance.b < 0.001f) {
            return glm::vec3(0.0f, 0.0f, 0.0f);
        }
    }
    
    return transmittance;
}

bool RayTracer::is_in_shadow_bvh(const glm::vec3& point, const glm::vec3& light_pos) const noexcept {
    glm::vec3 transmittance = compute_transmittance_bvh(point, light_pos);
    return (transmittance.r < 0.01f && transmittance.g < 0.01f && transmittance.b < 0.01f);
}

FloatType RayTracer::compute_soft_shadow(const glm::vec3& point, const glm::vec3& light_center, 
                                         FloatType light_radius, int num_samples) const noexcept {
    FloatType visible_light = 0.0f;
    constexpr int early_test_samples = 4;
    FloatType early_visible = 0.0f;
    
    for (int i = 0; i < early_test_samples; ++i) {
        glm::vec3 light_sample = sample_sphere_halton(i, light_radius, light_center);
        glm::vec3 transmittance = compute_transmittance_bvh(point, light_sample);
        early_visible += (transmittance.r + transmittance.g + transmittance.b) / 3.0f;
    }
    
    if (early_visible < 0.01f) return 0.0f;
    if (early_visible >= early_test_samples * 0.99f) return 1.0f;
    
    visible_light = early_visible;
    for (int i = early_test_samples; i < num_samples; ++i) {
        glm::vec3 light_sample = sample_sphere_halton(i, light_radius, light_center);
        glm::vec3 transmittance = compute_transmittance_bvh(point, light_sample);
        visible_light += (transmittance.r + transmittance.g + transmittance.b) / 3.0f;
    }
    
    return visible_light / static_cast<FloatType>(num_samples);
}

FloatType RayTracer::compute_lambertian_lighting(const glm::vec3& normal, const glm::vec3& to_light, 
                                                  FloatType distance, FloatType intensity) noexcept {
    glm::vec3 light_dir = glm::normalize(to_light);
    FloatType cos_theta = glm::dot(normal, light_dir);
    
    if (cos_theta <= 0.0f) {
        return 0.0f;
    }
    
    FloatType attenuation = (intensity * cos_theta) / (4.0f * std::numbers::pi * distance * distance);
    return std::clamp(attenuation, 0.0f, 1.0f);
}

FloatType RayTracer::compute_specular_lighting(const glm::vec3& normal, const glm::vec3& to_light, 
                                                const glm::vec3& to_camera, FloatType distance, 
                                                FloatType intensity, FloatType shininess) noexcept {
    glm::vec3 light_dir = glm::normalize(to_light);
    glm::vec3 view_dir = glm::normalize(to_camera);
    
    FloatType n_dot_v = glm::dot(normal, view_dir);
    if (n_dot_v <= 0.0f) {
        return 0.0f;
    }
    
    glm::vec3 halfway = glm::normalize(light_dir + view_dir);
    FloatType cos_alpha = glm::dot(normal, halfway);
    
    if (cos_alpha <= 0.0f) {
        return 0.0f;
    }
    
    FloatType spec_term = std::pow(cos_alpha, shininess);
    FloatType attenuation = (intensity * spec_term) / (4.0f * std::numbers::pi * distance * distance);
    
    return std::clamp(attenuation, 0.0f, 1.0f);
}

FloatType RayTracer::halton(int index, int base) noexcept {
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

glm::vec3 RayTracer::sample_sphere_halton(int index, FloatType radius, const glm::vec3& center) noexcept {
    FloatType u = halton(index, 2);
    FloatType v = halton(index, 3);
    
    FloatType theta = 2.0f * std::numbers::pi * u;
    FloatType phi = std::acos(2.0f * v - 1.0f);
    
    FloatType sin_phi = std::sin(phi);
    FloatType x = radius * sin_phi * std::cos(theta);
    FloatType y = radius * sin_phi * std::sin(theta);
    FloatType z = radius * std::cos(phi);
    
    return center + glm::vec3(x, y, z);
}

// BVH construction (moved from Renderer)
static inline RayTracer::AABB tri_aabb(const Face& f) noexcept {
    glm::vec3 mn = glm::min(glm::min(f.vertices[0], f.vertices[1]), f.vertices[2]);
    glm::vec3 mx = glm::max(glm::max(f.vertices[0], f.vertices[1]), f.vertices[2]);
    return RayTracer::AABB{mn, mx};
}

bool RayTracer::intersect_aabb(const glm::vec3& ro, const glm::vec3& rd, const AABB& box, FloatType tmax) noexcept {
    glm::vec3 inv = glm::vec3(1.0f) / rd;
    glm::vec3 t0 = (box.min - ro) * inv;
    glm::vec3 t1 = (box.max - ro) * inv;
    glm::vec3 tmin = glm::min(t0, t1);
    glm::vec3 tmaxv = glm::max(t0, t1);
    FloatType t_enter = std::max(std::max(tmin.x, tmin.y), tmin.z);
    FloatType t_exit = std::min(std::min(tmaxv.x, tmaxv.y), tmaxv.z);
    return t_enter <= t_exit && t_exit >= 0.0f && t_enter <= tmax;
}

std::pair<std::vector<int>, std::vector<RayTracer::BVHNode>> RayTracer::build_bvh(const std::vector<Face>& faces) noexcept {
    std::vector<int> tri_indices(faces.size());
    std::iota(tri_indices.begin(), tri_indices.end(), 0);
    
    struct Cent { glm::vec3 c; RayTracer::AABB b; };
    std::vector<Cent> data(faces.size());
    for (std::size_t i = 0; i < faces.size(); ++i) {
        auto b = tri_aabb(faces[i]);
        glm::vec3 c = (faces[i].vertices[0] + faces[i].vertices[1] + faces[i].vertices[2]) / 3.0f;
        data[i] = Cent{c, b};
    }
    
    std::vector<RayTracer::BVHNode> nodes;
    
    auto surface_area = [](const RayTracer::AABB& box) -> FloatType {
        glm::vec3 extent = box.max - box.min;
        return 2.0f * (extent.x * extent.y + extent.y * extent.z + extent.z * extent.x);
    };
    
    constexpr FloatType traversal_cost = 1.0f;
    constexpr FloatType intersection_cost = 1.0f;
    constexpr int sah_buckets = 12;
    constexpr int leaf_threshold = 4;
    
    std::function<int(int, int)> build = [&](int start, int end) -> int {
        RayTracer::AABB box{glm::vec3(std::numeric_limits<float>::infinity()), glm::vec3(-std::numeric_limits<float>::infinity())};
        RayTracer::AABB cbox{box.min, box.max};
        for (int i = start; i < end; ++i) {
            box.min = glm::min(box.min, data[tri_indices[i]].b.min);
            box.max = glm::max(box.max, data[tri_indices[i]].b.max);
            cbox.min = glm::min(cbox.min, data[tri_indices[i]].c);
            cbox.max = glm::max(cbox.max, data[tri_indices[i]].c);
        }
        
        int count = end - start;
        int node_index = (int)nodes.size();
        nodes.push_back(RayTracer::BVHNode{box, -1, -1, start, count});
        
        if (count <= leaf_threshold) {
            return node_index;
        }
        
        FloatType best_cost = std::numeric_limits<FloatType>::infinity();
        int best_axis = -1;
        int best_split = -1;
        
        glm::vec3 extent = cbox.max - cbox.min;
        FloatType parent_area = surface_area(box);
        
        for (int axis = 0; axis < 3; ++axis) {
            if (extent[axis] < 1e-6f) continue;
            
            struct Bucket {
                int count = 0;
                RayTracer::AABB bounds{glm::vec3(std::numeric_limits<float>::infinity()), 
                                      glm::vec3(-std::numeric_limits<float>::infinity())};
            };
            std::array<Bucket, sah_buckets> buckets;
            
            for (int i = start; i < end; ++i) {
                FloatType centroid = data[tri_indices[i]].c[axis];
                int bucket_idx = static_cast<int>(sah_buckets * 
                    ((centroid - cbox.min[axis]) / extent[axis]));
                bucket_idx = std::clamp(bucket_idx, 0, sah_buckets - 1);
                
                buckets[bucket_idx].count++;
                buckets[bucket_idx].bounds.min = glm::min(buckets[bucket_idx].bounds.min, 
                                                          data[tri_indices[i]].b.min);
                buckets[bucket_idx].bounds.max = glm::max(buckets[bucket_idx].bounds.max, 
                                                          data[tri_indices[i]].b.max);
            }
            
            for (int split = 0; split < sah_buckets - 1; ++split) {
                RayTracer::AABB left_box{glm::vec3(std::numeric_limits<float>::infinity()), 
                                       glm::vec3(-std::numeric_limits<float>::infinity())};
                int left_count = 0;
                for (int i = 0; i <= split; ++i) {
                    if (buckets[i].count > 0) {
                        left_box.min = glm::min(left_box.min, buckets[i].bounds.min);
                        left_box.max = glm::max(left_box.max, buckets[i].bounds.max);
                        left_count += buckets[i].count;
                    }
                }
                
                RayTracer::AABB right_box{glm::vec3(std::numeric_limits<float>::infinity()), 
                                        glm::vec3(-std::numeric_limits<float>::infinity())};
                int right_count = 0;
                for (int i = split + 1; i < sah_buckets; ++i) {
                    if (buckets[i].count > 0) {
                        right_box.min = glm::min(right_box.min, buckets[i].bounds.min);
                        right_box.max = glm::max(right_box.max, buckets[i].bounds.max);
                        right_count += buckets[i].count;
                    }
                }
                
                if (left_count == 0 || right_count == 0) continue;
                
                FloatType cost = traversal_cost;
                cost += (surface_area(left_box) / parent_area) * left_count * intersection_cost;
                cost += (surface_area(right_box) / parent_area) * right_count * intersection_cost;
                
                if (cost < best_cost) {
                    best_cost = cost;
                    best_axis = axis;
                    best_split = split;
                }
            }
        }
        
        FloatType leaf_cost = count * intersection_cost;
        
        if (best_cost >= leaf_cost || best_axis == -1) {
            return node_index;
        }
        
        auto mid_iter = std::partition(tri_indices.begin() + start, tri_indices.begin() + end,
            [&](int idx) {
                FloatType centroid = data[idx].c[best_axis];
                int bucket_idx = static_cast<int>(sah_buckets * 
                    ((centroid - cbox.min[best_axis]) / extent[best_axis]));
                bucket_idx = std::clamp(bucket_idx, 0, sah_buckets - 1);
                return bucket_idx <= best_split;
            });
        
        int mid = static_cast<int>(mid_iter - tri_indices.begin());
        
        if (mid == start || mid == end) {
            mid = (start + end) / 2;
        }
        
        int left = build(start, mid);
        int right = build(mid, end);
        nodes[node_index].left = left;
        nodes[node_index].right = right;
        nodes[node_index].count = 0;
        return node_index;
    };
    
    build(0, (int)faces.size());
    
    return {std::move(tri_indices), std::move(nodes)};
}
