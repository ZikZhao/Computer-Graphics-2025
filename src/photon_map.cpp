#include <algorithm>
#include <cmath>
#include <iostream>
#include "photon_map.hpp"

PhotonMap::PhotonMap(const World& world) 
    : world_(world) 
{
    // Launch worker thread to trace photons asynchronously
    worker_thread_ = std::jthread([this]() { trace_photons(); });
}

std::vector<Photon> PhotonMap::query_photons(const Face* face, const glm::vec3& point, 
                                             FloatType radius) const {
    std::vector<Photon> result;
    GridCell center_cell = get_grid_cell(point);
    auto [cx, cy, cz] = center_cell;
    int cell_range = static_cast<int>(std::ceil(radius / GRID_CELL_SIZE)) + 1;
    FloatType radius_sq = radius * radius;
    for (int dx = -cell_range; dx <= cell_range; ++dx) {
        for (int dy = -cell_range; dy <= cell_range; ++dy) {
            for (int dz = -cell_range; dz <= cell_range; ++dz) {
                GridCell neighbor_cell = std::make_tuple(cx + dx, cy + dy, cz + dz);
                auto it = spatial_grid_.find(neighbor_cell);
                if (it == spatial_grid_.end()) continue;
                for (const auto& photon : it->second) {
                    if (photon.face != face) continue;
                    glm::vec3 diff = photon.position - point;
                    FloatType dist_sq = glm::dot(diff, diff);
                    if (dist_sq < radius_sq) result.push_back(photon);
                }
            }
        }
    }
    return result;
}

ColourHDR PhotonMap::estimate_caustic(const Face* face, const glm::vec3& point, 
                                      const glm::vec3& normal, FloatType search_radius) const noexcept {
    if (!is_ready()) return ColourHDR(0.0f, 0.0f, 0.0f);
    auto photons = query_photons(face, point, search_radius);
    if (photons.empty()) return ColourHDR(0.0f, 0.0f, 0.0f);
    glm::vec3 accumulated_flux(0.0f);
    constexpr FloatType k = 1.1f;
    for (const auto& photon : photons) {
        FloatType alignment = glm::dot(photon.direction, normal);
        if (alignment > 0.0f) continue;
        glm::vec3 diff = photon.position - point;
        FloatType dist = glm::length(diff);
        FloatType weight = std::max(0.0f, 1.0f - (dist / (k * search_radius)));
        FloatType cos_theta = std::max(0.0f, -alignment);
        accumulated_flux += photon.power * (cos_theta * weight);
    }
    FloatType search_area = std::numbers::pi * search_radius * search_radius;
    glm::vec3 radiance = accumulated_flux / search_area;
    return ColourHDR(radiance.x, radiance.y, radiance.z);
}

std::size_t PhotonMap::total_photons() const noexcept {
    std::size_t count = 0;
    for (const auto& [face, photons] : photon_map_) count += photons.size();
    return count;
}

void PhotonMap::trace_photons() {
    const auto& area_lights = world_.area_lights();
    if (area_lights.empty()) {
        std::cerr << "PhotonMap: No area lights found, skipping photon tracing\n";
        is_ready_.store(true, std::memory_order_release);
        return;
    }
    
    // Find all transparent (refractive) objects in the scene and compute global AABB
    std::vector<const Face*> transparent_faces;
    glm::vec3 aabb_min(std::numeric_limits<FloatType>::infinity());
    glm::vec3 aabb_max(-std::numeric_limits<FloatType>::infinity());
    for (const auto& face : world_.all_faces()) {
        if (is_transparent(face.material)) {
            transparent_faces.push_back(&face);
            for (int k = 0; k < 3; ++k) {
                aabb_min = glm::min(aabb_min, world_.all_vertices()[face.vertex_indices[k]]);
                aabb_max = glm::max(aabb_max, world_.all_vertices()[face.vertex_indices[k]]);
            }
        }
    }
    
    if (transparent_faces.empty()) {
        std::cout << "PhotonMap: No transparent objects found, skipping photon emission\n";
        is_ready_.store(true, std::memory_order_release);
        return;
    }
    
    std::cout << "PhotonMap: Found " << transparent_faces.size() << " transparent faces\n";
    std::cout << "PhotonMap: Emitting photons from " << area_lights.size() << " area lights\n";
    
    // Compute bounding sphere for transparent region
    glm::vec3 target_center = (aabb_min + aabb_max) * 0.5f;
    FloatType target_radius = glm::length(aabb_max - target_center);
    
    // Distribute photons across area lights based on area * luminance
    std::vector<FloatType> weights(area_lights.size());
    FloatType weight_sum = 0.0f;
    for (std::size_t i = 0; i < area_lights.size(); ++i) {
        const Face* lf = area_lights[i];
        glm::vec3 e0 = world_.all_vertices()[lf->vertex_indices[1]] - world_.all_vertices()[lf->vertex_indices[0]];
        glm::vec3 e1 = world_.all_vertices()[lf->vertex_indices[2]] - world_.all_vertices()[lf->vertex_indices[0]];
        FloatType area = 0.5f * glm::length(glm::cross(e0, e1));
        glm::vec3 Le = lf->material.emission;
        FloatType lum = 0.2126f * Le.x + 0.7152f * Le.y + 0.0722f * Le.z;
        FloatType w = std::max<FloatType>(1e-6f, area * lum);
        weights[i] = w;
        weight_sum += w;
    }
    
    int photons_emitted = 0;
    for (std::size_t i = 0; i < area_lights.size(); ++i) {
        int photons_for_light = static_cast<int>(PHOTONS_PER_LIGHT * (weights[i] / weight_sum));
        if (photons_for_light <= 0) continue;
        emit_photons_from_area_light(*area_lights[i], target_center, target_radius, photons_for_light);
        photons_emitted += photons_for_light;
    }
    
    std::cout << "PhotonMap: Emitted " << photons_emitted << " photons\n";
    std::cout << "PhotonMap: Photon tracing complete. Total photons stored: " 
              << total_photons() << "\n";
    
    is_ready_.store(true, std::memory_order_release);
}

void PhotonMap::emit_photons_to_object(const Face& target_face, int num_photons) {
    const glm::vec3& light_pos = world_.light_position();
    
    // Compute face center as target point
    glm::vec3 face_center = (world_.all_vertices()[target_face.vertex_indices[0]]
                           + world_.all_vertices()[target_face.vertex_indices[1]]
                           + world_.all_vertices()[target_face.vertex_indices[2]]) / 3.0f;
    glm::vec3 to_face = glm::normalize(face_center - light_pos);
    
    // Emit photons in a cone toward the face
    FloatType cone_angle = 15.0f * (std::numbers::pi / 180.0f);  // 15 degree cone
    // Use reasonable photon power
    glm::vec3 photon_power(world_.light_intensity() * 0.1f / static_cast<FloatType>(num_photons));
    
    for (int i = 0; i < num_photons; ++i) {
        // Use Halton sequence for stratified sampling instead of random
        glm::vec3 direction = SampleConeHalton(i, to_face, cone_angle);
        trace_single_photon(light_pos, direction, photon_power, 0);
    }
}

void PhotonMap::emit_photons_from_area_light(const Face& light_face, const glm::vec3& target_center, FloatType target_radius, int num_photons) {
    glm::vec3 e0 = world_.all_vertices()[light_face.vertex_indices[1]] - world_.all_vertices()[light_face.vertex_indices[0]];
    glm::vec3 e1 = world_.all_vertices()[light_face.vertex_indices[2]] - world_.all_vertices()[light_face.vertex_indices[0]];
    FloatType area = 0.5f * glm::length(glm::cross(e0, e1));
    glm::vec3 Le = light_face.material.emission;
    glm::vec3 n_light = glm::normalize(light_face.face_normal);
    
    glm::vec3 photon_power = Le * (area / static_cast<FloatType>(std::max(1, num_photons)));
    photon_power *= 5.0f; // artistic boost
    
    
    for (int i = 0; i < num_photons; ++i) {
        FloatType u1 = Halton(i, 2);
        FloatType u2 = Halton(i, 3);
        FloatType su = std::sqrt(u1);
        FloatType b0 = 1.0f - su;
        FloatType b1 = su * (1.0f - u2);
        FloatType b2 = su * u2;
        glm::vec3 light_p = world_.all_vertices()[light_face.vertex_indices[0]] + b1 * e0 + b2 * e1;
        glm::vec3 origin = light_p + n_light * 1e-4f;
        glm::vec3 to_center = glm::normalize(target_center - origin);
        FloatType dist = glm::length(target_center - origin);
        FloatType cone_angle = std::atan(target_radius / std::max<FloatType>(dist, 1e-4f)) * 1.5f;
        glm::vec3 direction = SampleConeHalton(i, to_center, cone_angle);
        trace_single_photon(origin, direction, photon_power, 0);
    }
}

void PhotonMap::trace_single_photon(const glm::vec3& origin, const glm::vec3& direction, 
                                    const glm::vec3& power, int depth,
                                    const glm::vec3& medium_entry_point,
                                    bool interacted_with_transparent) {
    
    // Stop if exceeded max bounces
    if (depth >= MAX_PHOTON_BOUNCES) {
        return;
    }
    
    // Find intersection
    auto hit_opt = find_intersection(origin, direction);
    if (!hit_opt.has_value()) {
        return;  // Photon escaped scene
    }
    
    const auto& hit = hit_opt.value();
    const Face* hit_face = &world_.all_faces()[hit.triangleIndex];
    const Material& mat = hit_face->material;
    
    // Check if surface is transparent or diffuse
    bool is_transparent_surface = is_transparent(mat);
    
    // Continue tracing if hit transparent object
    if (is_transparent_surface) {
        // Mark that this photon has interacted with a transparent surface
        interacted_with_transparent = true;
        // Determine if photon is currently inside or outside the medium
        bool currently_inside = glm::length(medium_entry_point) > 0.0f;
        
        // Refraction/reflection logic
        glm::vec3 normal = hit.front_face ? hit.normal : -hit.normal;
        FloatType eta = hit.front_face ? (1.0f / mat.ior) : mat.ior;
        
        // Compute refraction parameters
        FloatType cos_theta = std::min(glm::dot(-direction, normal), 1.0f);
        FloatType sin_theta = std::sqrt(1.0f - cos_theta * cos_theta);
        bool cannot_refract = eta * sin_theta > 1.0f;
        
        // Fresnel (Schlick's approximation)
        FloatType r0 = (1.0f - eta) / (1.0f + eta);
        r0 = r0 * r0;
        FloatType reflectance = r0 + (1.0f - r0) * std::pow(1.0f - cos_theta, 5.0f);
        
        // Prepare for next ray
        glm::vec3 new_direction;
        glm::vec3 new_power = power;
        glm::vec3 new_entry_point = medium_entry_point;
        
        // Apply Beer-Lambert absorption if photon was traveling inside medium
        if (currently_inside) {
            FloatType travel_dist = glm::length(hit.intersectionPoint - medium_entry_point);
            
            if (travel_dist > 0.0f && mat.td > 0.0f) {
                glm::vec3 effective_sigma_a;
                
                if (glm::length(mat.sigma_a) > 0.0f) {
                    effective_sigma_a = mat.sigma_a;
                } else {
                    // Derive absorption from tint color and transmission depth
                    effective_sigma_a = glm::vec3(
                        -std::log(std::max(mat.base_color.r, 0.001f)) / mat.td,
                        -std::log(std::max(mat.base_color.g, 0.001f)) / mat.td,
                        -std::log(std::max(mat.base_color.b, 0.001f)) / mat.td
                    );
                }
                
                // Apply Beer-Lambert law: I = I0 * exp(-sigma_a * distance)
                new_power = glm::vec3(
                    power.x * std::exp(-effective_sigma_a.x * travel_dist),
                    power.y * std::exp(-effective_sigma_a.y * travel_dist),
                    power.z * std::exp(-effective_sigma_a.z * travel_dist)
                );
            }
        }
        
        // Russian roulette after first bounce (with energy compensation for unbiased estimation)
        if (depth >= 1) {
            FloatType power_magnitude = glm::length(new_power);
            // Survival probability based on photon power
            FloatType survival_prob = std::min(0.95f, std::max(0.1f, power_magnitude / MIN_PHOTON_POWER));
            FloatType rr = random_float();
            if (rr > survival_prob) {
                return;  // Photon absorbed
            }
            // Energy compensation: boost surviving photon power to maintain expected value
            // E[new_power] = survival_prob * (new_power / survival_prob) + (1 - survival_prob) * 0 = new_power
            new_power /= survival_prob;
        }
        
        // Decide refraction vs reflection
        if (cannot_refract) {
            // Total internal reflection (only happens inside medium)
            new_direction = glm::reflect(direction, normal);
            // Stay inside, update entry point for next segment
            new_entry_point = hit.intersectionPoint;
        } else if (random_float() < reflectance * 0.5f) {
            // Fresnel reflection (reduced probability for caustics)
            new_direction = glm::reflect(direction, normal);
            // If reflecting from outside, stay outside; if from inside, stay inside
            if (currently_inside) {
                new_entry_point = hit.intersectionPoint;
            } else {
                new_entry_point = glm::vec3(0.0f);  // Outside
            }
        } else {
            // Refraction
            if (hit.front_face) {
                // Entering medium
                new_entry_point = hit.intersectionPoint;
            } else {
                // Exiting medium
                new_entry_point = glm::vec3(0.0f);
            }
            new_direction = glm::refract(direction, normal, eta);
        }
        
        // Continue tracing
        glm::vec3 offset = new_direction * 0.001f;
        trace_single_photon(hit.intersectionPoint + offset, new_direction, new_power, depth + 1, 
                           new_entry_point, interacted_with_transparent);
    } else {
        // Hit diffuse surface
        glm::vec3 final_power = power;
        
        // Apply absorption if photon is currently inside medium
        bool currently_inside = glm::length(medium_entry_point) > 0.0f;
        if (currently_inside) {
            FloatType travel_dist = glm::length(hit.intersectionPoint - medium_entry_point);
            
            const Face* last_transparent_face = nullptr;
            for (const auto& face : world_.all_faces()) {
                if (is_transparent(face.material)) {
                    last_transparent_face = &face;
                    break;
                }
            }
            
            if (last_transparent_face) {
                const Material& mat = last_transparent_face->material;
                glm::vec3 effective_sigma_a = glm::vec3(
                    -std::log(std::max(mat.base_color.r, 0.001f)) / mat.td,
                    -std::log(std::max(mat.base_color.g, 0.001f)) / mat.td,
                    -std::log(std::max(mat.base_color.b, 0.001f)) / mat.td
                );
                
                final_power = glm::vec3(
                    power.x * std::exp(-effective_sigma_a.x * travel_dist),
                    power.y * std::exp(-effective_sigma_a.y * travel_dist),
                    power.z * std::exp(-effective_sigma_a.z * travel_dist)
                );
            }
        }
        
        // Store photon if it has interacted with any transparent surface
        // This captures:
        // - Refractive caustics (with Beer-Lambert absorption if traveled through medium)
        // - External reflections from glass (white light for ceiling caustics)
        // - Internal reflections (with absorption)
        if (interacted_with_transparent) {
            store_photon(Photon(hit.intersectionPoint, direction, final_power, hit_face));
        }
    }
}

void PhotonMap::store_photon(const Photon& photon) {
    // No mutex needed: only single worker thread writes during construction
    photon_map_[photon.face].push_back(photon);
    
    // Also store in spatial grid for fast lookup
    GridCell cell = get_grid_cell(photon.position);
    spatial_grid_[cell].push_back(photon);
}

std::optional<RayTriangleIntersection> PhotonMap::find_intersection(
    const glm::vec3& ro, const glm::vec3& rd) const noexcept {
    auto rec = world_.accelerator().intersect(ro, rd, world_.all_faces());
    if (rec.triangleIndex == static_cast<std::size_t>(-1)) {
        return std::nullopt;
    }
    return rec;
}

std::optional<RayTriangleIntersection> PhotonMap::intersect_triangle(
    const glm::vec3& ro, const glm::vec3& rd, const Face& face) const noexcept {
    
    const glm::vec3& v0 = world_.all_vertices()[face.vertex_indices[0]];
    const glm::vec3& v1 = world_.all_vertices()[face.vertex_indices[1]];
    const glm::vec3& v2 = world_.all_vertices()[face.vertex_indices[2]];
    
    // Möller-Trumbore intersection algorithm
    constexpr FloatType EPSILON = 1e-6f;
    glm::vec3 edge1 = v1 - v0;
    glm::vec3 edge2 = v2 - v0;
    glm::vec3 h = glm::cross(rd, edge2);
    FloatType a = glm::dot(edge1, h);
    
    if (std::abs(a) < EPSILON) {
        return std::nullopt;  // Ray parallel to triangle
    }
    
    FloatType f = 1.0f / a;
    glm::vec3 s = ro - v0;
    FloatType u = f * glm::dot(s, h);
    
    if (u < 0.0f || u > 1.0f) {
        return std::nullopt;
    }
    
    glm::vec3 q = glm::cross(s, edge1);
    FloatType v = f * glm::dot(rd, q);
    
    if (v < 0.0f || u + v > 1.0f) {
        return std::nullopt;
    }
    
    FloatType t = f * glm::dot(edge2, q);
    
    if (t < EPSILON) {
        return std::nullopt;  // Intersection behind ray origin
    }
    
    // Compute intersection details
    RayTriangleIntersection hit;
    hit.intersectionPoint = ro + rd * t;
    hit.distanceFromCamera = t;
    hit.u = u;
    hit.v = v;
    
    // Interpolate normal (smooth shading)
    FloatType w = 1.0f - u - v;
    hit.normal = glm::normalize(w * face.vertex_normals[0] + 
                                 u * face.vertex_normals[1] + 
                                 v * face.vertex_normals[2]);
    
    // Geometric normal
    hit.geom_normal = face.face_normal;
    
    // Determine front/back face
    hit.front_face = glm::dot(rd, hit.geom_normal) < 0.0f;
    if (!hit.front_face) {
        hit.geom_normal = -hit.geom_normal;
    }
    
    return hit;
}


PhotonMap::GridCell PhotonMap::get_grid_cell(const glm::vec3& position) noexcept {
    int x = static_cast<int>(std::floor(position.x / GRID_CELL_SIZE));
    int y = static_cast<int>(std::floor(position.y / GRID_CELL_SIZE));
    int z = static_cast<int>(std::floor(position.z / GRID_CELL_SIZE));
    return std::make_tuple(x, y, z);
}

FloatType PhotonMap::random_float(FloatType min, FloatType max) noexcept {
    thread_local std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<FloatType> dist(min, max);
    return dist(rng);
}
