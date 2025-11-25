#include "PhotonMap.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

PhotonMap::PhotonMap(const World& world) 
    : world_(world) 
{
    // Launch worker thread to trace photons asynchronously
    worker_thread_ = std::jthread([this](std::stop_token stop_tok) {
        if (!stop_tok.stop_requested()) {
            trace_photons();
        }
    });
}

void PhotonMap::trace_photons() {
    if (!world_.has_light()) {
        std::cerr << "PhotonMap: No light source found in scene, skipping photon tracing\n";
        is_ready_.store(true, std::memory_order_release);
        return;
    }
    
    std::cout << "PhotonMap: Starting photon tracing from light at " 
              << world_.light_position().x << ", " 
              << world_.light_position().y << ", " 
              << world_.light_position().z << "\n";
    
    // Find all transparent (refractive) objects in the scene
    std::vector<const Face*> transparent_faces;
    for (const auto& face : world_.all_faces()) {
        if (is_transparent(face.material)) {
            transparent_faces.push_back(&face);
        }
    }
    
    if (transparent_faces.empty()) {
        std::cout << "PhotonMap: No transparent objects found, skipping photon emission\n";
        is_ready_.store(true, std::memory_order_release);
        return;
    }
    
    std::cout << "PhotonMap: Found " << transparent_faces.size() << " transparent faces\n";
    
    // Emit photons toward each transparent object
    int photons_emitted = 0;
    for (const auto* face : transparent_faces) {
        int photons_for_face = PHOTONS_PER_LIGHT / transparent_faces.size();
        emit_photons_to_object(*face, photons_for_face);
        photons_emitted += photons_for_face;
    }
    
    std::cout << "PhotonMap: Emitted " << photons_emitted << " photons\n";
    std::cout << "PhotonMap: Photon tracing complete. Total photons stored: " 
              << total_photons() << "\n";
    
    is_ready_.store(true, std::memory_order_release);
}

void PhotonMap::emit_photons_to_object(const Face& target_face, int num_photons) {
    const glm::vec3& light_pos = world_.light_position();
    
    // Compute face center as target point
    glm::vec3 face_center = (target_face.vertices[0] + target_face.vertices[1] + target_face.vertices[2]) / 3.0f;
    glm::vec3 to_face = glm::normalize(face_center - light_pos);
    
    // Emit photons in a cone toward the face
    FloatType cone_angle = 15.0f * (std::numbers::pi / 180.0f);  // 15 degree cone
    // Use reasonable photon power
    glm::vec3 photon_power(world_.light_intensity() * 0.1f / static_cast<FloatType>(num_photons));
    
    for (int i = 0; i < num_photons; ++i) {
        glm::vec3 direction = random_in_cone(to_face, cone_angle);
        trace_single_photon(light_pos, direction, photon_power, 0);
    }
}

void PhotonMap::trace_single_photon(const glm::vec3& origin, const glm::vec3& direction, 
                                    const glm::vec3& power, int depth,
                                    const glm::vec3& medium_entry_point,
                                    bool has_refracted) {
    // Stop if exceeded max bounces
    if (depth >= MAX_PHOTON_BOUNCES) {
        return;
    }
    
    // Russian roulette after second bounce to avoid premature termination
    // This ensures most photons get at least one chance to create caustics
    if (depth >= 2) {
        FloatType power_magnitude = glm::length(power);
        // Use a more lenient survival probability that doesn't penalize low-power photons as much
        FloatType survival_prob = 0.8f + 0.15f * std::min(1.0f, power_magnitude / MIN_PHOTON_POWER);
        if (random_float() > survival_prob) {
            return;  // Photon absorbed
        }
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
        // Refraction/reflection logic
        glm::vec3 normal = hit.front_face ? hit.normal : -hit.normal;
        FloatType eta = hit.front_face ? (1.0f / mat.ior) : mat.ior;
        
        // Compute refraction
        FloatType cos_theta = std::min(glm::dot(-direction, normal), 1.0f);
        FloatType sin_theta = std::sqrt(1.0f - cos_theta * cos_theta);
        
        bool cannot_refract = eta * sin_theta > 1.0f;
        glm::vec3 new_direction;
        glm::vec3 new_power = power;
        glm::vec3 new_entry_point = medium_entry_point;
        
        // Beer-Lambert absorption: attenuate by material color during transmission
        if (!hit.front_face) {
            // Photon is exiting the medium, apply absorption based on distance traveled inside
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
            // Reset entry point when exiting
            new_entry_point = glm::vec3(0.0f);
        } else {
            // Photon is entering the medium, record entry point
            new_entry_point = hit.intersectionPoint;
        }
        
        // Fresnel (Schlick's approximation)
        FloatType r0 = (1.0f - eta) / (1.0f + eta);
        r0 = r0 * r0;
        FloatType reflectance = r0 + (1.0f - r0) * std::pow(1.0f - cos_theta, 5.0f);
        
        // For caustics, we primarily want refracted photons
        bool will_refract = false;
        bool completed_refraction = false;  // Track if photon completed entry->absorption->exit cycle
        
        if (cannot_refract) {
            // Total internal reflection
            new_direction = glm::reflect(direction, normal);
        } else if (random_float() < reflectance * 0.5f) {
            // Reduce reflection probability to focus on refractive caustics
            new_direction = glm::reflect(direction, normal);
        } else {
            // Refract (preferred for caustics)
            new_direction = glm::refract(direction, normal, eta);
            will_refract = true;
            // Mark as completed refraction ONLY when exiting (after absorption)
            if (!hit.front_face) {
                completed_refraction = true;
            }
        }
        
        // Continue tracing
        glm::vec3 offset = new_direction * 0.001f;  // Avoid self-intersection
        trace_single_photon(hit.intersectionPoint + offset, new_direction, new_power, depth + 1, new_entry_point, has_refracted || completed_refraction);
    } else {
        // Hit diffuse surface - store photon ONLY if it has refracted through glass
        // This excludes pure mirror reflections and only captures refractive caustics
        if (has_refracted) {
            // This is a refractive caustic photon - power includes Beer-Lambert absorption
            store_photon(Photon(hit.intersectionPoint, direction, power, hit_face));
        }
    }
}

void PhotonMap::store_photon(const Photon& photon) {
    // No mutex needed: only single worker thread writes during construction
    photon_map_[photon.face].push_back(photon);
}

std::optional<RayTriangleIntersection> PhotonMap::find_intersection(
    const glm::vec3& ro, const glm::vec3& rd) const noexcept {
    
    std::optional<RayTriangleIntersection> closest_hit;
    FloatType closest_dist = std::numeric_limits<FloatType>::max();
    
    for (std::size_t i = 0; i < world_.all_faces().size(); ++i) {
        const auto& face = world_.all_faces()[i];
        auto hit_opt = intersect_triangle(ro, rd, face);
        
        if (hit_opt.has_value() && hit_opt->distanceFromCamera < closest_dist) {
            closest_dist = hit_opt->distanceFromCamera;
            closest_hit = hit_opt;
            closest_hit->triangleIndex = i;
        }
    }
    
    return closest_hit;
}

std::optional<RayTriangleIntersection> PhotonMap::intersect_triangle(
    const glm::vec3& ro, const glm::vec3& rd, const Face& face) noexcept {
    
    const glm::vec3& v0 = face.vertices[0];
    const glm::vec3& v1 = face.vertices[1];
    const glm::vec3& v2 = face.vertices[2];
    
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

std::vector<Photon> PhotonMap::query_photons(const Face* face, const glm::vec3& point, 
                                              FloatType radius) const {
    std::vector<Photon> result;
    
    auto it = photon_map_.find(face);
    if (it == photon_map_.end()) {
        return result;  // No photons on this face
    }
    
    FloatType radius_sq = radius * radius;
    for (const auto& photon : it->second) {
        glm::vec3 diff = photon.position - point;
        FloatType dist_sq = glm::dot(diff, diff);
        if (dist_sq < radius_sq) {
            result.push_back(photon);
        }
    }
    
    return result;
}

ColourHDR PhotonMap::estimate_caustic(const Face* face, const glm::vec3& point, 
                                      const glm::vec3& normal, FloatType search_radius) const noexcept {
    if (!is_ready()) {
        return ColourHDR(0.0f, 0.0f, 0.0f);
    }
    
    auto photons = query_photons(face, point, search_radius);
    
    if (photons.empty()) {
        return ColourHDR(0.0f, 0.0f, 0.0f);
    }
    
    // Cone filtering with energy conservation
    glm::vec3 accumulated_flux(0.0f);
    constexpr FloatType k = 1.1f;  // Cone filter parameter for smooth falloff
    
    for (const auto& photon : photons) {
        // Leakage check: ensure photon is on the correct side of surface
        // Photon direction points toward surface, normal points away
        FloatType alignment = glm::dot(photon.direction, normal);
        if (alignment > 0.0f) {
            // Photon hit from backface, skip to prevent light leaking
            continue;
        }
        
        // Calculate distance-based cone filter weight
        glm::vec3 diff = photon.position - point;
        FloatType dist = glm::length(diff);
        FloatType weight = 1.0f - (dist / (k * search_radius));
        weight = std::max(0.0f, weight);  // Clamp to [0, 1]
        
        // BRDF: Lambertian diffuse (cos_theta / pi already in power)
        FloatType cos_theta = std::max(0.0f, -alignment);  // -dot because direction points in
        
        // Accumulate weighted photon contribution
        accumulated_flux += photon.power * (cos_theta * weight);
    }
    
    // Energy conservation: normalize by search area (not photon count!)
    FloatType search_area = std::numbers::pi * search_radius * search_radius;
    glm::vec3 radiance = accumulated_flux / search_area;
    
    // DEBUG: Log caustic color estimation occasionally
    static int estimate_count = 0;
    if (estimate_count < 5 && glm::length(radiance) > 0.0001f) {
        std::cout << "Caustic estimate #" << estimate_count << ": photons=" << photons.size()
                  << ", radiance=(" << radiance.x << ", " << radiance.y << ", " << radiance.z << ")\n";
        estimate_count++;
    }
    
    return ColourHDR(radiance.x, radiance.y, radiance.z);
}

std::size_t PhotonMap::total_photons() const noexcept {
    std::size_t count = 0;
    for (const auto& [face, photons] : photon_map_) {
        count += photons.size();
    }
    return count;
}

// ============================================================================
// Random Sampling Utilities
// ============================================================================

glm::vec3 PhotonMap::random_unit_vector() noexcept {
    thread_local static std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<FloatType> dist(-1.0f, 1.0f);
    while (true) {
        glm::vec3 v(dist(rng), dist(rng), dist(rng));
        FloatType len_sq = glm::dot(v, v);
        if (len_sq > 0.0001f && len_sq <= 1.0f) {
            return v / std::sqrt(len_sq);
        }
    }
}

glm::vec3 PhotonMap::random_in_cone(const glm::vec3& direction, FloatType cone_angle) noexcept {
    thread_local static std::mt19937 rng(std::random_device{}());
    // Generate random direction within cone around given direction
    std::uniform_real_distribution<FloatType> dist(0.0f, 1.0f);
    
    FloatType cos_angle = std::cos(cone_angle);
    FloatType z = cos_angle + (1.0f - cos_angle) * dist(rng);
    FloatType phi = 2.0f * std::numbers::pi * dist(rng);
    
    FloatType sin_theta = std::sqrt(1.0f - z * z);
    glm::vec3 sample_dir(sin_theta * std::cos(phi), sin_theta * std::sin(phi), z);
    
    // Build orthonormal basis around direction
    glm::vec3 up = std::abs(direction.y) < 0.999f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    glm::vec3 right = glm::normalize(glm::cross(up, direction));
    glm::vec3 forward = glm::cross(direction, right);
    
    // Transform sample to world space
    return glm::normalize(sample_dir.x * right + sample_dir.y * forward + sample_dir.z * direction);
}

FloatType PhotonMap::random_float(FloatType min, FloatType max) noexcept {
    thread_local static std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<FloatType> dist(min, max);
    return dist(rng);
}
