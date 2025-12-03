#include "photon_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <iostream>

FloatType PhotonMap::RandomFloat(FloatType min, FloatType max) noexcept {
    thread_local std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<FloatType> dist(min, max);
    return dist(rng);
}

PhotonMap::PhotonMap(const World& world) : world_(world) {
    worker_thread_ = std::jthread([this]() { trace_photons(); });
}

std::size_t PhotonMap::total_photons() const noexcept {
    std::size_t count = 0;
    for (const auto& [face, photons] : photon_map_) count += photons.size();
    return count;
}

std::vector<Photon> PhotonMap::query_photons(
    const glm::vec3& point, FloatType radius
) const {
    std::vector<Photon> result;
    GridCell center_cell = get_grid_cell(point);
    auto [cx, cy, cz] = center_cell;
    int cell_range = static_cast<int>(std::ceil(radius / GridCellSize)) + 1;
    FloatType radius_sq = radius * radius;
    for (int dx = -cell_range; dx <= cell_range; ++dx) {
        for (int dy = -cell_range; dy <= cell_range; ++dy) {
            for (int dz = -cell_range; dz <= cell_range; ++dz) {
                int nx = cx + dx;
                int ny = cy + dy;
                int nz = cz + dz;
                if (nx < 0 || ny < 0 || nz < 0 || nx >= grid_width_ || ny >= grid_height_ ||
                    nz >= grid_depth_)
                    continue;
                std::size_t idx = static_cast<std::size_t>(
                    nx + ny * grid_width_ + nz * grid_width_ * grid_height_
                );
                const auto& cell = grid_[idx];
                for (const auto& photon : cell) {
                    // Removed strict face matching - use spatial proximity only
                    // Photons on nearby surfaces contribute to caustics
                    glm::vec3 diff = photon.position - point;
                    FloatType dist_sq = glm::dot(diff, diff);
                    if (dist_sq < radius_sq) result.push_back(photon);
                }
            }
        }
    }
    return result;
}

ColourHDR PhotonMap::estimate_caustic(
    const glm::vec3& point, const glm::vec3& normal, FloatType search_radius
) const noexcept {
    if (!is_ready()) return ColourHDR{.red = 0.0f, .green = 0.0f, .blue = 0.0f};
    auto photons = query_photons(point, search_radius);
    if (photons.empty()) return ColourHDR{.red = 0.0f, .green = 0.0f, .blue = 0.0f};
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
    return ColourHDR{.red = radiance.x, .green = radiance.y, .blue = radiance.z};
}

void PhotonMap::trace_photons() {
    const auto& emissive_faces = world_.emissive_faces_;

    // Find all transparent (refractive) objects in the scene
    std::vector<const Face*> transparent_faces;
    for (const auto& face : world_.all_faces_) {
        if (IsTransparent(face.material)) {
            transparent_faces.push_back(&face);
        }
    }

    if (emissive_faces.empty() || transparent_faces.empty()) {
        std::cout << "[PhotonMap] No Emissive or Refractive Faces\n";
        is_ready_.store(true, std::memory_order_release);
        return;
    }

    // Compute AABB for transparent objects (for targeting emission)
    glm::vec3 transparent_aabb_min(std::numeric_limits<FloatType>::infinity());
    glm::vec3 transparent_aabb_max(-std::numeric_limits<FloatType>::infinity());
    for (const auto* face : transparent_faces) {
        for (int k = 0; k < 3; ++k) {
            transparent_aabb_min = glm::min(transparent_aabb_min, world_.all_vertices_[face->v_indices[k]]);
            transparent_aabb_max = glm::max(transparent_aabb_max, world_.all_vertices_[face->v_indices[k]]);
        }
    }

    // Compute AABB for ENTIRE scene (for photon storage grid)
    glm::vec3 scene_aabb_min(std::numeric_limits<FloatType>::infinity());
    glm::vec3 scene_aabb_max(-std::numeric_limits<FloatType>::infinity());
    for (const auto& face : world_.all_faces_) {
        for (int k = 0; k < 3; ++k) {
            scene_aabb_min = glm::min(scene_aabb_min, world_.all_vertices_[face.v_indices[k]]);
            scene_aabb_max = glm::max(scene_aabb_max, world_.all_vertices_[face.v_indices[k]]);
        }
    }

    std::cout << std::format(
        "[PhotonMap] Start Tracing: {} Emissive Faces | {} Refractive Faces\n",
        emissive_faces.size(),
        transparent_faces.size()
    );

    auto start_time = std::chrono::steady_clock::now();

    // Reset killed photon counter
    killed_photon_count_.store(0, std::memory_order_relaxed);

    // Target center/radius for emission cone (based on transparent objects)
    glm::vec3 target_center = (transparent_aabb_min + transparent_aabb_max) * 0.5f;
    FloatType target_radius = glm::length(transparent_aabb_max - target_center);

    // Spatial indexing: use ENTIRE SCENE bounds for grid
    // This ensures photons landing on floors/walls are properly stored
    grid_origin_ = scene_aabb_min - glm::vec3(GridCellSize);  // Add padding
    glm::vec3 extent = (scene_aabb_max - scene_aabb_min) + glm::vec3(2.0f * GridCellSize);
    grid_width_ = std::max(1, static_cast<int>(std::ceil(extent.x / GridCellSize)));
    grid_height_ = std::max(1, static_cast<int>(std::ceil(extent.y / GridCellSize)));
    grid_depth_ = std::max(1, static_cast<int>(std::ceil(extent.z / GridCellSize)));
    std::size_t grid_size = static_cast<std::size_t>(grid_width_) *
                            static_cast<std::size_t>(grid_height_) *
                            static_cast<std::size_t>(grid_depth_);
    grid_.clear();
    grid_.resize(grid_size);

    // Phase 1: Compute total scene flux and relative probability weights for each light
    std::vector<FloatType> weights(emissive_faces.size());
    FloatType weight_sum = 0.0f;
    total_light_flux_ = 0.0f;
    
    for (std::size_t i = 0; i < emissive_faces.size(); ++i) {
        const Face* lf = emissive_faces[i];
        glm::vec3 e0 =
            world_.all_vertices_[lf->v_indices[1]] - world_.all_vertices_[lf->v_indices[0]];
        glm::vec3 e1 =
            world_.all_vertices_[lf->v_indices[2]] - world_.all_vertices_[lf->v_indices[0]];
        FloatType area = 0.5f * glm::length(glm::cross(e0, e1));
        glm::vec3 Le = lf->material.emission;
        // Luminance (ITU-R BT.709) used for energy-aware photon allocation.
        FloatType lum = 0.2126f * Le.x + 0.7152f * Le.y + 0.0722f * Le.z;
        FloatType flux = area * lum;
        total_light_flux_ += flux;
        FloatType w = std::max<FloatType>(1e-6f, flux);
        weights[i] = w;
        weight_sum += w;
    }

    // Phase 2: Adaptive emission loop
    // Keep emitting batches until we reach TargetStoredPhotons or hit the safety limit
    std::size_t total_emitted_count = 0;
    std::size_t batch_index = 0;
    
    while (total_photons() < TargetStoredPhotons && total_emitted_count < MaxEmittedPhotons) {
        // Emit a batch distributed across all light sources
        for (std::size_t i = 0; i < emissive_faces.size(); ++i) {
            // Calculate how many photons this light gets from the batch
            std::size_t photons_for_light = static_cast<std::size_t>(
                static_cast<FloatType>(BatchSize) * (weights[i] / weight_sum)
            );
            if (photons_for_light == 0) photons_for_light = 1;  // At least 1 photon per light
            
            emit_photon_batch(
                *emissive_faces[i],
                target_center,
                target_radius,
                batch_index * BatchSize + total_emitted_count,  // Unique Halton index
                photons_for_light
            );
            total_emitted_count += photons_for_light;
        }
        batch_index++;
        
        // Progress report every 100 batches
        if (batch_index % 100 == 0) {
            std::cout << std::format(
                "[PhotonMap] Progress: Stored {} / {} target | Emitted: {}\n",
                total_photons(),
                TargetStoredPhotons,
                total_emitted_count
            );
        }
    }

    // Phase 3: Energy normalization
    // Scale all stored photon powers by (Total Light Flux / Total Emitted Count) * ArtisticBoost
    normalize_photon_power(total_emitted_count);

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;

    std::cout << std::format(
        "[PhotonMap] Completed in {:#.3g}s\n",
        elapsed.count()
    );

    is_ready_.store(true, std::memory_order_release);
}

void PhotonMap::emit_photon_batch(
    const Face& light_face,
    const glm::vec3& target_center,
    FloatType target_radius,
    std::size_t batch_start_index,
    std::size_t batch_size
) {
    glm::vec3 e0 = world_.all_vertices_[light_face.v_indices[1]] -
                   world_.all_vertices_[light_face.v_indices[0]];
    glm::vec3 e1 = world_.all_vertices_[light_face.v_indices[2]] -
                   world_.all_vertices_[light_face.v_indices[0]];
    glm::vec3 Le = light_face.material.emission;
    glm::vec3 n_light = glm::normalize(light_face.face_normal);

    // Store photons with light emission color as base.
    // Actual power will be normalized later using:
    //   factor = total_flux / (stored_count + killed_count)
    // This ensures absorbed energy (killed photons) reduces the final brightness.
    glm::vec3 photon_base_color = Le;

    for (std::size_t i = 0; i < batch_size; ++i) {
        std::size_t halton_idx = batch_start_index + i;
        FloatType u1 = Halton(static_cast<int>(halton_idx), 2);
        FloatType u2 = Halton(static_cast<int>(halton_idx), 3);
        FloatType su = std::sqrt(u1);
        FloatType b0 = 1.0f - su;
        FloatType b1 = su * (1.0f - u2);
        FloatType b2 = su * u2;
        glm::vec3 light_p = world_.all_vertices_[light_face.v_indices[0]] + b1 * e0 + b2 * e1;
        glm::vec3 origin = light_p + n_light * 1e-4f;
        glm::vec3 to_center = glm::normalize(target_center - origin);
        FloatType dist = glm::length(target_center - origin);
        FloatType cone_angle = std::atan(target_radius / std::max<FloatType>(dist, 1e-4f)) * 1.5f;
        glm::vec3 direction = SampleConeHalton(static_cast<int>(halton_idx), to_center, cone_angle);
        trace_single_photon(origin, direction, photon_base_color, 0);
    }
}

void PhotonMap::normalize_photon_power(std::size_t total_emitted_count) {
    if (total_emitted_count == 0) return;
    
    std::size_t stored_count = total_photons();
    std::size_t killed_count = killed_photon_count_.load(std::memory_order_relaxed);
    std::size_t invalid_count = total_emitted_count - stored_count - killed_count;
    
    // Energy conservation: ALL emitted photons share the total light flux equally.
    // - stored photons: energy that reached diffuse surfaces (contributes to caustics)
    // - killed photons: energy absorbed by the medium (no contribution)
    // - invalid photons: energy that missed/escaped (no contribution)
    //
    // Each photon carries: total_flux / total_emitted_count
    // Only stored photons contribute their (attenuated) power to the final image.
    
    // Scaling factor for artistic control (adjust if too bright/dim)
    constexpr FloatType ScalingFactor = 0.1f;
    FloatType factor = (total_light_flux_ / static_cast<FloatType>(total_emitted_count)) * ScalingFactor;
    
    std::cout << std::format(
        "[PhotonMap] Summary:\n"
        "           Emitted: {}\n"
        "           Stored:  {} ({:.1f}%)\n"
        "           Killed:  {} ({:.1f}%)\n"
        "           Invalid: {} ({:.1f}%)\n"
        "           Flux: {:.4f}, Factor: {:.6f}\n",
        total_emitted_count,
        stored_count, 100.0 * stored_count / total_emitted_count,
        killed_count, 100.0 * killed_count / total_emitted_count,
        invalid_count, 100.0 * invalid_count / total_emitted_count,
        total_light_flux_,
        factor
    );
    
    // Scale all photons in photon_map_
    for (auto& [face, photons] : photon_map_) {
        for (auto& p : photons) {
            p.power *= factor;
        }
    }
    
    // Scale all photons in grid_ (same photons, stored by spatial location)
    for (auto& cell : grid_) {
        for (auto& p : cell) {
            p.power *= factor;
        }
    }
}

void PhotonMap::trace_single_photon(
    const glm::vec3& origin,
    const glm::vec3& direction,
    const glm::vec3& power,
    int depth,
    const glm::vec3& medium_entry_point,
    bool interacted_with_transparent
) {
    if (depth >= MaxPhotonBounces) return;

    auto hit_opt = find_intersection(origin, direction);
    if (!hit_opt.has_value()) return;

    const auto& hit = hit_opt.value();
    const Face* hit_face = &world_.all_faces_[hit.triangleIndex];
    const Material& mat = hit_face->material;

    bool is_transparent_surface = IsTransparent(mat);

    if (is_transparent_surface) {
        interacted_with_transparent = true;
        bool currently_inside = glm::length(medium_entry_point) > 0.0f;

        glm::vec3 normal = hit.front_face ? hit.normal : -hit.normal;
        FloatType eta = hit.front_face ? (1.0f / mat.ior) : mat.ior;

        FloatType cos_theta = std::min(glm::dot(-direction, normal), 1.0f);
        FloatType sin_theta = std::sqrt(1.0f - cos_theta * cos_theta);
        bool cannot_refract = eta * sin_theta > 1.0f;

        FloatType r0 = (1.0f - eta) / (1.0f + eta);
        r0 = r0 * r0;
        FloatType reflectance = r0 + (1.0f - r0) * std::pow(1.0f - cos_theta, 5.0f);

        glm::vec3 new_direction;
        glm::vec3 new_power = power;
        glm::vec3 new_entry_point = medium_entry_point;

        if (currently_inside) {
            FloatType travel_dist = glm::length(hit.intersectionPoint - medium_entry_point);

            if (travel_dist > 0.0f && mat.td > 0.0f) {
                glm::vec3 effective_sigma_a;

                if (glm::length(mat.sigma_a) > 0.0f) {
                    effective_sigma_a = mat.sigma_a;
                } else {
                    effective_sigma_a = glm::vec3(
                        -std::log(std::max(mat.base_color.r, 0.001f)) / mat.td,
                        -std::log(std::max(mat.base_color.g, 0.001f)) / mat.td,
                        -std::log(std::max(mat.base_color.b, 0.001f)) / mat.td
                    );
                }

                new_power = glm::vec3(
                    power.x * std::exp(-effective_sigma_a.x * travel_dist),
                    power.y * std::exp(-effective_sigma_a.y * travel_dist),
                    power.z * std::exp(-effective_sigma_a.z * travel_dist)
                );
            }
        }

        // Russian Roulette for early termination of low-power photons
        // We compare against the ORIGINAL power to detect significant absorption.
        // If power drops below a threshold of the original, probabilistically terminate.
        if (depth >= 1) {
            FloatType original_magnitude = glm::length(power);  // power before absorption this bounce
            FloatType new_magnitude = glm::length(new_power);   // power after absorption
            
            // Calculate survival ratio (how much energy remains)
            FloatType survival_ratio = (original_magnitude > 1e-6f) 
                ? (new_magnitude / original_magnitude) 
                : 0.0f;
            
            // If significant absorption occurred, apply RR
            constexpr FloatType RR_Threshold = 0.5f;  // Trigger RR when < 50% survives
            if (survival_ratio < RR_Threshold) {
                // Probability of survival proportional to remaining energy
                FloatType survival_prob = survival_ratio / RR_Threshold;
                if (RandomFloat() > survival_prob) {
                    // Kill the photon - count it as absorbed energy
                    killed_photon_count_.fetch_add(1, std::memory_order_relaxed);
                    return;
                }
                // Photon survives but keeps its attenuated power (no compensation)
            }
        }

        if (cannot_refract) {
            new_direction = glm::reflect(direction, normal);
            new_entry_point = hit.intersectionPoint;
        } else if (RandomFloat() < reflectance * 0.5f) {
            new_direction = glm::reflect(direction, normal);
            if (currently_inside) {
                new_entry_point = hit.intersectionPoint;
            } else {
                new_entry_point = glm::vec3(0.0f);
            }
        } else {
            if (hit.front_face) {
                new_entry_point = hit.intersectionPoint;
            } else {
                new_entry_point = glm::vec3(0.0f);
            }
            new_direction = glm::refract(direction, normal, eta);
        }

        glm::vec3 offset = new_direction * 0.001f;
        trace_single_photon(
            hit.intersectionPoint + offset,
            new_direction,
            new_power,
            depth + 1,
            new_entry_point,
            interacted_with_transparent
        );
    } else {
        glm::vec3 final_power = power;

        bool currently_inside = glm::length(medium_entry_point) > 0.0f;
        if (currently_inside) {
            FloatType travel_dist = glm::length(hit.intersectionPoint - medium_entry_point);

            const Face* last_transparent_face = nullptr;
            for (const auto& face : world_.all_faces_) {
                if (IsTransparent(face.material)) {
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

        if (interacted_with_transparent) {
            store_photon(Photon{hit.intersectionPoint, direction, final_power, hit_face});
        }
    }
}

void PhotonMap::store_photon(const Photon& photon) {
    photon_map_[photon.face].push_back(photon);

    // Store in flattened grid: contiguous vectors per cell minimize pointer
    // chasing and improve memory coherence during neighborhood queries.
    auto [cx, cy, cz] = get_grid_cell(photon.position);
    if (cx < 0 || cy < 0 || cz < 0 || cx >= grid_width_ || cy >= grid_height_ ||
        cz >= grid_depth_) {
        return;  // Out of bounds: skip
    }
    std::size_t idx =
        static_cast<std::size_t>(cx + cy * grid_width_ + cz * grid_width_ * grid_height_);
    grid_[idx].push_back(photon);
}

PhotonMap::GridCell PhotonMap::get_grid_cell(const glm::vec3& position) const noexcept {
    glm::vec3 local = position - grid_origin_;
    int x = static_cast<int>(std::floor(local.x / GridCellSize));
    int y = static_cast<int>(std::floor(local.y / GridCellSize));
    int z = static_cast<int>(std::floor(local.z / GridCellSize));
    return std::make_tuple(x, y, z);
}

std::optional<RayTriangleIntersection> PhotonMap::intersect_triangle(
    const glm::vec3& ro, const glm::vec3& rd, const Face& face
) const noexcept {
    const glm::vec3& v0 = world_.all_vertices_[face.v_indices[0]];
    const glm::vec3& v1 = world_.all_vertices_[face.v_indices[1]];
    const glm::vec3& v2 = world_.all_vertices_[face.v_indices[2]];

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
    auto fetch_normal = [&](int idx) -> glm::vec3 {
        std::uint32_t ni = face.vn_indices[idx];
        if (ni != std::numeric_limits<std::uint32_t>::max() &&
            ni < world_.all_vertex_normals_.size())
            return world_.all_vertex_normals_[ni];
        std::uint32_t vi = face.v_indices[idx];
        if (vi < world_.all_vertex_normals_by_vertex_.size() &&
            glm::length(world_.all_vertex_normals_by_vertex_[vi]) > 0.001f)
            return world_.all_vertex_normals_by_vertex_[vi];
        return face.face_normal;
    };
    glm::vec3 n0 = fetch_normal(0);
    glm::vec3 n1 = fetch_normal(1);
    glm::vec3 n2 = fetch_normal(2);
    hit.normal = glm::normalize(w * n0 + u * n1 + v * n2);

    // Geometric normal
    hit.geom_normal = face.face_normal;

    // Determine front/back face
    hit.front_face = glm::dot(rd, hit.geom_normal) < 0.0f;
    if (!hit.front_face) {
        hit.geom_normal = -hit.geom_normal;
    }

    return hit;
}

std::optional<RayTriangleIntersection> PhotonMap::find_intersection(
    const glm::vec3& ro, const glm::vec3& rd
) const noexcept {
    auto rec = world_.accelerator_.intersect(ro, rd, world_.all_faces_);
    if (rec.triangleIndex == static_cast<std::size_t>(-1)) {
        return std::nullopt;
    }
    return rec;
}
