#include "photon_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <iostream>

#include "constants.hpp"

PhotonMap::PhotonMap(const World& world) : world_(world) {
    worker_thread_ = std::jthread([this](std::stop_token st) { build_photon_map(st); });
}

std::vector<Photon> PhotonMap::query_photons(const glm::vec3& point, FloatType radius) const {
    std::vector<Photon> result;
    GridCell center_cell = get_grid_cell(point);
    auto [cx, cy, cz] = center_cell;
    int cell_range = static_cast<int>(std::ceil(radius / Constant::PhotonGridCellSize)) + 1;
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
    for (const auto& photon : photons) {
        FloatType alignment = glm::dot(photon.direction, normal);
        if (alignment > 0.0f) continue;
        glm::vec3 diff = photon.position - point;
        FloatType dist = glm::length(diff);
        FloatType weight = std::max(0.0f, 1.0f - (dist / (Constant::ConeFilterK * search_radius)));
        FloatType cos_theta = std::max(0.0f, -alignment);
        accumulated_flux += photon.power * (cos_theta * weight);
    }
    FloatType search_area = std::numbers::pi * search_radius * search_radius;
    glm::vec3 radiance = accumulated_flux / search_area;
    return ColourHDR{.red = radiance.x, .green = radiance.y, .blue = radiance.z};
}

void PhotonMap::build_photon_map(std::stop_token st) {
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
            transparent_aabb_min =
                glm::min(transparent_aabb_min, world_.all_vertices_[face->v_indices[k]]);
            transparent_aabb_max =
                glm::max(transparent_aabb_max, world_.all_vertices_[face->v_indices[k]]);
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

    // Target center/radius for emission cone (based on transparent objects)
    glm::vec3 target_center = (transparent_aabb_min + transparent_aabb_max) * 0.5f;
    FloatType target_radius = glm::length(transparent_aabb_max - target_center);

    // Spatial indexing: use entire scene bounds for grid
    // This ensures photons landing on floors/walls are properly stored
    grid_origin_ = scene_aabb_min - glm::vec3(Constant::PhotonGridCellSize);  // Add padding
    glm::vec3 extent =
        (scene_aabb_max - scene_aabb_min) + glm::vec3(2.0f * Constant::PhotonGridCellSize);
    grid_width_ = std::max(1, static_cast<int>(std::ceil(extent.x / Constant::PhotonGridCellSize)));
    grid_height_ =
        std::max(1, static_cast<int>(std::ceil(extent.y / Constant::PhotonGridCellSize)));
    grid_depth_ = std::max(1, static_cast<int>(std::ceil(extent.z / Constant::PhotonGridCellSize)));
    std::size_t grid_size = static_cast<std::size_t>(grid_width_) *
                            static_cast<std::size_t>(grid_height_) *
                            static_cast<std::size_t>(grid_depth_);
    grid_.clear();
    grid_.resize(grid_size);

    // Compute total scene flux and relative probability weights for each light
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
        FloatType flux = area * Luminance(Le) * std::numbers::pi_v<FloatType>;
        total_light_flux_ += flux;
        FloatType w = std::max<FloatType>(1e-6f, flux);
        weights[i] = w;
        weight_sum += w;
    }

    // Keep emitting batches until we reach TargetStoredPhotons or hit the safety limit
    std::size_t total_emitted_count = 0;
    std::size_t total_stored_count = 0;
    std::size_t batch_index = 0;

    while (total_stored_count < Constant::TargetStoredPhotons &&
           total_emitted_count < Constant::MaxEmittedPhotons) {
        // Emit a batch distributed across all light sources
        for (std::size_t i = 0; i < emissive_faces.size(); ++i) {
            std::size_t photons_for_light = static_cast<std::size_t>(
                static_cast<FloatType>(Constant::PhotonBatchSize) * (weights[i] / weight_sum)
            );
            if (photons_for_light == 0) photons_for_light = 1;

            total_stored_count += emit_photon_batch(
                *emissive_faces[i],
                target_center,
                target_radius,
                batch_index * Constant::PhotonBatchSize + total_emitted_count,
                photons_for_light
            );
            total_emitted_count += photons_for_light;
        }
        batch_index++;

        if (batch_index % 100 == 0) {
            std::cout << std::format(
                "[PhotonMap] Progress: Stored {} / {} target | Emitted: {}\n",
                total_stored_count,
                Constant::TargetStoredPhotons,
                total_emitted_count
            );
        }

        if (st.stop_requested()) {
            std::cout << "[PhotonMap] Stopping early due to stop request\n";
            break;
        }
    }

    // Rescale stored photon powers based on total emitted flux
    normalize_photon_power(total_emitted_count);

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    std::cout << std::format(
        "[PhotonMap] Completed in {:#.3g}s | Stored {} / {} ({:.1f}%)\n",
        elapsed.count(),
        total_stored_count,
        total_emitted_count,
        100.0 * static_cast<double>(total_stored_count) / static_cast<double>(total_emitted_count)
    );
    is_ready_.store(true, std::memory_order_release);
}

std::size_t PhotonMap::emit_photon_batch(
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

    // Compute the light center for cone angle calculation (average of all sample points)
    glm::vec3 light_center =
        world_.all_vertices_[light_face.v_indices[0]] + (e0 + e1) / 3.0f;  // Triangle centroid
    FloatType avg_dist = glm::length(target_center - light_center);
    FloatType raw_cone_angle =
        std::atan(target_radius / std::max<FloatType>(avg_dist, 1e-4f)) * 1.5f;
    // Clamp cone angle to 90° to prevent photons from emitting backwards
    FloatType cone_angle = std::min(raw_cone_angle, std::numbers::pi_v<FloatType> / 2.0f);

    // Scale by solid angle fraction (1 - cos(theta)) to avoid brightness amplification in narrow
    // cones
    FloatType cone_fraction = std::min(1.0f - std::cos(cone_angle), 1.0f);

    // Use normalized color tint only -- total_light_flux_ already contains Le magnitude,
    // so multiplying by Le again would cause Le² scaling. Power normalization happens later.
    FloatType lum = Luminance(Le);
    glm::vec3 normalized_tint = (lum > 1e-6f) ? (Le / lum) : glm::vec3(1.0f);
    glm::vec3 photon_base_color = normalized_tint * cone_fraction;

    std::size_t stored_count = 0;
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
        glm::vec3 direction = SampleConeHalton(static_cast<int>(halton_idx), to_center, cone_angle);
        if (trace_single_photon(origin, direction, photon_base_color, 0)) {
            ++stored_count;
        }
    }
    return stored_count;
}

void PhotonMap::normalize_photon_power(std::size_t total_emitted_count) {
    if (total_emitted_count == 0) return;

    FloatType factor = total_light_flux_ / static_cast<FloatType>(total_emitted_count);

    for (auto& cell : grid_) {
        for (auto& p : cell) {
            p.power *= factor;
        }
    }
}

bool PhotonMap::trace_single_photon(
    const glm::vec3& origin,
    const glm::vec3& direction,
    const glm::vec3& power,
    int depth,
    const glm::vec3& medium_entry_point,
    bool interacted_with_transparent
) {
    if (depth >= Constant::MaxPhotonBounces) return false;

    auto hit_opt = find_intersection(origin, direction);
    if (!hit_opt.has_value()) return false;

    const auto& hit = hit_opt.value();
    const Face* hit_face = &world_.all_faces_[hit.triangleIndex];
    const Material& mat = hit_face->material;

    bool is_transparent_surface = IsTransparent(mat);

    if (is_transparent_surface) {
        // Handle transparent (refractive) surface
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
                glm::vec3 sigma_a = EffectiveSigmaA(mat.sigma_a, mat.base_color, mat.td);
                new_power = power * BeerLambert(sigma_a, travel_dist);
            }
        }

        // Russian Roulette: probabilistically terminate photons with significant absorption
        if (depth >= 1) {
            FloatType survival_ratio = glm::length(new_power) / glm::length(power);
            if (survival_ratio < Constant::RussianRouletteThreshold) {
                if (Rand() > survival_ratio / Constant::RussianRouletteThreshold) {
                    return false;
                }
            }
        }

        if (cannot_refract) {
            new_direction = glm::reflect(direction, normal);
            new_entry_point = hit.intersectionPoint;
        } else if (Rand() < reflectance * 0.5f) {
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
        return trace_single_photon(
            hit.intersectionPoint + offset,
            new_direction,
            new_power,
            depth + 1,
            new_entry_point,
            interacted_with_transparent
        );
    } else {
        // Absorptive or diffuse surface: store photon if we interacted with transparent
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
                glm::vec3 sigma_a = EffectiveSigmaA(mat.sigma_a, mat.base_color, mat.td);
                final_power = power * BeerLambert(sigma_a, travel_dist);
            }
        }

        if (interacted_with_transparent) {
            store_photon(Photon{hit.intersectionPoint, direction, final_power, hit_face});
            return true;
        }
        return false;
    }
}

void PhotonMap::store_photon(const Photon& photon) {
    auto [cx, cy, cz] = get_grid_cell(photon.position);
    std::size_t idx =
        static_cast<std::size_t>(cx + cy * grid_width_ + cz * grid_width_ * grid_height_);
    grid_[idx].push_back(photon);
}

PhotonMap::GridCell PhotonMap::get_grid_cell(const glm::vec3& position) const noexcept {
    glm::vec3 local = position - grid_origin_;
    int x = static_cast<int>(std::floor(local.x / Constant::PhotonGridCellSize));
    int y = static_cast<int>(std::floor(local.y / Constant::PhotonGridCellSize));
    int z = static_cast<int>(std::floor(local.z / Constant::PhotonGridCellSize));
    return PhotonMap::GridCell(x, y, z);
}

std::optional<RayTriangleIntersection> PhotonMap::intersect_triangle(
    const glm::vec3& ro, const glm::vec3& rd, const Face& face
) const noexcept {
    const glm::vec3& v0 = world_.all_vertices_[face.v_indices[0]];
    const glm::vec3& v1 = world_.all_vertices_[face.v_indices[1]];
    const glm::vec3& v2 = world_.all_vertices_[face.v_indices[2]];

    // Möller-Trumbore intersection algorithm
    glm::vec3 edge1 = v1 - v0;
    glm::vec3 edge2 = v2 - v0;
    glm::vec3 h = glm::cross(rd, edge2);
    FloatType a = glm::dot(edge1, h);

    if (std::abs(a) < Constant::Epsilon) {
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

    if (t < Constant::Epsilon) {
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

    // Determine front/back face
    hit.geom_normal = face.face_normal;
    hit.front_face = glm::dot(rd, hit.geom_normal) < 0.0f;
    if (!hit.front_face) {
        hit.geom_normal = -hit.geom_normal;
    }

    return hit;
}

std::optional<RayTriangleIntersection> PhotonMap::find_intersection(
    const glm::vec3& ro, const glm::vec3& rd
) const noexcept {
    auto rec = world_.accelerator_->intersect(ro, rd, world_.all_faces_);
    if (rec.triangleIndex == static_cast<std::size_t>(-1)) {
        return std::nullopt;
    }
    return rec;
}
