#pragma once
#include <atomic>
#include <map>
#include <mutex>
#include <random>
#include <thread>
#include <unordered_map>
#include <vector>

#include "utils.hpp"
#include "world.hpp"

/**
 * @brief Represents a single photon used for caustic estimation.
 */
struct Photon {
    glm::vec3 position;   // 3D position of photon impact
    glm::vec3 direction;  // Incident direction
    glm::vec3 power;      // RGB power/energy of photon
    const Face* face;     // Face where photon landed
};

/**
 * @brief Hash functor for integer 3D grid coordinates.
 */
struct GridCellHash {
    std::size_t operator()(const std::tuple<int, int, int>& cell) const noexcept {
        auto h1 = std::hash<int>{}(std::get<0>(cell));
        auto h2 = std::hash<int>{}(std::get<1>(cell));
        auto h3 = std::hash<int>{}(std::get<2>(cell));
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

/**
 * @brief Photon mapping accelerator for estimating caustic radiance.
 *
 * Builds a spatial index of photons traced from emissive surfaces and supports
 * neighborhood queries to reconstruct radiance at hit points.
 */
class PhotonMap {
public:
    using GridCell = std::tuple<int, int, int>;

public:
    static constexpr std::size_t TargetStoredPhotons = 50000;   // Target number of stored caustic photons
    static constexpr std::size_t MaxEmittedPhotons = 10000000;  // Safety bailout to prevent infinite loops
    static constexpr std::size_t BatchSize = 5000;              // Photons emitted per batch iteration
    static constexpr int MaxPhotonBounces = 5;
    static constexpr FloatType MinPhotonPower = 0.01f;
    static constexpr FloatType CausticSearchRadius = 0.4f;
    static constexpr FloatType GridCellSize = CausticSearchRadius;

    /// Check if material is transparent (refractive)
    static constexpr bool IsTransparent(const Material& mat) noexcept {
        return mat.tw > 0.0f && mat.ior != 1.0f;
    }

    /**
     * @brief Random float helper for stochastic decisions.
     * @param min Inclusive lower bound.
     * @param max Exclusive upper bound.
     * @return Random value in [min, max).
     */
    static FloatType RandomFloat(FloatType min = 0.0f, FloatType max = 1.0f) noexcept;

private:
    const World& world_;
    std::map<const Face*, std::vector<Photon>> photon_map_;
    std::vector<std::vector<Photon>> grid_;
    int grid_width_ = 0;
    int grid_height_ = 0;
    int grid_depth_ = 0;
    glm::vec3 grid_origin_ = glm::vec3(0.0f);
    FloatType total_light_flux_ = 0.0f;  // Total scene flux for energy normalization
    std::atomic<std::size_t> killed_photon_count_{0};  // Photons terminated by RR (absorbed energy)

    std::jthread worker_thread_;
    std::atomic<bool> is_ready_{false};

public:
    explicit PhotonMap(const World& world);

public:
    [[nodiscard]] bool is_ready() const noexcept {
        return is_ready_.load(std::memory_order_acquire);
    }

    /**
     * @brief Returns total number of stored photons.
     * @return Photon count across all faces and grid cells.
     */
    [[nodiscard]] std::size_t total_photons() const noexcept;

    /**
     * @brief Retrieves photons within a radius of a point on a given face.
     * @param face Target surface (used to filter hits).
     * @param point Query point in world space.
     * @param radius Search radius.
     * @return List of nearby photons satisfying the radial and face filter.
     */
    [[nodiscard]] std::vector<Photon> query_photons(
        const Face* face, const glm::vec3& point, FloatType radius
    ) const;

    /**
     * @brief Helper to iterate all photons for debug visualization.
     * @tparam Func Callable taking a const Photon& parameter.
     * @param func Function to call for each stored photon.
     */
    template <typename Func>
    void for_each_photon(Func func) const {
        for (const auto& [face, photons] : photon_map_) {
            for (const auto& p : photons) {
                func(p);
            }
        }
    }

    /**
     * @brief Estimates caustic radiance at a surface point.
     * @param face Surface being shaded.
     * @param point World-space hit position.
     * @param normal Shading normal at the hit.
     * @param search_radius Radius for photon gathering.
     * @return Estimated radiance in HDR space.
     */
    [[nodiscard]] ColourHDR estimate_caustic(
        const Face* face, const glm::vec3& point, const glm::vec3& normal, FloatType search_radius
    ) const noexcept;

private:
    void trace_photons();

    /**
     * @brief Emits a batch of photons from an area light.
     * @param light_face The emissive face.
     * @param target_center Center of the transparent objects AABB.
     * @param target_radius Radius encompassing transparent objects.
     * @param batch_start_index Starting index for Halton sequence.
     * @param batch_size Number of photons to emit in this batch.
     * @param weight Relative weight of this light source.
     * @param weight_sum Total weight of all light sources.
     */
    void emit_photon_batch(
        const Face& light_face,
        const glm::vec3& target_center,
        FloatType target_radius,
        std::size_t batch_start_index,
        std::size_t batch_size,
        FloatType weight,
        FloatType weight_sum
    );

    /**
     * @brief Normalizes all stored photon powers based on total emitted count.
     * @param total_emitted_count Number of photons that were attempted/emitted.
     */
    void normalize_photon_power(std::size_t total_emitted_count);

    void trace_single_photon(
        const glm::vec3& origin,
        const glm::vec3& direction,
        const glm::vec3& power,
        int depth,
        const glm::vec3& medium_entry_point = glm::vec3(0.0f),
        bool interacted_with_transparent = false
    );

    void store_photon(const Photon& photon);

    GridCell get_grid_cell(const glm::vec3& position) const noexcept;

    std::optional<RayTriangleIntersection> intersect_triangle(
        const glm::vec3& ro, const glm::vec3& rd, const Face& face
    ) const noexcept;

    std::optional<RayTriangleIntersection> find_intersection(
        const glm::vec3& ro, const glm::vec3& rd
    ) const noexcept;
};
