#pragma once
#include <atomic>
#include <thread>
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
 * @class PhotonMap
 * @brief Photon mapping accelerator for estimating caustic radiance.
 *
 * Builds a spatial index of photons traced from emissive surfaces and supports
 * neighborhood queries to reconstruct radiance at hit points.
 */
class PhotonMap {
public:
    using GridCell = std::tuple<int, int, int>;

public:
    /// @brief Checks if material is transparent (refractive)
    static constexpr bool IsTransparent(const Material& mat) noexcept {
        return mat.tw > 0.0f && mat.ior != 1.0f;
    }

private:
    const World& world_;
    std::vector<std::vector<Photon>> grid_;
    int grid_width_ = 0;
    int grid_height_ = 0;
    int grid_depth_ = 0;
    glm::vec3 grid_origin_ = glm::vec3(0.0f);
    FloatType total_light_flux_ = 0.0f;  // Total scene flux for energy normalization

    std::jthread worker_thread_;
    std::atomic<bool> is_ready_{false};

public:
    explicit PhotonMap(const World& world);

public:
    [[nodiscard]] bool is_ready() const noexcept {
        return is_ready_.load(std::memory_order_acquire);
    }

    [[nodiscard]] const std::vector<std::vector<Photon>>& grid() const noexcept { return grid_; }

    /**
     * @brief Retrieves photons within a radius of a point.
     * @param point Query point in world space.
     * @param radius Search radius.
     * @return List of nearby photons satisfying the radial filter.
     */
    [[nodiscard]] std::vector<Photon> query_photons(const glm::vec3& point, FloatType radius) const;

    /**
     * @brief Estimates caustic radiance at a surface point.
     * @param point World-space hit position.
     * @param normal Shading normal at the hit.
     * @param search_radius Radius for photon gathering.
     * @return Estimated radiance in HDR space.
     */
    [[nodiscard]] ColourHDR estimate_caustic(
        const glm::vec3& point, const glm::vec3& normal, FloatType search_radius
    ) const noexcept;

private:
    /**
     * @brief Builds the photon map in a background thread.
     * @param st Stop token to allow cooperative cancellation.
     *
     * This function emits photons from emissive surfaces, traces them through the scene,
     * and stores them in a spatial grid for later querying.
     */
    void build_photon_map(std::stop_token st);

    /**
     * @brief Emits a batch of photons from an area light.
     * @return Number of photons stored from this batch.
     */
    [[nodiscard]] std::size_t emit_photon_batch(
        const Face& light_face,
        const glm::vec3& target_center,
        FloatType target_radius,
        std::size_t batch_start_index,
        std::size_t batch_size
    );

    /**
     * @brief Normalizes all stored photon powers based on total emitted count.
     * @param total_emitted_count Number of photons that were attempted/emitted.
     */
    void normalize_photon_power(std::size_t total_emitted_count);

    /**
     * @brief Traces a single photon through the scene.
     * @return true if a photon was stored, false otherwise.
     */
    [[nodiscard]] bool trace_single_photon(
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
