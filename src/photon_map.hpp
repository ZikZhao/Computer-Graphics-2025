#pragma once
#include <thread>
#include <map>
#include <unordered_map>
#include <vector>
#include <random>
#include <mutex>
#include <atomic>
#include "world.hpp"
#include "shader.hpp"
#include "utils.hpp"

/**
 * @brief Represents a single photon used for caustic estimation.
 */
struct Photon {
    glm::vec3 position;          // 3D position of photon impact
    glm::vec3 direction;         // Incident direction
    glm::vec3 power;             // RGB power/energy of photon
    const Face* face;            // Face where photon landed
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
    // Configuration
    static constexpr int PhotonsPerLight = 200000;     // Number of photons to emit from light source
    static constexpr int MaxPhotonBounces = 5;        // Maximum number of bounces per photon
    static constexpr FloatType MinPhotonPower = 0.01f; // Minimum power threshold for Russian roulette
    static constexpr FloatType CausticSearchRadius = 0.4f;  // Search radius for caustic photon gathering
    static constexpr FloatType GridCellSize = CausticSearchRadius;  // Grid cell size = search radius for exact 27-cell coverage
    
private:
    const World& world_;
    
    // Photon storage: map from Face pointer to vector of photons hitting that face
    std::map<const Face*, std::vector<Photon>> photon_map_;
    
    // Flattened spatial grid for fast indexing
    using GridCell = std::tuple<int, int, int>;
    std::vector<std::vector<Photon>> grid_;
    int grid_width_ = 0;
    int grid_height_ = 0;
    int grid_depth_ = 0;
    glm::vec3 grid_origin_ = glm::vec3(0.0f);
    
    // Threading
    std::jthread worker_thread_;
    std::atomic<bool> is_ready_{false};
    
public:
    /**
     * @brief Constructs and launches the photon tracing worker.
     * @param world World reference used for geometry and materials.
     */
    explicit PhotonMap(const World& world);
    ~PhotonMap() = default;
    
    /**
     * @brief Indicates whether the photon map has finished building.
     * @return True when neighborhood queries are available.
     */
    bool is_ready() const noexcept { return is_ready_.load(std::memory_order_acquire); }
    
    /**
     * @brief Retrieves photons within a radius of a point on a given face.
     * @param face Target surface (used to filter hits).
     * @param point Query point in world space.
     * @param radius Search radius.
     * @return List of nearby photons satisfying the radial and face filter.
     */
    std::vector<Photon> query_photons(const Face* face, const glm::vec3& point, FloatType radius) const;
    
    /**
     * @brief Estimates caustic radiance at a surface point.
     * @param face Surface being shaded.
     * @param point World-space hit position.
     * @param normal Shading normal at the hit.
     * @param search_radius Radius for photon gathering.
     * @return Estimated radiance in HDR space.
     */
    ColourHDR estimate_caustic(const Face* face, const glm::vec3& point, 
                               const glm::vec3& normal, FloatType search_radius) const noexcept;
    
    /**
     * @brief Returns total number of stored photons.
     * @return Photon count across all faces and grid cells.
     */
    std::size_t total_photons() const noexcept;
    
private:
    // Main photon tracing worker (runs in jthread)
    void trace_photons();
    
    // Emit photons from an area light toward transparent targets
    void emit_photons_from_area_light(const Face& light_face, const glm::vec3& target_center, FloatType target_radius, int num_photons);
    
    // Trace a single photon through the scene
    void trace_single_photon(const glm::vec3& origin, const glm::vec3& direction, 
                             const glm::vec3& power, int depth, 
                             const glm::vec3& medium_entry_point = glm::vec3(0.0f),
                             bool interacted_with_transparent = false);
    
    // Store a photon in the map
    void store_photon(const Photon& photon);
    
    // Compute grid cell coordinates for a 3D position (relative to grid_origin_)
    GridCell GetGridCell(const glm::vec3& position) const noexcept;
    
    // Check if material is transparent (refractive)
    static bool IsTransparent(const Material& mat) noexcept {
        return mat.tw > 0.0f && mat.ior != 1.0f;
    }
    
    // Ray-triangle intersection (simplified from RayTracer)
    std::optional<RayTriangleIntersection> intersect_triangle(
        const glm::vec3& ro, const glm::vec3& rd, const Face& face) const noexcept;
    
    // Find closest intersection in scene
    std::optional<RayTriangleIntersection> find_intersection(
        const glm::vec3& ro, const glm::vec3& rd) const noexcept;
    
    /**
     * @brief Random float helper for stochastic decisions.
     * @param min Inclusive lower bound.
     * @param max Exclusive upper bound.
     * @return Random value in \[min, max).
     */
    static FloatType RandomFloat(FloatType min = 0.0f, FloatType max = 1.0f) noexcept;
};
