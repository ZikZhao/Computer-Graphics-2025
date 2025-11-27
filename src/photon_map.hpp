#pragma once
#include "world.hpp"
#include "shader.hpp"
#include "utils.hpp"
#include <thread>
#include <map>
#include <unordered_map>
#include <vector>
#include <random>
#include <mutex>
#include <atomic>

// Structure representing a single photon
struct Photon {
    glm::vec3 position;          // 3D position of photon impact
    glm::vec3 direction;         // Incident direction
    glm::vec3 power;             // RGB power/energy of photon
    const Face* face;            // Face where photon landed
    
    Photon() noexcept = default;
    Photon(const glm::vec3& pos, const glm::vec3& dir, const glm::vec3& pwr, const Face* f) noexcept
        : position(pos), direction(dir), power(pwr), face(f) {}
};

// Hash function for 3D grid coordinates
struct GridCellHash {
    std::size_t operator()(const std::tuple<int, int, int>& cell) const noexcept {
        auto h1 = std::hash<int>{}(std::get<0>(cell));
        auto h2 = std::hash<int>{}(std::get<1>(cell));
        auto h3 = std::hash<int>{}(std::get<2>(cell));
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

// PhotonMap class for caustics computation
class PhotonMap {
public:
    // Configuration
    static constexpr int PHOTONS_PER_LIGHT = 200000;     // Number of photons to emit from light source
    static constexpr int MAX_PHOTON_BOUNCES = 5;        // Maximum number of bounces per photon
    static constexpr FloatType MIN_PHOTON_POWER = 0.01f; // Minimum power threshold for Russian roulette
    static constexpr FloatType CAUSTIC_SEARCH_RADIUS = 0.4f;  // Search radius for caustic photon gathering
    static constexpr FloatType GRID_CELL_SIZE = CAUSTIC_SEARCH_RADIUS;  // Grid cell size = search radius for exact 27-cell coverage
    
private:
    const World& world_;
    
    // Photon storage: map from Face pointer to vector of photons hitting that face
    std::map<const Face*, std::vector<Photon>> photon_map_;
    
    // Spatial hash grid: maps grid cell coordinates to photons in that cell
    // Key is (x_cell, y_cell, z_cell) tuple
    using GridCell = std::tuple<int, int, int>;
    std::unordered_map<GridCell, std::vector<Photon>, GridCellHash> spatial_grid_;
    
    // Threading
    std::jthread worker_thread_;
    std::atomic<bool> is_ready_{false};
    
public:
    explicit PhotonMap(const World& world);
    ~PhotonMap() = default;
    
    // Check if photon map is ready for queries
    bool is_ready() const noexcept { return is_ready_.load(std::memory_order_acquire); }
    
    // Query photons near a point on a given face
    std::vector<Photon> query_photons(const Face* face, const glm::vec3& point, FloatType radius) const;
    
    // Estimate caustic radiance at a point
    ColourHDR estimate_caustic(const Face* face, const glm::vec3& point, 
                               const glm::vec3& normal, FloatType search_radius) const noexcept;
    
    // Get total number of stored photons
    std::size_t total_photons() const noexcept;
    
private:
    // Main photon tracing worker (runs in jthread)
    void trace_photons();
    
    // Emit photons from legacy point light toward transparent objects
    void emit_photons_to_object(const Face& target_face, int num_photons);

    // Emit photons from an area light toward transparent targets
    void emit_photons_from_area_light(const Face& light_face, const glm::vec3& target_center, FloatType target_radius, int num_photons);
    
    // Trace a single photon through the scene
    void trace_single_photon(const glm::vec3& origin, const glm::vec3& direction, 
                             const glm::vec3& power, int depth, 
                             const glm::vec3& medium_entry_point = glm::vec3(0.0f),
                             bool interacted_with_transparent = false);
    
    // Store a photon in the map
    void store_photon(const Photon& photon);
    
    // Compute grid cell coordinates for a 3D position
    static GridCell get_grid_cell(const glm::vec3& position) noexcept;
    
    // Check if material is transparent (refractive)
    static bool is_transparent(const Material& mat) noexcept {
        return mat.tw > 0.0f && mat.ior != 1.0f;
    }
    
    // Ray-triangle intersection (simplified from RayTracer)
    static std::optional<RayTriangleIntersection> intersect_triangle(
        const glm::vec3& ro, const glm::vec3& rd, const Face& face) noexcept;
    
    // Find closest intersection in scene
    std::optional<RayTriangleIntersection> find_intersection(
        const glm::vec3& ro, const glm::vec3& rd) const noexcept;
    
    // Random number generation for stochastic decisions
    static FloatType random_float(FloatType min = 0.0f, FloatType max = 1.0f) noexcept;
};
