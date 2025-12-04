#pragma once
#include <chrono>
#include <cstddef>
#include <numbers>

/**
 * @brief Centralized constants for the renderer.
 */
namespace Constant {

// Window / Display
inline constexpr std::size_t WindowWidth = 1920;
inline constexpr std::size_t WindowHeight = 1080;
inline constexpr auto TargetFrameTime = std::chrono::microseconds(15000);  // ~66 FPS cap

// Camera
inline constexpr auto OrbitInterval = std::chrono::seconds(1) / 60;
inline constexpr double FOV = 45.0;
inline constexpr double NearPlane = 0.001;
inline constexpr double FarPlane = 100.0;
inline constexpr float MaxPitch = 1.553343f;              // glm::radians(89.0f)
inline constexpr float OrbitAngleIncrement = -0.004363f;  // glm::radians(-0.25f)

// Input
inline constexpr float MoveSpeed = 3.0f;
inline constexpr float RollSpeed = 0.5f;
inline constexpr float MouseSensitivity = 0.002f;
inline constexpr float RotateSpeed = 0.2f;
inline constexpr float ZoomFactor = 1.1f;
inline constexpr float GammaStep = 0.01f;

// Renderer
inline constexpr int TileSize = 32;      // Tile-based rendering block size
inline constexpr int VideoSamples = 64;  // SPP for video recording mode
inline constexpr int TargetFPS = 60;     // Target FPS for video recording
inline constexpr float DefaultGamma = 2.2f;

// Ray Tracing
inline constexpr int MaxRayDepth = 64;             // Absolute max recursion depth
inline constexpr float RayEpsilon = 0.001f;        // Offset to avoid self-intersection
inline constexpr float Epsilon = 1e-6f;            // General numerical epsilon
inline constexpr float AmbientIntensity = 0.025f;  // Ambient light contribution

// Photon Mapping
inline constexpr std::size_t TargetStoredPhotons = 50000;     // Target caustic photons
inline constexpr std::size_t MaxEmittedPhotons = 10'000'000;  // Safety bailout
inline constexpr std::size_t PhotonBatchSize = 5000;
inline constexpr int MaxPhotonBounces = 5;
inline constexpr float MinPhotonPower = 0.01f;
inline constexpr float CausticSearchRadius = 0.4f;
inline constexpr float PhotonGridCellSize = CausticSearchRadius;
inline constexpr float RussianRouletteThreshold = 0.5f;
inline constexpr float PhotonVisualizationExposure = 10000.0f;
inline constexpr float ConeFilterK = 1.1f;  // Cone filter constant for density estimation

// BVH
inline constexpr float SAHTraversalCost = 1.0f;
inline constexpr float SAHIntersectionCost = 1.0f;
inline constexpr int SAHBuckets = 12;
inline constexpr int BVHLeafThreshold = 4;

// Auto Exposure
inline constexpr float TargetMiddleGray = 0.3f;

}  // namespace Constant
