#pragma once
#include <chrono>
#include <cstddef>
#include <numbers>

#include "utils.hpp"

/**
 * @brief Centralized constants for the renderer.
 */
namespace Constant {

// Window / Display
inline constexpr std::size_t WindowWidth = 640;
inline constexpr std::size_t WindowHeight = 480;
inline constexpr auto TargetFrameTime = std::chrono::microseconds(15000);  // ~66 FPS cap

// Camera
inline constexpr auto OrbitInterval = std::chrono::seconds(1) / 60;
inline constexpr double FOV = 45.0;
inline constexpr double NearPlane = 0.001;
inline constexpr double FarPlane = 100.0;
inline constexpr FloatType MaxPitch = 1.553343f;              // glm::radians(89.0f)
inline constexpr FloatType OrbitAngleIncrement = -0.004363f;  // glm::radians(-0.25f)

// Input
inline constexpr FloatType MoveSpeed = 3.0f;
inline constexpr FloatType RollSpeed = 0.5f;
inline constexpr FloatType MouseSensitivity = 0.002f;
inline constexpr FloatType RotateSpeed = 0.2f;
inline constexpr FloatType ZoomFactor = 1.1f;
inline constexpr FloatType GammaStep = 0.01f;

// Renderer
inline constexpr int TileSize = 32;      // Tile-based rendering block size
inline constexpr int VideoSamples = 64;  // SPP for video recording mode
inline constexpr int TargetFPS = 60;     // Target FPS for video recording
inline constexpr FloatType DefaultGamma = 2.2f;

// Ray Tracing
inline constexpr int MaxRayDepth = 64;           // Absolute max recursion depth
inline constexpr FloatType RayEpsilon = 0.001f;  // Offset to avoid self-intersection
inline constexpr FloatType Epsilon = 1e-6f;      // General numerical epsilon

// Photon Mapping
inline constexpr std::size_t TargetStoredPhotons = 50000;     // Target caustic photons
inline constexpr std::size_t MaxEmittedPhotons = 10'000'000;  // Safety bailout
inline constexpr std::size_t PhotonBatchSize = 5000;
inline constexpr int MaxPhotonBounces = 5;
inline constexpr FloatType MinPhotonPower = 0.01f;
inline constexpr FloatType CausticSearchRadius = 0.4f;
inline constexpr FloatType PhotonGridCellSize = CausticSearchRadius;
inline constexpr FloatType RussianRouletteThreshold = 0.5f;
inline constexpr FloatType PhotonVisualizationExposure = 10000.0f;
inline constexpr FloatType ConeFilterK = 1.1f;  // Cone filter constant for density estimation

// BVH
inline constexpr FloatType SAHTraversalCost = 1.0f;
inline constexpr FloatType SAHIntersectionCost = 1.0f;
inline constexpr int SAHBuckets = 12;
inline constexpr int BVHLeafThreshold = 4;

// Auto Exposure
inline constexpr FloatType TargetMiddleGray = 0.3f;

}  // namespace Constant
