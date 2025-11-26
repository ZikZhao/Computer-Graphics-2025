#pragma once
#include <glm/glm.hpp>
#include <array>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <fstream>
#include <string>
#include <cstdint>
#include <vector>
#include <thread>
#include <iostream>

using FloatType = decltype(std::declval<glm::vec3>().x);

// ============================================================================
// Math Utilities
// ============================================================================

constexpr FloatType ComputeInvZndc(FloatType progress, std::array<FloatType, 2> vertices_z_ndc) noexcept {
    return (1.0f - progress) / vertices_z_ndc[0] + progress / vertices_z_ndc[1];
}

constexpr FloatType ComputeInvZndc(std::array<FloatType, 3> bary, std::array<FloatType, 3> vertices_z_ndc) noexcept {
    return bary[0] / vertices_z_ndc[0] +
           bary[1] / vertices_z_ndc[1] +
           bary[2] / vertices_z_ndc[2];
}

// ============================================================================
// Container Utilities
// ============================================================================

template<typename T, std::size_t N>
class InplaceVector {
private:
    alignas(T) std::byte data_[N * sizeof(T)];
    std::size_t size_ = 0;
public:
    constexpr InplaceVector() noexcept = default;
    constexpr InplaceVector(auto&&... args) noexcept : InplaceVector() {
        (emplace_back(std::forward<decltype(args)>(args)), ...);
    }
    constexpr void push_back(const T& value) noexcept {
        assert(size_ < N);
        new (&data_[size_ * sizeof(T)]) T(value);
        ++size_;
    }
    constexpr void emplace_back(T&& value) noexcept {
        assert(size_ < N);
        new (&data_[size_ * sizeof(T)]) T(std::move(value));
        ++size_;
    }
    constexpr std::size_t size() const noexcept {
        return size_;
    }
    constexpr T& operator[](std::size_t index) noexcept {
        return *reinterpret_cast<T*>(&data_[index * sizeof(T)]);
    }
    constexpr const T& operator[](std::size_t index) const noexcept {
        return *reinterpret_cast<const T*>(&data_[index * sizeof(T)]);
    }
};

// ============================================================================
// Ray Tracing Utilities
// ============================================================================

// Calculate surface normal from triangle vertices
inline glm::vec3 CalculateNormal(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2) noexcept {
    glm::vec3 edge1 = v1 - v0;
    glm::vec3 edge2 = v2 - v0;
    return glm::normalize(glm::cross(edge1, edge2));
}

inline bool IntersectRayTriangle(
    const glm::vec3& ray_origin,
    const glm::vec3& ray_dir,
    const glm::vec3& v0,
    const glm::vec3& v1,
    const glm::vec3& v2,
    FloatType& out_t,
    FloatType& out_u,
    FloatType& out_v
) noexcept {
    constexpr FloatType EPSILON = 1e-6f;
    
    glm::vec3 edge1 = v1 - v0;
    glm::vec3 edge2 = v2 - v0;
    glm::vec3 h = glm::cross(ray_dir, edge2);
    FloatType a = glm::dot(edge1, h);
    
    // Ray is parallel to triangle
    if (std::abs(a) < EPSILON) {
        return false;
    }
    
    FloatType f = 1.0f / a;
    glm::vec3 s = ray_origin - v0;
    FloatType u = f * glm::dot(s, h);
    
    if (u < 0.0f || u > 1.0f) {
        return false;
    }
    
    glm::vec3 q = glm::cross(s, edge1);
    FloatType v = f * glm::dot(ray_dir, q);
    
    if (v < 0.0f || u + v > 1.0f) {
        return false;
    }
    
    FloatType t = f * glm::dot(edge2, q);
    
    out_t = t;
    out_u = u;
    out_v = v;
    
    return t > EPSILON;
}

inline glm::vec3 CalculateBarycentric(glm::vec2 v0, glm::vec2 v1, glm::vec2 v2, glm::vec2 p) noexcept {
    glm::vec2 e0 = v1 - v0;
    glm::vec2 e1 = v2 - v0;
    glm::vec2 e2 = p - v0;
    FloatType denominator = e0.x * e1.y - e0.y * e1.x;
    FloatType invDenominator = 1.0f / denominator;
    FloatType weight_v1 = (e2.x * e1.y - e2.y * e1.x) * invDenominator;
    FloatType weight_v2 = (e0.x * e2.y - e0.y * e2.x) * invDenominator;
    FloatType weight_v0 = 1.0f - weight_v1 - weight_v2;
    return glm::vec3(weight_v0, weight_v1, weight_v2);
}

// ============================================================================
// Low-Discrepancy Sampling (Halton Sequence)
// ============================================================================

inline FloatType halton(int index, int base) noexcept {
    FloatType result = 0.0f;
    FloatType f = 1.0f / base;
    int i = index;
    while (i > 0) {
        result += f * (i % base);
        i = i / base;
        f = f / base;
    }
    return result;
}

inline glm::vec3 sample_sphere_halton(int index, FloatType radius, const glm::vec3& center) noexcept {
    FloatType u = halton(index, 2);
    FloatType v = halton(index, 3);
    
    FloatType theta = 2.0f * std::numbers::pi * u;
    FloatType phi = std::acos(2.0f * v - 1.0f);
    
    FloatType sin_phi = std::sin(phi);
    FloatType x = radius * sin_phi * std::cos(theta);
    FloatType y = radius * sin_phi * std::sin(theta);
    FloatType z = radius * std::cos(phi);
    
    return center + glm::vec3(x, y, z);
}

inline glm::vec3 sample_unit_vector_halton(int index) noexcept {
    return sample_sphere_halton(index, 1.0f, glm::vec3(0.0f));
}

inline glm::vec3 sample_cone_halton(int index, const glm::vec3& direction, FloatType cone_angle) noexcept {
    FloatType u1 = halton(index, 2);
    FloatType u2 = halton(index, 3);
    
    FloatType cos_angle = std::cos(cone_angle);
    FloatType z = cos_angle + (1.0f - cos_angle) * u1;
    FloatType phi = 2.0f * std::numbers::pi * u2;
    
    FloatType sin_theta = std::sqrt(1.0f - z * z);
    glm::vec3 sample_dir(sin_theta * std::cos(phi), sin_theta * std::sin(phi), z);
    
    // Build orthonormal basis around direction
    glm::vec3 up = std::abs(direction.y) < 0.999f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    glm::vec3 right = glm::normalize(glm::cross(up, direction));
    glm::vec3 forward = glm::cross(direction, right);
    
    // Transform sample to world space
    return glm::normalize(sample_dir.x * right + sample_dir.y * forward + sample_dir.z * direction);
}

// ============================================================================
// Video Recording Utilities
// ============================================================================

class VideoRecorder {
public:
    static inline const std::string Y4M_FILENAME = "recording.y4m";
    static inline const std::string MP4_FILENAME = "recording.mp4";
    
    VideoRecorder(const std::vector<uint32_t>& pixel_buffer, size_t width, size_t height)
        : pixel_buffer(pixel_buffer), recording(false), frame_count(0), width(width), height(height) {
    }
    
    ~VideoRecorder() = default;
    
    // Start/stop recording
    void startRecording() {
        if (recording) {
            std::cout << "Already recording!" << std::endl;
            return;
        }
        
        file_stream.open(Y4M_FILENAME, std::ios::binary | std::ios::out);
        if (!file_stream.is_open()) {
            std::cerr << "Failed to open file for recording: " << Y4M_FILENAME << std::endl;
            return;
        }
        
        recording = true;
        frame_count = 0;
        writeHeader();
        std::cout << "Started recording to " << Y4M_FILENAME << std::endl;
    }
    
    void stopRecording() {
        if (!recording) {
            return;
        }
        
        recording = false;
        file_stream.close();
        
        std::cout << "Stopped recording. Captured " << frame_count << " frames." << std::endl;
        std::cout << "Converting to MP4..." << std::endl;
        
        // Start conversion in jthread - it will automatically join in destructor
        conversion_thread = std::jthread([this]() {
            convertToMP4();
        });
    }
    
    void toggleRecording() {
        if (recording) {
            stopRecording();
        } else {
            startRecording();
        }
    }
    
    // Capture current frame
    void capture_frame() {
        if (!recording) {
            return;
        }
        
        writeFrame();
        frame_count++;
    }
    
    // Check if currently recording
    bool is_recording() const noexcept { return recording; }
    
private:
    const std::vector<uint32_t>& pixel_buffer;
    std::ofstream file_stream;
    std::jthread conversion_thread;
    bool recording;
    int frame_count;
    size_t width;
    size_t height;
    
    // Write Y4M header
    void writeHeader() {
        // Y4M header format: YUV4MPEG2 W<width> H<height> F<fps_num>:<fps_den> Ip A<aspect> C420jpeg
        // Using 30 fps as default
        file_stream << "YUV4MPEG2 W" << width << " H" << height << " F30:1 Ip A1:1 C420jpeg\n";
    }
    
    // Convert RGB to YUV and write frame
    void writeFrame() {
        // Frame header
        file_stream << "FRAME\n";
        
        // Convert ARGB to YUV420
        std::vector<uint8_t> y_plane(width * height);
        std::vector<uint8_t> u_plane((width / 2) * (height / 2));
        std::vector<uint8_t> v_plane((width / 2) * (height / 2));
        
        // Convert RGB to YUV for each pixel
        for (size_t i = 0; i < height; i++) {
            for (size_t j = 0; j < width; j++) {
                uint32_t pixel = pixel_buffer[i * width + j];
                
                // Extract RGB components
                uint8_t r = (pixel >> 16) & 0xFF;
                uint8_t g = (pixel >> 8) & 0xFF;
                uint8_t b = (pixel >> 0) & 0xFF;
                
                // Convert to YUV using ITU-R BT.601 standard
                int y = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16;
                int u = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128;
                int v = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128;
                
                // Clamp values
                y = std::max(0, std::min(255, y));
                u = std::max(0, std::min(255, u));
                v = std::max(0, std::min(255, v));
                
                y_plane[i * width + j] = static_cast<uint8_t>(y);
                
                // Subsample U and V (4:2:0)
                if (i % 2 == 0 && j % 2 == 0) {
                    size_t uv_index = (i / 2) * (width / 2) + (j / 2);
                    u_plane[uv_index] = static_cast<uint8_t>(u);
                    v_plane[uv_index] = static_cast<uint8_t>(v);
                }
            }
        }
        
        // Write Y plane
        file_stream.write(reinterpret_cast<const char*>(y_plane.data()), y_plane.size());
        
        // Write U plane
        file_stream.write(reinterpret_cast<const char*>(u_plane.data()), u_plane.size());
        
        // Write V plane
        file_stream.write(reinterpret_cast<const char*>(v_plane.data()), v_plane.size());
    }
    
    // Convert to MP4 using ffmpeg and delete Y4M
    void convertToMP4() {
        // Redirect output to /dev/null on Linux/Mac, nul on Windows
        #ifdef _WIN32
            std::string null_device = "nul";
        #else
            std::string null_device = "/dev/null";
        #endif
        
        std::string command = std::string("ffmpeg -y -i ") + Y4M_FILENAME + " " + MP4_FILENAME + 
                            " > " + null_device + " 2>&1";
        
        int result = std::system(command.c_str());
        
        if (result == 0) {
            // Delete Y4M file after successful conversion
            std::ignore = std::system((std::string("rm ") + Y4M_FILENAME).c_str());
            std::cout << "Video saved as " << MP4_FILENAME << std::endl;
        } else {
            std::cerr << "Failed to convert to MP4. Make sure ffmpeg is installed." << std::endl;
        }
    }
};

