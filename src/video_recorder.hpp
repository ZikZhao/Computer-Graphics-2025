#pragma once
#include <cstdint>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

/**
 * @brief Records frames to Y4M and converts to MP4 using ffmpeg.
 */
class VideoRecorder {
public: // Static Methods & Constants
    static inline const std::string Y4MFilename = "recording.y4m";
    static inline const std::string MP4Filename = "recording.mp4";

private: // Data
    const std::vector<std::uint32_t>& pixel_buffer_;
    std::ofstream file_stream_;
    std::jthread conversion_thread_;
    bool recording_;
    int frame_count_;
    std::size_t width_;
    std::size_t height_;

public: // Lifecycle
    /**
     * @brief Binds a pixel buffer for capture.
     * @param pixel_buffer ARGB backbuffer.
     * @param width Frame width.
     * @param height Frame height.
     */
    explicit VideoRecorder(
        const std::vector<std::uint32_t>& pixel_buffer, std::size_t width, std::size_t height
    );
    ~VideoRecorder();

public: // Accessors & Data Binding
    /** @brief Returns true when recording is active. */
    [[nodiscard]] bool is_recording() const noexcept { return recording_; }

public: // Core Operations
    /** @brief Begins Y4M recording to disk. */
    void start_recording();
    /** @brief Ends recording and launches MP4 conversion. */
    void stop_recording();
    /** @brief Toggles recording state. */
    void toggle_recording();
    /** @brief Appends the current frame to the Y4M stream. */
    void capture_frame();

private: // Core Operations (Internal)
    void write_header();
    void write_frame();
    void convert_to_mp4();
};
