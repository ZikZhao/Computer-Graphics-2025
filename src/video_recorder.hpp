#pragma once
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

/// Records frames to Y4M and converts to MP4 using ffmpeg.
class VideoRecorder {
public:
    static inline const std::string Y4MFilename = "recording.y4m";
    static inline const std::string MP4Filename = "recording.mp4";

private:
    const std::vector<std::uint32_t>& pixel_buffer_;
    std::ofstream file_stream_;
    std::jthread conversion_thread_;
    bool recording_;
    int frame_count_;
    std::size_t width_;
    std::size_t height_;

public:
    /// Binds a pixel buffer for capture.
    /// @param pixel_buffer ARGB backbuffer.
    /// @param width Frame width.
    /// @param height Frame height.
    VideoRecorder(
        const std::vector<std::uint32_t>& pixel_buffer, std::size_t width, std::size_t height
    );

public:
    [[nodiscard]] bool is_recording() const noexcept { return recording_; }

public:
    void start_recording();
    void stop_recording();
    void toggle_recording();
    void capture_frame();

private:
    void write_header();
    void append_frame();
    void convert_to_mp4();
};
