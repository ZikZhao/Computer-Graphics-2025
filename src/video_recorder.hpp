#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <fstream>
#include <thread>

class VideoRecorder {
public:
    static inline const std::string Y4MFilename = "recording.y4m";
    static inline const std::string MP4Filename = "recording.mp4";
    VideoRecorder(const std::vector<uint32_t>& pixel_buffer, size_t width, size_t height);
    ~VideoRecorder();
    void start_recording();
    void stop_recording();
    void toggle_recording();
    void capture_frame();
    bool is_recording() const noexcept;
private:
    const std::vector<uint32_t>& pixel_buffer_;
    std::ofstream file_stream_;
    std::jthread conversion_thread_;
    bool recording_;
    int frame_count_;
    size_t width_;
    size_t height_;
    void write_header();
    void write_frame();
    void convert_to_mp4();
};
