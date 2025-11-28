#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <fstream>
#include <thread>

/**
 * @brief Records frames to Y4M and converts to MP4 using ffmpeg.
 */
class VideoRecorder {
public:
    static inline const std::string Y4MFilename = "recording.y4m";
    static inline const std::string MP4Filename = "recording.mp4";
    /**
     * @brief Binds a pixel buffer for capture.
     * @param pixel_buffer ARGB backbuffer.
     * @param width Frame width.
     * @param height Frame height.
     */
    explicit VideoRecorder(const std::vector<uint32_t>& pixel_buffer, size_t width, size_t height);
    /** @brief Destructor; joins conversion thread when active. */
    ~VideoRecorder();
    /** @brief Begins Y4M recording to disk. */
    void start_recording();
    /** @brief Ends recording and launches MP4 conversion. */
    void stop_recording();
    /** @brief Toggles recording state. */
    void toggle_recording();
    /** @brief Appends the current frame to the Y4M stream. */
    void capture_frame();
    /** @brief Returns true when recording is active. */
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
