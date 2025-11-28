#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <fstream>
#include <thread>

class VideoRecorder {
public:
    static inline const std::string Y4M_FILENAME = "recording.y4m";
    static inline const std::string MP4_FILENAME = "recording.mp4";
    VideoRecorder(const std::vector<uint32_t>& pixel_buffer, size_t width, size_t height);
    ~VideoRecorder();
    void startRecording();
    void stopRecording();
    void toggleRecording();
    void capture_frame();
    bool is_recording() const noexcept;
private:
    const std::vector<uint32_t>& pixel_buffer;
    std::ofstream file_stream;
    std::jthread conversion_thread;
    bool recording;
    int frame_count;
    size_t width;
    size_t height;
    void writeHeader();
    void writeFrame();
    void convertToMP4();
};

