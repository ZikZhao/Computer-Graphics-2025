#include <iostream>
#include <cstdlib>
#include "video_recorder.hpp"

VideoRecorder::VideoRecorder(const std::vector<uint32_t>& pixel_buffer, size_t width, size_t height)
    : pixel_buffer_(pixel_buffer), recording_(false), frame_count_(0), width_(width), height_(height) {}

VideoRecorder::~VideoRecorder() = default;

void VideoRecorder::start_recording() {
    
    // Open Y4M stream and initialize recording state
    if (recording_) {
        std::cout << "Already recording!" << std::endl;
        return;
    }
    file_stream_.open(Y4MFilename, std::ios::binary | std::ios::out);
    if (!file_stream_.is_open()) {
        std::cerr << "Failed to open file for recording: " << Y4MFilename << std::endl;
        return;
    }
    recording_ = true;
    frame_count_ = 0;
    write_header();
    std::cout << "Started recording to " << Y4MFilename << std::endl;
}

void VideoRecorder::stop_recording() {
    
    // Close stream and spawn asynchronous MP4 conversion
    if (!recording_) return;
    recording_ = false;
    file_stream_.close();
    std::cout << "Stopped recording. Captured " << frame_count_ << " frames." << std::endl;
    std::cout << "Converting to MP4..." << std::endl;
    conversion_thread_ = std::jthread([this]() { convert_to_mp4(); });
}

void VideoRecorder::toggle_recording() {
    
    // Convenience toggle for UI bindings
    if (recording_) stop_recording(); else start_recording();
}

void VideoRecorder::capture_frame() {
    
    // Append current backbuffer as a frame to the Y4M file
    if (!recording_) return;
    write_frame();
    frame_count_++;
}

bool VideoRecorder::is_recording() const noexcept { return recording_; }

void VideoRecorder::write_header() {
    
    // Minimal Y4M header describing resolution and format
    file_stream_ << "YUV4MPEG2 W" << width_ << " H" << height_ << " F30:1 Ip A1:1 C420jpeg\n";
}

void VideoRecorder::write_frame() {
    
    // Convert ARGB backbuffer to 4:2:0 planar YUV and write a frame chunk
    file_stream_ << "FRAME\n";
    std::vector<uint8_t> y_plane(width_ * height_);
    std::vector<uint8_t> u_plane((width_ / 2) * (height_ / 2));
    std::vector<uint8_t> v_plane((width_ / 2) * (height_ / 2));
    for (size_t i = 0; i < height_; i++) {
        for (size_t j = 0; j < width_; j++) {
            uint32_t pixel = pixel_buffer_[i * width_ + j];
            uint8_t r = (pixel >> 16) & 0xFF;
            uint8_t g = (pixel >> 8) & 0xFF;
            uint8_t b = (pixel >> 0) & 0xFF;
            int y = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16;
            int u = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128;
            int v = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128;
            y = std::max(0, std::min(255, y));
            u = std::max(0, std::min(255, u));
            v = std::max(0, std::min(255, v));
            y_plane[i * width_ + j] = static_cast<uint8_t>(y);
            if (i % 2 == 0 && j % 2 == 0) {
                size_t uv_index = (i / 2) * (width_ / 2) + (j / 2);
                u_plane[uv_index] = static_cast<uint8_t>(u);
                v_plane[uv_index] = static_cast<uint8_t>(v);
            }
        }
    }
    file_stream_.write(reinterpret_cast<const char*>(y_plane.data()), y_plane.size());
    file_stream_.write(reinterpret_cast<const char*>(u_plane.data()), u_plane.size());
    file_stream_.write(reinterpret_cast<const char*>(v_plane.data()), v_plane.size());
}

void VideoRecorder::convert_to_mp4() {
    
    // Invoke ffmpeg to transcode Y4M to MP4; silence console output
    #ifdef _WIN32
        std::string null_device = "nul";
    #else
        std::string null_device = "/dev/null";
    #endif
    std::string command = std::string("ffmpeg -y -i ") + Y4MFilename + " " + MP4Filename + 
                         " > " + null_device + " 2>&1";
    int result = std::system(command.c_str());
    if (result == 0) {
        std::ignore = std::system((std::string("rm ") + Y4MFilename).c_str());
        std::cout << "Video saved as " << MP4Filename << std::endl;
    } else {
        std::cerr << "Failed to convert to MP4. Make sure ffmpeg is installed." << std::endl;
    }
}
