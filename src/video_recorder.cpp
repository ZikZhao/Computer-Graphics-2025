#include "video_recorder.hpp"
#include <iostream>
#include <cstdlib>

VideoRecorder::VideoRecorder(const std::vector<uint32_t>& pixel_buffer, size_t width, size_t height)
    : pixel_buffer(pixel_buffer), recording(false), frame_count(0), width(width), height(height) {}

VideoRecorder::~VideoRecorder() = default;

void VideoRecorder::startRecording() {
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

void VideoRecorder::stopRecording() {
    if (!recording) return;
    recording = false;
    file_stream.close();
    std::cout << "Stopped recording. Captured " << frame_count << " frames." << std::endl;
    std::cout << "Converting to MP4..." << std::endl;
    conversion_thread = std::jthread([this]() { convertToMP4(); });
}

void VideoRecorder::toggleRecording() {
    if (recording) stopRecording(); else startRecording();
}

void VideoRecorder::capture_frame() {
    if (!recording) return;
    writeFrame();
    frame_count++;
}

bool VideoRecorder::is_recording() const noexcept { return recording; }

void VideoRecorder::writeHeader() {
    file_stream << "YUV4MPEG2 W" << width << " H" << height << " F30:1 Ip A1:1 C420jpeg\n";
}

void VideoRecorder::writeFrame() {
    file_stream << "FRAME\n";
    std::vector<uint8_t> y_plane(width * height);
    std::vector<uint8_t> u_plane((width / 2) * (height / 2));
    std::vector<uint8_t> v_plane((width / 2) * (height / 2));
    for (size_t i = 0; i < height; i++) {
        for (size_t j = 0; j < width; j++) {
            uint32_t pixel = pixel_buffer[i * width + j];
            uint8_t r = (pixel >> 16) & 0xFF;
            uint8_t g = (pixel >> 8) & 0xFF;
            uint8_t b = (pixel >> 0) & 0xFF;
            int y = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16;
            int u = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128;
            int v = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128;
            y = std::max(0, std::min(255, y));
            u = std::max(0, std::min(255, u));
            v = std::max(0, std::min(255, v));
            y_plane[i * width + j] = static_cast<uint8_t>(y);
            if (i % 2 == 0 && j % 2 == 0) {
                size_t uv_index = (i / 2) * (width / 2) + (j / 2);
                u_plane[uv_index] = static_cast<uint8_t>(u);
                v_plane[uv_index] = static_cast<uint8_t>(v);
            }
        }
    }
    file_stream.write(reinterpret_cast<const char*>(y_plane.data()), y_plane.size());
    file_stream.write(reinterpret_cast<const char*>(u_plane.data()), u_plane.size());
    file_stream.write(reinterpret_cast<const char*>(v_plane.data()), v_plane.size());
}

void VideoRecorder::convertToMP4() {
    #ifdef _WIN32
        std::string null_device = "nul";
    #else
        std::string null_device = "/dev/null";
    #endif
    std::string command = std::string("ffmpeg -y -i ") + Y4M_FILENAME + " " + MP4_FILENAME + 
                         " > " + null_device + " 2>&1";
    int result = std::system(command.c_str());
    if (result == 0) {
        std::ignore = std::system((std::string("rm ") + Y4M_FILENAME).c_str());
        std::cout << "Video saved as " << MP4_FILENAME << std::endl;
    } else {
        std::cerr << "Failed to convert to MP4. Make sure ffmpeg is installed." << std::endl;
    }
}

