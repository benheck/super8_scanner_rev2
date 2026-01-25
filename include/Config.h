#pragma once

#include <string>

namespace Config {
    // Camera settings
    constexpr int VIDEO_WIDTH = 1920;
    constexpr int VIDEO_HEIGHT = 1080;
    constexpr int VIDEO_FPS = 30;
    
    constexpr int PHOTO_WIDTH = 4056;
    constexpr int PHOTO_HEIGHT = 3040;
    
    // Film parameters
    constexpr float FILM_THICKNESS_MM = 0.15f;
    constexpr float SUPER8_FRAME_HEIGHT = 4.234f;
    constexpr float BASE_SPOOL_DIAMETER = 31.3f;
    
    // Sprocket detection
    constexpr int SPROCKET_THRESHOLD_DEFAULT = 25;
    constexpr int SAMPLE_Y_OFFSET = 180;
    
    // Export settings
    const std::string DEFAULT_EXPORT_FOLDER = "export";
    
    // Serial settings
    const std::string MARLIN_PORT = "/dev/ttyACM0";
    constexpr int MARLIN_BAUD = 115200;
}
