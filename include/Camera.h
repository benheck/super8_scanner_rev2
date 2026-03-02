#pragma once

#include <libcamera/libcamera.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <functional>
#include <queue>
#include <mutex>
#include <condition_variable>

class PiCamera {
public:
    PiCamera();
    ~PiCamera();

    // Initialization
    bool initialize();
    void shutdown();

    // Video mode
    bool setVideoBitDepth(int bitDepth);
    bool startVideo();
    bool stopVideo();
    bool startPhoto();
    bool stopPhoto();
    bool getVideoFrame(cv::Mat& frame);

    // Photo mode
    bool setPhotoBitDepth(int bitDepth);
    bool capturePhoto(cv::Mat& image);

    // Configuration
    void setVideoResolution(int width, int height);
    void setPhotoResolution(int width, int height);
    void setFramerate(int fps);
    void setWhiteBalance(const std::string& mode); // "auto", "incandescent", "tungsten", "fluorescent", "daylight"
    void setExposure(float milliseconds);
    void setGain(float gain);
    void setDenoise(bool enable);

    // Status
    bool isReady() const { return initialized_; }
    bool isVideoRunning() const { return videoRunning_; }

private:
    void processRequest(libcamera::Request* request);
    void videoRequestComplete(libcamera::Request* request);
    void photoRequestComplete(libcamera::Request* request);
    
    std::unique_ptr<libcamera::CameraManager> cameraManager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> videoConfig_;
    std::unique_ptr<libcamera::CameraConfiguration> photoConfig_;
    std::unique_ptr<libcamera::FrameBufferAllocator> videoAllocator_;
    std::unique_ptr<libcamera::FrameBufferAllocator> photoAllocator_;
    
    std::map<libcamera::FrameBuffer*, std::vector<uint8_t>> mappedBuffers_;
    std::vector<std::unique_ptr<libcamera::Request>> videoRequests_;
    std::vector<std::unique_ptr<libcamera::Request>> photoRequests_;
    
    // Video settings
    int videoWidth_ = 1920;
    int videoHeight_ = 1080;
    int videoFps_ = 30;
    unsigned int videoStride_ = 0;
    
    std::string videoBitDepth = "RGB888";
    std::string photoBitDepth = "RGB888";

    // Photo settings
    int photoWidth_ = 4056;
    int photoHeight_ = 3040;
    
    // Control values
    std::string whiteBalanceMode_ = "incandescent";
    float exposureMs_ = 0.0f;  // 0 = auto
    float gain_ = 1.0f;
    bool denoiseEnabled_ = false;
    
    // State
    bool initialized_ = false;
    bool videoRunning_ = false;
    bool photoRunning_ = false;
    
    // Thread-safe frame queue
    std::queue<cv::Mat> frameQueue_;
    std::mutex queueMutex_;
    std::condition_variable queueCV_;
    const size_t MAX_QUEUE_SIZE = 3;
    
    // Photo capture
    std::mutex photoMutex_;
    std::condition_variable photoCV_;
    cv::Mat capturedPhoto_;
    bool photoReady_ = false;
    bool captureRequested_ = false;
    int photoFrameCount_ = 0;
    size_t currentPhotoRequest_ = 0;
};
