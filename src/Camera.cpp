#include "Camera.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <sys/mman.h>
#include <unistd.h>

using namespace libcamera;

PiCamera::PiCamera() {
    cameraManager_ = std::make_unique<CameraManager>();
}

PiCamera::~PiCamera() {
    shutdown();
}

bool PiCamera::initialize() {
    if (initialized_) return true;
    
    // Start camera manager
    int ret = cameraManager_->start();
    if (ret) {
        std::cerr << "Failed to start camera manager: " << ret << std::endl;
        return false;
    }
    
    // Get cameras
    auto cameras = cameraManager_->cameras();
    if (cameras.empty()) {
        std::cerr << "No cameras found" << std::endl;
        return false;
    }
    
    std::cout << "Found " << cameras.size() << " camera(s)" << std::endl;
    
    // Use first camera
    camera_ = cameras[0];
    
    std::cout << "Acquiring camera: " << camera_->id() << std::endl;
    
    // Acquire camera
    if (camera_->acquire()) {
        std::cerr << "Failed to acquire camera (may be in use by another process)" << std::endl;
        std::cerr << "Try: pkill -9 rpicam; pkill -9 scanner" << std::endl;
        return false;
    }
    
    std::cout << "Camera acquired successfully" << std::endl;
    
    // Small delay to let camera settle
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    initialized_ = true;
    return true;
}

void PiCamera::shutdown() {
    std::cout << "Shutting down camera..." << std::endl;
    
    stopVideo();
    
    if (camera_) {
        std::cout << "Releasing camera..." << std::endl;
        camera_->release();
        camera_.reset();
    }
    
    if (cameraManager_) {
        cameraManager_->stop();
    }
    
    initialized_ = false;
    std::cout << "Camera shutdown complete" << std::endl;
}

bool PiCamera::startVideo() {

    if (!initialized_) {
        std::cerr << "Camera not initialized" << std::endl;
        return false;
    }
    if (videoRunning_) {
        std::cout << "Video already running" << std::endl;
        return true;
    }
    
    std::cout << "Generating video configuration..." << std::endl;
    
    // Generate video configuration
    videoConfig_ = camera_->generateConfiguration({StreamRole::VideoRecording});
    if (!videoConfig_) {
        std::cerr << "Failed to generate video configuration" << std::endl;
        return false;
    }
    
    std::cout << "Video configuration generated with " << videoConfig_->size() << " stream(s)" << std::endl;
    
    // Configure video stream
    StreamConfiguration& streamConfig = videoConfig_->at(0);
    streamConfig.size.width = videoWidth_;
    streamConfig.size.height = videoHeight_;
    // RGB888 - 8-bit per channel RGB for better performance
    streamConfig.pixelFormat = PixelFormat::fromString(videoBitDepth);

    std::cout << "Requested format: " << videoBitDepth << std::endl;
    std::cout << "Requested size: " << videoWidth_ << "x" << videoHeight_ << std::endl;
    
    // Validate configuration
    std::cout << "Validating configuration..." << std::endl;
    CameraConfiguration::Status validation = videoConfig_->validate();

    if (validation == CameraConfiguration::Invalid) {
        std::cerr << "Video configuration invalid" << std::endl;
        return false;
    }

    if (validation == CameraConfiguration::Adjusted) {
        std::cout << "Video configuration adjusted" << std::endl;
    }
    
    // Apply configuration
    std::cout << "Applying configuration..." << std::endl;

    if (camera_->configure(videoConfig_.get())) {
        std::cerr << "Failed to configure camera for video" << std::endl;
        return false;
    }
    
    // Store stride for later use
    videoStride_ = streamConfig.stride;
    std::cout << "Video stream configured: " << videoWidth_ << "x" << videoHeight_ 
              << " format=" << streamConfig.pixelFormat.toString() << std::endl;
    
    // Allocate buffers
    Stream* stream = streamConfig.stream();
    allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
    
    if (allocator_->allocate(stream) < 0) {
        std::cerr << "Failed to allocate buffers" << std::endl;
        return false;
    }
    
    // Create requests
    const std::vector<std::unique_ptr<FrameBuffer>>& buffers = allocator_->buffers(stream);
    for (unsigned int i = 0; i < buffers.size(); ++i) {
        std::unique_ptr<Request> request = camera_->createRequest();
        if (!request) {
            std::cerr << "Failed to create request" << std::endl;
            return false;
        }
        
        const std::unique_ptr<FrameBuffer>& buffer = buffers[i];
        if (request->addBuffer(stream, buffer.get())) {
            std::cerr << "Failed to add buffer to request" << std::endl;
            return false;
        }
        
        // Set controls
        ControlList& controls = request->controls();
        
        // Set white balance
        if (whiteBalanceMode_ == "auto") {
            controls.set(controls::AwbEnable, true);
        } else {
            controls.set(controls::AwbEnable, false);
            if (whiteBalanceMode_ == "incandescent") {
                controls.set(controls::ColourGains, libcamera::Span<const float, 2>({1.5f, 1.0f}));
            } else if (whiteBalanceMode_ == "daylight") {
                controls.set(controls::ColourGains, libcamera::Span<const float, 2>({1.0f, 1.0f}));
            }
        }
        
        // Set exposure
        if (exposureMs_ > 0.0f) {
            controls.set(controls::AeEnable, false);
            controls.set(controls::ExposureTime, static_cast<int32_t>(exposureMs_ * 1000));
            controls.set(controls::AnalogueGain, gain_);
        } else {
            controls.set(controls::AeEnable, true);
        }
        
        videoRequests_.push_back(std::move(request));
    }
    
    // Connect request completed signal
    camera_->requestCompleted.connect(this, &PiCamera::videoRequestComplete);
    
    // Start camera
    if (camera_->start()) {
        std::cerr << "Failed to start camera" << std::endl;
        return false;
    }
    
    // Queue all requests
    for (std::unique_ptr<Request>& request : videoRequests_) {
        camera_->queueRequest(request.get());
    }
    
    videoRunning_ = true;
    std::cout << "Video stream started: " << videoWidth_ << "x" << videoHeight_ << std::endl;
    
    return true;
}

bool PiCamera::setVideoBitDepth(int bitDepth) {

    if (bitDepth == 8) {
        videoBitDepth = "RGB888";
    } else if (bitDepth == 16) {
        videoBitDepth = "RGB161616";
    } else {
        std::cerr << "Unsupported video bit depth: " << bitDepth << std::endl;
        return false;
    }

    return true;

}

bool PiCamera::stopVideo() {

    if (!videoRunning_) return true;
    
    camera_->stop();
    camera_->requestCompleted.disconnect(this, &PiCamera::videoRequestComplete);
    
    videoRequests_.clear();
    mappedBuffers_.clear();
    
    if (allocator_) {
        allocator_->free(videoConfig_->at(0).stream());
        allocator_.reset();
    }
    
    // Clear frame queue
    std::lock_guard<std::mutex> lock(queueMutex_);
    while (!frameQueue_.empty()) {
        frameQueue_.pop();
    }
    
    videoRunning_ = false;
    std::cout << "Video stream stopped" << std::endl;
    
    return true;
}

void PiCamera::videoRequestComplete(Request* request) {

    if (request->status() == Request::RequestCancelled) {
        return;
    }
    
    // Get the buffer
    const Request::BufferMap& buffers = request->buffers();
    for (auto bufferPair : buffers) {
        FrameBuffer* buffer = bufferPair.second;
        
        // Get planes
        const Span<const FrameBuffer::Plane> planes = buffer->planes();
        
        if (planes.empty()) {
            std::cerr << "No planes in buffer" << std::endl;
            continue;
        }
        
        // Map first plane (entire buffer for contiguous formats)
        const FrameBuffer::Plane& plane = planes[0];
        void* memory = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
        if (memory == MAP_FAILED) {
            std::cerr << "Failed to mmap buffer" << std::endl;
            continue;
        }
        
        cv::Mat bgrImage;
        
        try {
            if (videoBitDepth == "RGB161616") {
                // RGB161616 is 6 bytes per pixel (16-bit R, G, B)
                cv::Mat rgb16Image(videoHeight_, videoWidth_, CV_16UC3, memory);
                
                // Convert to 8-bit BGR for display
                rgb16Image.convertTo(bgrImage, CV_8UC3, 1.0/256.0);  // Scale down to 8-bit
                cv::cvtColor(bgrImage, bgrImage, cv::COLOR_RGB2BGR);
            } else {
                // RGB888 is 3 bytes per pixel (8-bit R, G, B)
                cv::Mat rgb8Image(videoHeight_, videoWidth_, CV_8UC3, memory);
                
                // Convert RGB to BGR for display (no bit depth conversion needed)
                cv::cvtColor(rgb8Image, bgrImage, cv::COLOR_RGB2BGR);
            }
        } catch (const cv::Exception& e) {
            std::cerr << "Video conversion error: " << e.what() << std::endl;
            munmap(memory, plane.length);
            continue;
        }
        
        // Add to queue (thread-safe)
        {
            std::lock_guard<std::mutex> lock(queueMutex_);
            if (frameQueue_.size() < MAX_QUEUE_SIZE) {
                frameQueue_.push(bgrImage.clone());
                queueCV_.notify_one();
            }
        }
        
        munmap(memory, plane.length);
    }
    
    // Requeue the request
    request->reuse(Request::ReuseBuffers);
    camera_->queueRequest(request);

}

bool PiCamera::getVideoFrame(cv::Mat& frame) {

    std::unique_lock<std::mutex> lock(queueMutex_);
    
    if (frameQueue_.empty()) {
        // Wait for a frame with timeout
        if (queueCV_.wait_for(lock, std::chrono::milliseconds(100)) == std::cv_status::timeout) {
            return false;
        }
    }
    
    if (frameQueue_.empty()) {
        return false;
    }
    
    frame = frameQueue_.front();
    frameQueue_.pop();
    
    return true;
}

bool PiCamera::setPhotoBitDepth(int bitDepth) {

    if (bitDepth == 8) {
        photoBitDepth = "RGB888";
    } else if (bitDepth == 16) {
        photoBitDepth = "SRGGB10"; //"RGB161616";
    } else {
        std::cerr << "Unsupported photo bit depth: " << bitDepth << std::endl;
        return false;
    }

    return true;

}

bool PiCamera::capturePhoto(cv::Mat& image) {

    if (!initialized_) return false;
    
    // Stop video if running
    bool wasRunning = videoRunning_;

    if (wasRunning) {
        stopVideo();
    }
    
    // Generate photo configuration
    auto photoConfig = camera_->generateConfiguration({StreamRole::StillCapture});

    if (!photoConfig) {
        std::cerr << "Failed to generate photo configuration" << std::endl;
        return false;
    }
    
    // Configure photo stream
    StreamConfiguration& streamConfig = photoConfig->at(0);
    streamConfig.size.width = photoWidth_;
    streamConfig.size.height = photoHeight_;
    streamConfig.pixelFormat = PixelFormat::fromString(photoBitDepth);
    
    // Validate and apply
    photoConfig->validate();

    if (camera_->configure(photoConfig.get())) {
        std::cerr << "Failed to configure camera for photo" << std::endl;
        return false;
    }
    
    // Allocate buffer
    Stream* stream = streamConfig.stream();
    auto allocator = std::make_unique<FrameBufferAllocator>(camera_);
    allocator->allocate(stream);
    
    const std::vector<std::unique_ptr<FrameBuffer>>& buffers = allocator->buffers(stream);
    
    // Create request
    std::unique_ptr<Request> request = camera_->createRequest();
    request->addBuffer(stream, buffers[0].get());
    
    // Set controls (same as video)
    ControlList& controls = request->controls();

    if (whiteBalanceMode_ == "auto") {
        controls.set(controls::AwbEnable, true);
    }
    else {
        controls.set(controls::AwbEnable, false);
        if (whiteBalanceMode_ == "incandescent") {
            controls.set(controls::ColourGains, libcamera::Span<const float, 2>({1.5f, 1.0f}));
        }
    }
    
    if (exposureMs_ > 0.0f) {
        controls.set(controls::AeEnable, false);
        controls.set(controls::ExposureTime, static_cast<int32_t>(exposureMs_ * 1000));
        controls.set(controls::AnalogueGain, gain_);
    }
    
    // Setup photo capture callback
    photoReady_ = false;
    camera_->requestCompleted.connect(this, &PiCamera::photoRequestComplete);
    
    // Start and queue request
    camera_->start();
    camera_->queueRequest(request.get());
    
    // Wait for photo
    {
        std::unique_lock<std::mutex> lock(photoMutex_);
        photoCV_.wait_for(lock, std::chrono::seconds(2), [this] { return photoReady_; });
    }
    
    camera_->stop();
    camera_->requestCompleted.disconnect(this, &PiCamera::photoRequestComplete);
    
    if (photoReady_) {
        image = capturedPhoto_.clone();
    }
    
    // Restart video if it was running
    if (wasRunning) {
        startVideo();
    }
    
    return photoReady_;

}

void PiCamera::photoRequestComplete(Request* request) {

    if (request->status() != Request::RequestComplete) {
        return;
    }
    
    const Request::BufferMap& buffers = request->buffers();

    for (auto bufferPair : buffers) {
        FrameBuffer* buffer = bufferPair.second;
        const FrameBuffer::Plane& plane = buffer->planes()[0];
        
        void* memory = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
        if (memory != MAP_FAILED) {
            // Get the actual stride from the stream configuration
            const StreamConfiguration& cfg = request->buffers().begin()->first->configuration();
            size_t stride = cfg.stride;
            
            // Process based on bit depth
            if (photoBitDepth == "SRGGB10") {
                // RAW 10-bit Bayer format - need to debayer
                // SRGGB10 is packed 10-bit, but we'll treat it as 16-bit for OpenCV
                cv::Mat rawImage(photoHeight_, photoWidth_, CV_16UC1, memory, stride);
                
                // Debayer using RG Bayer pattern to BGR
                cv::cvtColor(rawImage, capturedPhoto_, cv::COLOR_BayerRG2BGR);
            } else if (photoBitDepth == "RGB161616") {
                // 16-bit RGB processing
                cv::Mat rgb16Image(photoHeight_, photoWidth_, CV_16UC3, memory, stride);
                cv::cvtColor(rgb16Image, capturedPhoto_, cv::COLOR_RGB2BGR);
            } else {
                // 8-bit processing (RGB888)
                cv::Mat rgb8Image(photoHeight_, photoWidth_, CV_8UC3, memory, stride);
                cv::cvtColor(rgb8Image, capturedPhoto_, cv::COLOR_RGB2BGR);
            }
            
            munmap(memory, plane.length);
            
            std::lock_guard<std::mutex> lock(photoMutex_);
            photoReady_ = true;
            photoCV_.notify_one();
        }
    }

}

// Configuration methods
void PiCamera::setVideoResolution(int width, int height) {
    videoWidth_ = width;
    videoHeight_ = height;
}

void PiCamera::setPhotoResolution(int width, int height) {
    photoWidth_ = width;
    photoHeight_ = height;
}

void PiCamera::setFramerate(int fps) {
    videoFps_ = fps;
}

void PiCamera::setWhiteBalance(const std::string& mode) {
    whiteBalanceMode_ = mode;
}

void PiCamera::setExposure(float milliseconds) {
    exposureMs_ = milliseconds;
}

void PiCamera::setGain(float gain) {
    gain_ = gain;
}

void PiCamera::setDenoise(bool enable) {
    denoiseEnabled_ = enable;
}
