# Super 8 Film Scanner - Raspberry Pi 5 (rpicam/libcamera)

A modern C++ application for scanning Super 8mm film using Raspberry Pi 5 with the newer rpicam camera stack (libcamera).

## Hardware Requirements

- Raspberry Pi 5
- Raspberry Pi Camera Module (HQ or Camera Module 3 recommended)
- Marlin-based film transport system
- LED backlight
- Cooling fan (optional)

## Software Requirements

- Raspberry Pi OS (Bookworm or newer - 2024+)
- libcamera development files
- OpenCV 4.x
- Qt5 or Qt6
- Boost (system, thread)
- CMake 3.16+

## Installation

### 1. Install System Dependencies

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    libcamera-dev \
    libopencv-dev \
    qtbase5-dev \
    libboost-system-dev \
    libboost-thread-dev \
    pkg-config
```

### 2. Build the Application

```bash
cd /home/ben/scanner
mkdir build
cd build
cmake ..
make -j4
```

### 3. Run the Scanner

```bash
./scanner
```

## Configuration

### Camera Settings

Edit `include/Config.h` to adjust camera parameters:

- **VIDEO_WIDTH/HEIGHT**: Preview resolution (default: 1920x1080)
- **PHOTO_WIDTH/HEIGHT**: Capture resolution (default: 4056x3040)
- **VIDEO_FPS**: Preview frame rate (default: 30)

### Film Parameters

- **FILM_THICKNESS_MM**: Super 8 film thickness (default: 0.15mm)
- **SUPER8_FRAME_HEIGHT**: Frame height in mm (default: 4.234mm)
- **BASE_SPOOL_DIAMETER**: Starting spool diameter (default: 31.3mm)

### Serial Connection

- **MARLIN_PORT**: Serial port for Marlin controller (default: `/dev/ttyACM0`)
- **MARLIN_BAUD**: Baud rate (default: 115200)

To find your serial port:
```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

## Usage

### Initial Setup

1. **Connect Hardware**: Ensure camera, Marlin controller, and power are connected
2. **Launch Application**: Run `./scanner`
3. **Enable Motors**: Click "Motors: OFF" to enable
4. **Turn on Light**: Click "Light: OFF" to illuminate film
5. **Load Film**: Manually load film into the transport

### Calibration

1. **Switch to Setup Mode**: Click "Setup Mode" button
2. **Adjust Sprocket Threshold**: Use slider until sprocket hole is reliably detected (red circle)
3. **Frame Alignment**: Use advance/rewind buttons to position first frame
4. **Set Spool Diameter**: Enter current takeup spool diameter (measure with calipers)

### Scanning

1. **Select Export Folder**: Click "Select Export Folder" and choose destination
2. **Switch to Preview Mode**: Click "Preview Mode" to see live alignment
3. **Start Scan**: Click "Start Scan" when ready
4. **Monitor Progress**: Frame counter shows captured frames
5. **Stop When Done**: Click "Stop Scan" or let it run to end of film

### Spool Diameter Tracking

As film winds onto the takeup spool, the diameter increases. You should:
- Measure the spool diameter periodically
- Update the value in the "Spool Diameter" field
- This ensures accurate frame spacing

Formula: New diameter = base + (2 × film thickness × frames wound)

## Architecture

### Camera Class (`Camera.h/cpp`)

Clean wrapper around libcamera providing:
- **Video Streaming**: Continuous YUV420 → BGR conversion at 30fps
- **Still Capture**: High-resolution photo capture (4K)
- **Controls**: White balance, exposure, gain, denoise
- **Thread Safety**: Frame queue with mutex protection

### MarlinController Class (`MarlinController.h/cpp`)

Serial communication with Marlin firmware:
- **G-code Commands**: Send movement, light, fan commands
- **Film Advance**: Precise movement based on spool diameter
- **Response Handling**: Wait for "ok" confirmations

### Main Application (`main.cpp`)

Qt-based GUI with three states:
- **PREVIEW**: Live video with alignment guides
- **SETUP**: Still image for sprocket calibration
- **SCANNING**: Automated capture loop

Features:
- Real-time sprocket hole detection using OpenCV contour analysis
- Overlay guides for frame alignment
- Automatic spool diameter compensation
- Sequential PNG export

## Key Differences from Old LCCV Version

### What Changed

1. **No LCCV Dependency**: Direct libcamera C++ API usage
2. **Cleaner Architecture**: Separated Camera class with clear interfaces
3. **Modern CMake**: Auto-detects Qt5/Qt6
4. **Thread-Safe Frame Queue**: Proper mutex protection for video frames
5. **Simplified Buffer Handling**: Automatic YUV→BGR conversion

### Migration from libcamera-* to rpicam-*

The command-line tools have been renamed:
- `libcamera-vid` → `rpicam-vid`
- `libcamera-still` → `rpicam-still`
- `libcamera-hello` → `rpicam-hello`

However, **the underlying C++ library is still `libcamera`**. This application uses the libcamera C++ API directly, so it works with both old and new Raspberry Pi OS versions.

## Troubleshooting

### Camera Not Found

```bash
# Check if camera is detected
rpicam-hello --list-cameras

# Should show camera details
```

### Permission Denied on Serial Port

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and back in for changes to take effect
```

### Build Errors

If Qt6 is not found:
```bash
# Install Qt5 instead
sudo apt install qtbase5-dev
```

If libcamera headers not found:
```bash
# Install libcamera development files
sudo apt install libcamera-dev
```

### Poor Frame Detection

- Increase lighting intensity
- Adjust sprocket threshold slider
- Check that film is properly tensioned
- Verify camera focus is sharp

## Performance Notes

- Video preview runs at ~30 FPS (1920x1080)
- Still capture takes ~200-500ms per frame (4056x3040)
- Total scan time: ~10-15 seconds per frame including movement
- A 50ft Super 8 reel (~3600 frames) takes approximately 10-15 hours

## Advanced Configuration

### Camera Controls

You can modify `Camera.cpp` to expose additional libcamera controls:
- Sharpness
- Contrast  
- Saturation
- Brightness
- Noise reduction modes

### Custom White Balance

To set custom color gains:
```cpp
// In Camera.cpp, videoRequestComplete():
controls.set(controls::ColourGains, 
    libcamera::Span<const float, 2>({redGain, blueGain}));
```

### GPIO Control

To control light/fan via GPIO instead of Marlin:
```bash
# Install GPIO library
sudo apt install libgpiod-dev
```

Then modify `MarlinController` to use GPIO pins.

## License

This is a custom application for film scanning. Modify as needed for your hardware setup.

## Credits

Built for Raspberry Pi 5 with modern rpicam/libcamera stack.
Original LCCV-based version: github.com/benheck/Super8_Super_Scanner
