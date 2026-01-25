#!/bin/bash

# Build script for Super 8 Scanner

set -e

echo "========================================="
echo "Super 8 Scanner Build Script"
echo "========================================="

# Check if we're in the right directory
if [ ! -f "CMakeLists.txt" ]; then
    echo "Error: CMakeLists.txt not found!"
    echo "Please run this script from the project root directory."
    exit 1
fi

# Create build directory
echo "Creating build directory..."
mkdir -p build
cd build

# Run CMake
echo "Running CMake..."
cmake ..

# Build
echo "Building application..."
make -j$(nproc)

echo "========================================="
echo "Build complete!"
echo "Run with: LC_ALL=en_US.UTF-8 ./build/scanner"
echo "========================================="

# Run the scanner with proper UTF-8 locale
# echo ""
# echo "Starting scanner..."
# LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 ./scanner
