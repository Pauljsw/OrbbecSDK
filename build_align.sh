#!/bin/bash

echo "========================================="
echo "Building Post-Alignment Tool"
echo "========================================="

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Check if OpenCV is installed
if ! pkg-config --exists opencv4; then
    echo "⚠️  Warning: OpenCV 4 not found, trying opencv..."
    if ! pkg-config --exists opencv; then
        echo "❌ ERROR: OpenCV not found!"
        echo ""
        echo "Please install OpenCV:"
        echo "  sudo apt-get update"
        echo "  sudo apt-get install libopencv-dev"
        exit 1
    fi
    OPENCV_PKG="opencv"
else
    OPENCV_PKG="opencv4"
fi

echo "✓ Found OpenCV: $OPENCV_PKG"

# Compile single image alignment tool
echo ""
echo "Compiling post_align_single..."
g++ post_align_single.cpp -o post_align_single \
    -I./include \
    -L./lib/linux_x64 \
    $(pkg-config --cflags --libs $OPENCV_PKG) \
    -lOrbbecSDK \
    -std=c++11 \
    -Wno-deprecated

if [ $? -eq 0 ]; then
    echo "✅ post_align_single compiled successfully!"
else
    echo "❌ Compilation failed!"
    exit 1
fi

# Compile batch alignment tool
echo ""
echo "Compiling post_align_batch..."
g++ post_align_batch.cpp -o post_align_batch \
    -I./include \
    -L./lib/linux_x64 \
    $(pkg-config --cflags --libs $OPENCV_PKG) \
    -lOrbbecSDK \
    -std=c++11 \
    -Wno-deprecated

if [ $? -eq 0 ]; then
    echo "✅ post_align_batch compiled successfully!"
else
    echo "❌ Compilation failed!"
    exit 1
fi

# Compile single point cloud generation tool
echo ""
echo "Compiling create_pointcloud_single..."
g++ create_pointcloud_single.cpp -o create_pointcloud_single \
    -I./include \
    -L./lib/linux_x64 \
    $(pkg-config --cflags --libs $OPENCV_PKG) \
    -lOrbbecSDK \
    -std=c++11 \
    -Wno-deprecated

if [ $? -eq 0 ]; then
    echo "✅ create_pointcloud_single compiled successfully!"
else
    echo "❌ Compilation failed!"
    exit 1
fi

# Compile batch point cloud generation tool
echo ""
echo "Compiling create_pointcloud_batch..."
g++ create_pointcloud_batch.cpp -o create_pointcloud_batch \
    -I./include \
    -L./lib/linux_x64 \
    $(pkg-config --cflags --libs $OPENCV_PKG) \
    -lOrbbecSDK \
    -std=c++17 \
    -Wno-deprecated

if [ $? -eq 0 ]; then
    echo "✅ create_pointcloud_batch compiled successfully!"
else
    echo "❌ Compilation failed!"
    exit 1
fi

echo ""
echo "========================================="
echo "✅ Build Complete!"
echo "========================================="
echo ""
echo "Available tools:"
echo "  1. post_align_single     - Align single depth to RGB"
echo "  2. post_align_batch      - Batch align depth to RGB"
echo "  3. create_pointcloud_single  - Generate single point cloud"
echo "  4. create_pointcloud_batch   - Batch generate point clouds"
echo ""
