#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo "========================================"
echo "Testing Single Point Cloud Generation"
echo "========================================"
echo ""

# Test with sample images
CALIB_FILE="femto_bolt_CL8855300FR_calibration.bin"
RGB_FILE="/home/sungwoo/yoloclaude/ysfm_new/data/rgb/camera_RGB_1737530027_1737530028.png"
ALIGNED_DEPTH_FILE="/home/sungwoo/yoloclaude/ysfm_new/data/aligned_depth/aligned_depth_1737530027.png"

if [ ! -f "$CALIB_FILE" ]; then
    echo "Error: Calibration file not found: $CALIB_FILE"
    exit 1
fi

if [ ! -f "$RGB_FILE" ]; then
    echo "Error: RGB file not found: $RGB_FILE"
    exit 1
fi

if [ ! -f "$ALIGNED_DEPTH_FILE" ]; then
    echo "Error: Aligned depth file not found: $ALIGNED_DEPTH_FILE"
    exit 1
fi

echo "Running point cloud generation..."
echo ""

./create_pointcloud_single "$CALIB_FILE" "$RGB_FILE" "$ALIGNED_DEPTH_FILE"

echo ""
echo "========================================"
echo "Test complete!"
echo "========================================"
