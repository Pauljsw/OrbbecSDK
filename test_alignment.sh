#!/bin/bash

echo "========================================="
echo "Test Post-Processing Alignment"
echo "========================================="

# Set library path
export LD_LIBRARY_PATH=/home/user/OrbbecSDK/lib/linux_x64:$LD_LIBRARY_PATH

# Test files (from user's data)
CALIB_FILE="femto_bolt_CL8855300FR_calibration.bin"
DEPTH_FILE="/home/sungwoo/yoloclaude/ysfm_new/data/depth/camera_DPT_1761702483_382657024.png"
RGB_FILE="/home/sungwoo/yoloclaude/ysfm_new/data/rgb/camera_RGB_1761702483_382657024.png"
OUTPUT_FILE="aligned_depth_test.png"

echo ""
echo "Test configuration:"
echo "  Calibration: $CALIB_FILE"
echo "  Depth: $DEPTH_FILE"
echo "  RGB: $RGB_FILE"
echo "  Output: $OUTPUT_FILE"
echo ""

# Check if files exist
if [ ! -f "$CALIB_FILE" ]; then
    echo "❌ ERROR: Calibration file not found: $CALIB_FILE"
    exit 1
fi

if [ ! -f "$DEPTH_FILE" ]; then
    echo "❌ ERROR: Depth image not found: $DEPTH_FILE"
    exit 1
fi

if [ ! -f "$RGB_FILE" ]; then
    echo "❌ ERROR: RGB image not found: $RGB_FILE"
    exit 1
fi

if [ ! -f "./post_align_single" ]; then
    echo "❌ ERROR: post_align_single not found. Please run ./build_align.sh first"
    exit 1
fi

echo "Running alignment..."
echo ""

./post_align_single \
    --calib "$CALIB_FILE" \
    --depth "$DEPTH_FILE" \
    --rgb "$RGB_FILE" \
    --output "$OUTPUT_FILE" \
    --hole-fill nearest

if [ $? -eq 0 ]; then
    echo ""
    echo "========================================="
    echo "✅ Test completed successfully!"
    echo "========================================="
    echo ""
    echo "Check output files:"
    echo "  ls -lh aligned_depth_test*"
    echo ""
    echo "View overlay:"
    echo "  eog aligned_depth_test_overlay.png"
    echo "  (or use any image viewer)"
else
    echo ""
    echo "❌ Test failed!"
    exit 1
fi
