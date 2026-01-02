#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo "========================================"
echo "Batch Point Cloud Generation"
echo "========================================"
echo ""

# Configuration
CALIB_FILE="femto_bolt_CL8855300FR_calibration.bin"
RGB_DIR="/home/sungwoo/yoloclaude/ysfm_new/data/rgb"
ALIGNED_DEPTH_DIR="/home/sungwoo/yoloclaude/ysfm_new/data/aligned_depth"
OUTPUT_DIR="/home/sungwoo/yoloclaude/ysfm_new/data/pointclouds"

# Verify inputs
if [ ! -f "$CALIB_FILE" ]; then
    echo "Error: Calibration file not found: $CALIB_FILE"
    exit 1
fi

if [ ! -d "$RGB_DIR" ]; then
    echo "Error: RGB directory not found: $RGB_DIR"
    exit 1
fi

if [ ! -d "$ALIGNED_DEPTH_DIR" ]; then
    echo "Error: Aligned depth directory not found: $ALIGNED_DEPTH_DIR"
    exit 1
fi

echo "Configuration:"
echo "  Calibration: $CALIB_FILE"
echo "  RGB directory: $RGB_DIR"
echo "  Aligned depth directory: $ALIGNED_DEPTH_DIR"
echo "  Output directory: $OUTPUT_DIR"
echo ""

read -p "Proceed with batch processing? (y/n) " -n 1 -r
echo ""

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Cancelled."
    exit 0
fi

echo ""
echo "Starting batch processing..."
echo ""

./create_pointcloud_batch "$CALIB_FILE" "$RGB_DIR" "$ALIGNED_DEPTH_DIR" "$OUTPUT_DIR"

echo ""
echo "========================================"
echo "Batch processing complete!"
echo "========================================"
