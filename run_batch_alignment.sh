#!/bin/bash

echo "========================================="
echo "Batch RGB-Depth Alignment Processing"
echo "========================================="

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Set library path
export LD_LIBRARY_PATH="$SCRIPT_DIR/lib/linux_x64:$LD_LIBRARY_PATH"

# Configuration
CALIB_FILE="femto_bolt_CL8855300FR_calibration.bin"
RGB_DIR="/home/sungwoo/yoloclaude/ysfm_new/data/rgb"
DEPTH_DIR="/home/sungwoo/yoloclaude/ysfm_new/data/depth"
OUTPUT_DIR="/home/sungwoo/yoloclaude/ysfm_new/data/aligned_depth"

echo ""
echo "Configuration:"
echo "  Calibration: $CALIB_FILE"
echo "  RGB directory: $RGB_DIR"
echo "  Depth directory: $DEPTH_DIR"
echo "  Output directory: $OUTPUT_DIR"
echo ""

# Check if program exists
if [ ! -f "./post_align_batch" ]; then
    echo "❌ ERROR: post_align_batch not found. Please run ./build_align.sh first"
    exit 1
fi

# Check if calibration file exists
if [ ! -f "$CALIB_FILE" ]; then
    echo "❌ ERROR: Calibration file not found: $CALIB_FILE"
    exit 1
fi

# Check if directories exist
if [ ! -d "$RGB_DIR" ]; then
    echo "❌ ERROR: RGB directory not found: $RGB_DIR"
    exit 1
fi

if [ ! -d "$DEPTH_DIR" ]; then
    echo "❌ ERROR: Depth directory not found: $DEPTH_DIR"
    exit 1
fi

# Run batch alignment
echo "Starting batch processing..."
echo ""

./post_align_batch \
    --calib "$CALIB_FILE" \
    --rgb-dir "$RGB_DIR" \
    --depth-dir "$DEPTH_DIR" \
    --output-dir "$OUTPUT_DIR"

if [ $? -eq 0 ]; then
    echo ""
    echo "========================================="
    echo "✅ Batch processing completed!"
    echo "========================================="
    echo ""
    echo "Check output:"
    echo "  ls -lh $OUTPUT_DIR | head -20"
    echo ""
    echo "Verify results:"
    echo "  ls $OUTPUT_DIR/*.png | wc -l"
else
    echo ""
    echo "❌ Batch processing failed!"
    exit 1
fi
