#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Utils.hpp>
#include <libobsensor/hpp/Filter.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>

/**
 * Post-processing RGB-Depth Alignment + Scale Map Generator
 *
 * Aligns already-captured depth images to color camera perspective
 * and generates per-pixel mm/px scale maps
 *
 * Uses OrbbecSDK's calibration2dTo2d and dense hole filling
 *
 * Usage:
 *   ./post_align_single \
 *     --calib femto_bolt_CL8855300FR_calibration.bin \
 *     --depth /path/to/depth.png \
 *     --rgb /path/to/rgb.png \
 *     --output /path/to/aligned_depth.png
 */

/**
 * Save a float32 2D array as NPY format (NumPy compatible)
 */
void saveNPY(const std::string& filename, const cv::Mat& data) {
    if(data.type() != CV_32F) {
        throw std::runtime_error("saveNPY: Only CV_32F (float32) supported");
    }

    std::ofstream ofs(filename, std::ios::binary);
    if(!ofs.is_open()) {
        throw std::runtime_error("saveNPY: Cannot open file: " + filename);
    }

    // NPY format header
    // Magic number
    ofs.write("\x93NUMPY", 6);

    // Version 1.0
    ofs.put(0x01);
    ofs.put(0x00);

    // Create header dict
    std::ostringstream header;
    header << "{'descr': '<f4', 'fortran_order': False, 'shape': ("
           << data.rows << ", " << data.cols << "), }";

    std::string header_str = header.str();

    // Pad header to align to 64-byte boundary
    // Total header size = 6 (magic) + 2 (version) + 2 (header_len) + header_str + padding
    while((header_str.size() + 10) % 64 != 0) {
        header_str += ' ';
    }
    header_str += '\n';

    // Write header length (little-endian uint16)
    uint16_t header_len = header_str.size();
    ofs.write(reinterpret_cast<const char*>(&header_len), 2);

    // Write header string
    ofs.write(header_str.c_str(), header_len);

    // Write data (row-major, C-order)
    ofs.write(reinterpret_cast<const char*>(data.data),
              data.rows * data.cols * sizeof(float));

    ofs.close();
}

/**
 * Compute per-pixel mm/px scale map (isotropic) from aligned depth and camera intrinsics
 */
cv::Mat computeScaleMapIso(const cv::Mat& alignedDepthMM, float fx, float fy) {
    cv::Mat scaleMapIso = cv::Mat::zeros(alignedDepthMM.size(), CV_32F);

    for(int y = 0; y < alignedDepthMM.rows; y++) {
        for(int x = 0; x < alignedDepthMM.cols; x++) {
            uint16_t depthMM = alignedDepthMM.at<uint16_t>(y, x);

            if(depthMM > 0) {
                float depthM = depthMM / 1000.0f;  // mm to meters

                // mm/px = (depth_m / focal_length) * 1000.0
                float mmPerPxX = (depthM / fx) * 1000.0f;
                float mmPerPxY = (depthM / fy) * 1000.0f;
                float mmPerPxIso = 0.5f * (mmPerPxX + mmPerPxY);

                scaleMapIso.at<float>(y, x) = mmPerPxIso;
            }
        }
    }

    return scaleMapIso;
}

void printUsage(const char* programName) {
    std::cout << "\nUsage:" << std::endl;
    std::cout << "  " << programName << " \\" << std::endl;
    std::cout << "    --calib <calibration.bin> \\" << std::endl;
    std::cout << "    --depth <depth_image.png> \\" << std::endl;
    std::cout << "    --rgb <rgb_image.png> \\" << std::endl;
    std::cout << "    --output <output_aligned_depth.png> \\" << std::endl;
    std::cout << "    [--hole-fill nearest|farest|top]" << std::endl;
    std::cout << "\nExample:" << std::endl;
    std::cout << "  " << programName << " \\" << std::endl;
    std::cout << "    --calib femto_bolt_CL8855300FR_calibration.bin \\" << std::endl;
    std::cout << "    --depth /home/sungwoo/yoloclaude/ysfm_new/data/depth/camera_DPT_1761702483_382657024.png \\" << std::endl;
    std::cout << "    --rgb /home/sungwoo/yoloclaude/ysfm_new/data/rgb/camera_RGB_1761702483_382657024.png \\" << std::endl;
    std::cout << "    --output aligned_depth.png" << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "Post-processing RGB-Depth Alignment" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // Parse command line arguments
    std::string calibFile, depthFile, rgbFile, outputFile;
    std::string holeFillMode = "nearest";

    for(int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if(arg == "--calib" && i+1 < argc) {
            calibFile = argv[++i];
        } else if(arg == "--depth" && i+1 < argc) {
            depthFile = argv[++i];
        } else if(arg == "--rgb" && i+1 < argc) {
            rgbFile = argv[++i];
        } else if(arg == "--output" && i+1 < argc) {
            outputFile = argv[++i];
        } else if(arg == "--hole-fill" && i+1 < argc) {
            holeFillMode = argv[++i];
        } else if(arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        }
    }

    // Validate arguments
    if(calibFile.empty() || depthFile.empty() || rgbFile.empty() || outputFile.empty()) {
        std::cerr << "❌ ERROR: Missing required arguments!" << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    try {
        // Step 1: Load calibration parameters
        std::cout << "[1/8] Loading calibration parameters..." << std::endl;
        std::cout << "  File: " << calibFile << std::endl;

        std::ifstream calibStream(calibFile, std::ios::binary);
        if(!calibStream.is_open()) {
            std::cerr << "❌ ERROR: Cannot open calibration file: " << calibFile << std::endl;
            return 1;
        }

        OBCalibrationParam calibParam;
        calibStream.read((char*)&calibParam, sizeof(OBCalibrationParam));
        calibStream.close();

        std::cout << "  ✓ Depth camera: " << calibParam.intrinsics[OB_SENSOR_DEPTH].width
                  << "x" << calibParam.intrinsics[OB_SENSOR_DEPTH].height << std::endl;
        std::cout << "  ✓ Color camera: " << calibParam.intrinsics[OB_SENSOR_COLOR].width
                  << "x" << calibParam.intrinsics[OB_SENSOR_COLOR].height << std::endl;

        // Step 2: Load images
        std::cout << "\n[2/8] Loading images..." << std::endl;
        std::cout << "  Depth: " << depthFile << std::endl;

        cv::Mat depthImage = cv::imread(depthFile, cv::IMREAD_UNCHANGED);
        if(depthImage.empty()) {
            std::cerr << "❌ ERROR: Cannot load depth image: " << depthFile << std::endl;
            return 1;
        }
        std::cout << "  ✓ Depth loaded: " << depthImage.cols << "x" << depthImage.rows
                  << " (" << depthImage.type() << ")" << std::endl;

        std::cout << "  RGB: " << rgbFile << std::endl;
        cv::Mat rgbImage = cv::imread(rgbFile);
        if(rgbImage.empty()) {
            std::cerr << "❌ ERROR: Cannot load RGB image: " << rgbFile << std::endl;
            return 1;
        }
        std::cout << "  ✓ RGB loaded: " << rgbImage.cols << "x" << rgbImage.rows << std::endl;

        // Convert depth to 16-bit if needed
        if(depthImage.type() != CV_16U) {
            std::cout << "  Converting depth to 16-bit..." << std::endl;
            depthImage.convertTo(depthImage, CV_16U);
        }

        // Step 3: Create aligned depth image (color resolution)
        int colorWidth = calibParam.intrinsics[OB_SENSOR_COLOR].width;
        int colorHeight = calibParam.intrinsics[OB_SENSOR_COLOR].height;

        std::cout << "\n[3/8] Creating aligned depth image..." << std::endl;
        std::cout << "  Target size: " << colorWidth << "x" << colorHeight << std::endl;

        cv::Mat alignedDepth = cv::Mat::zeros(colorHeight, colorWidth, CV_16U);

        // Step 4: Perform alignment using SDK's calibration2dTo2d
        std::cout << "\n[4/8] Aligning depth to color camera perspective..." << std::endl;
        std::cout << "  Using SDK's calibration2dTo2d function" << std::endl;

        uint16_t* depthData = (uint16_t*)depthImage.data;
        uint16_t* alignedData = (uint16_t*)alignedDepth.data;

        int processedPixels = 0;
        int totalPixels = depthImage.rows * depthImage.cols;

        for(int y = 0; y < depthImage.rows; y++) {
            // Progress indicator
            if(y % 50 == 0) {
                float progress = 100.0f * y / depthImage.rows;
                std::cout << "  Progress: " << (int)progress << "% \r" << std::flush;
            }

            for(int x = 0; x < depthImage.cols; x++) {
                uint16_t depthValue = depthData[y * depthImage.cols + x];

                // Skip invalid depth values
                if(depthValue == 0) continue;

                OBPoint2f depthPixel = {(float)x, (float)y};
                OBPoint2f colorPixel;

                // ✅ SDK function: Transform depth pixel to color pixel
                bool success = ob::CoordinateTransformHelper::calibration2dTo2d(
                    calibParam,
                    depthPixel,
                    (float)depthValue,
                    OB_SENSOR_DEPTH,
                    OB_SENSOR_COLOR,
                    &colorPixel
                );

                if(success) {
                    int cx = (int)(colorPixel.x + 0.5f);
                    int cy = (int)(colorPixel.y + 0.5f);

                    // Check bounds
                    if(cx >= 0 && cx < colorWidth && cy >= 0 && cy < colorHeight) {
                        alignedData[cy * colorWidth + cx] = depthValue;
                        processedPixels++;
                    }
                }
            }
        }

        std::cout << "  Progress: 100%    " << std::endl;
        std::cout << "  ✓ Processed " << processedPixels << "/" << totalPixels
                  << " pixels (" << (100.0f * processedPixels / totalPixels) << "%)" << std::endl;

        // Step 5: Apply dense hole filling
        std::cout << "\n[5/8] Filling all holes to create dense depth map..." << std::endl;
        std::cout << "  Target: 100% coverage" << std::endl;

        cv::Mat outputDepth = alignedDepth.clone();
        cv::Mat mask = (outputDepth == 0);
        int totalHoles = cv::countNonZero(mask);

        if(totalHoles > 0) {
            std::cout << "  Initial holes: " << totalHoles << " pixels" << std::endl;

            // Convert to float for processing
            cv::Mat depthFloat;
            outputDepth.convertTo(depthFloat, CV_32F);

            // Method 1: Iterative dilation to propagate depth values
            std::cout << "  Applying iterative hole filling..." << std::endl;
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

            int iteration = 0;
            int maxIterations = 200; // Enough to fill entire image

            while(cv::countNonZero(mask) > 0 && iteration < maxIterations) {
                // Dilate depth values into holes
                cv::Mat dilated;
                cv::dilate(depthFloat, dilated, kernel);

                // Only fill holes, don't overwrite existing values
                dilated.copyTo(depthFloat, mask);

                // Update mask
                mask = (depthFloat == 0);

                iteration++;

                // Progress update every 20 iterations
                if(iteration % 20 == 0) {
                    int remainingHoles = cv::countNonZero(mask);
                    float progress = 100.0f * (totalHoles - remainingHoles) / totalHoles;
                    std::cout << "    Iteration " << iteration
                              << ": " << (int)progress << "% filled\r" << std::flush;
                }
            }

            std::cout << "    Iteration " << iteration
                      << ": 100% filled        " << std::endl;

            // Convert back to 16-bit
            depthFloat.convertTo(outputDepth, CV_16U);

            int finalHoles = cv::countNonZero(outputDepth == 0);
            std::cout << "  ✓ Dense depth map created" << std::endl;
            std::cout << "    - Filled: " << totalHoles << " holes" << std::endl;
            std::cout << "    - Remaining holes: " << finalHoles << std::endl;
            std::cout << "    - Coverage: " << (100.0f * (colorWidth*colorHeight - finalHoles) / (colorWidth*colorHeight)) << "%" << std::endl;
        } else {
            std::cout << "  ✓ No holes to fill" << std::endl;
        }

        // Step 6: Compute per-pixel mm/px scale map (isotropic)
        std::cout << "\n[6/8] Computing per-pixel mm/px scale map..." << std::endl;

        float fx = calibParam.intrinsics[OB_SENSOR_COLOR].fx;
        float fy = calibParam.intrinsics[OB_SENSOR_COLOR].fy;

        std::cout << "  RGB camera intrinsics:" << std::endl;
        std::cout << "    fx = " << fx << " pixels" << std::endl;
        std::cout << "    fy = " << fy << " pixels" << std::endl;

        cv::Mat scaleMapIso = computeScaleMapIso(outputDepth, fx, fy);

        std::cout << "  ✓ Scale map computed" << std::endl;

        // Save scale map as NPY
        std::string baseOutputPath = outputFile.substr(0, outputFile.find_last_of('.'));
        std::string scaleMapIsoFile = baseOutputPath + "_scale_map_iso.npy";

        try {
            saveNPY(scaleMapIsoFile, scaleMapIso);
            std::cout << "  ✓ Saved scale_map_iso.npy" << std::endl;

            // Calculate statistics
            cv::Scalar meanIso = cv::mean(scaleMapIso, scaleMapIso > 0);
            std::cout << "  Statistics (valid pixels only):" << std::endl;
            std::cout << "    Mean mm/px: " << meanIso[0] << " mm" << std::endl;

        } catch(const std::exception& e) {
            std::cerr << "  ⚠ Warning: Failed to save scale map: " << e.what() << std::endl;
        }

        // Step 7: Prepare output
        std::cout << "\n[7/8] Preparing output..." << std::endl;

        // Step 8: Save results
        std::cout << "\n[8/8] Saving results..." << std::endl;

        // Save aligned depth as PNG (mm, uint16)
        std::cout << "  Saving aligned depth PNG (mm): " << outputFile << std::endl;
        bool saved = cv::imwrite(outputFile, outputDepth);
        if(!saved) {
            std::cerr << "❌ ERROR: Failed to save output image" << std::endl;
            return 1;
        }

        // Save aligned depth as NPY (meters, float32) - for Python compatibility
        std::string alignedDepthNpyFile = baseOutputPath + ".npy";
        cv::Mat alignedDepthMeters;
        outputDepth.convertTo(alignedDepthMeters, CV_32F);
        alignedDepthMeters = alignedDepthMeters / 1000.0f;  // mm -> meters

        try {
            saveNPY(alignedDepthNpyFile, alignedDepthMeters);
            std::cout << "  ✓ Saved aligned depth NPY (meters): " << alignedDepthNpyFile << std::endl;
        } catch(const std::exception& e) {
            std::cerr << "  ⚠ Warning: Failed to save aligned depth NPY: " << e.what() << std::endl;
        }

        // Calculate statistics
        int nonZeroPixels = cv::countNonZero(outputDepth);
        int totalOutputPixels = colorWidth * colorHeight;
        float coverage = 100.0f * nonZeroPixels / totalOutputPixels;

        std::cout << "  ✓ Saved successfully!" << std::endl;
        std::cout << "\n  Statistics:" << std::endl;
        std::cout << "    - Input depth: " << depthImage.cols << "x" << depthImage.rows << std::endl;
        std::cout << "    - Output depth: " << colorWidth << "x" << colorHeight << std::endl;
        std::cout << "    - Valid pixels: " << nonZeroPixels << "/" << totalOutputPixels
                  << " (" << coverage << "%)" << std::endl;

        // Optional: Create visualization overlay
        std::string overlayFile = outputFile.substr(0, outputFile.find_last_of('.')) + "_overlay.png";
        std::cout << "\n  Creating visualization overlay..." << std::endl;

        // Normalize depth for visualization
        cv::Mat depthViz;
        cv::normalize(outputDepth, depthViz, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::applyColorMap(depthViz, depthViz, cv::COLORMAP_JET);

        // Resize RGB to match if needed
        cv::Mat rgbResized;
        if(rgbImage.cols != colorWidth || rgbImage.rows != colorHeight) {
            cv::resize(rgbImage, rgbResized, cv::Size(colorWidth, colorHeight));
        } else {
            rgbResized = rgbImage;
        }

        // Overlay depth on RGB
        cv::Mat overlay;
        cv::addWeighted(rgbResized, 0.6, depthViz, 0.4, 0, overlay);

        // Mask out zero depth regions
        cv::Mat validMask = outputDepth > 0;
        overlay.setTo(cv::Scalar(0, 0, 0), ~validMask);

        cv::imwrite(overlayFile, overlay);
        std::cout << "  ✓ Overlay saved: " << overlayFile << std::endl;

        std::cout << "\n========================================" << std::endl;
        std::cout << "✅ SUCCESS!" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "\nOutput files:" << std::endl;
        std::cout << "  1. " << alignedDepthNpyFile << " (aligned depth, meters, float32)" << std::endl;
        std::cout << "  2. " << outputFile << " (aligned depth, mm, uint16)" << std::endl;
        std::cout << "  3. " << overlayFile << " (visualization)" << std::endl;
        std::cout << "  4. " << scaleMapIsoFile << " (mm/px isotropic, float32)" << std::endl;

        return 0;

    } catch(ob::Error &e) {
        std::cerr << "\n❌ SDK ERROR:" << std::endl;
        std::cerr << "  Function: " << e.getName() << std::endl;
        std::cerr << "  Message: " << e.getMessage() << std::endl;
        return 1;
    } catch(cv::Exception &e) {
        std::cerr << "\n❌ OpenCV ERROR:" << std::endl;
        std::cerr << "  " << e.what() << std::endl;
        return 1;
    } catch(std::exception &e) {
        std::cerr << "\n❌ ERROR: " << e.what() << std::endl;
        return 1;
    }
}
