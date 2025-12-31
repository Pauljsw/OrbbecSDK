#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Utils.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cstring>
#include <sys/stat.h>
#include <dirent.h>
#include <chrono>

/**
 * Batch RGB-Depth Alignment Tool
 *
 * Processes all RGB-Depth image pairs in a directory
 * - Matches camera_RGB_*.png with camera_DPT_*.png by timestamp
 * - Aligns depth to RGB perspective
 * - Applies dense hole filling
 * - Saves aligned depth images
 *
 * Usage:
 *   ./post_align_batch \
 *     --calib femto_bolt_CL8855300FR_calibration.bin \
 *     --rgb-dir /path/to/rgb/ \
 *     --depth-dir /path/to/depth/ \
 *     --output-dir /path/to/output/
 */

struct ImagePair {
    std::string rgbPath;
    std::string depthPath;
    std::string timestamp;
};

void printUsage(const char* programName) {
    std::cout << "\nUsage:" << std::endl;
    std::cout << "  " << programName << " \\" << std::endl;
    std::cout << "    --calib <calibration.bin> \\" << std::endl;
    std::cout << "    --rgb-dir <rgb_directory> \\" << std::endl;
    std::cout << "    --depth-dir <depth_directory> \\" << std::endl;
    std::cout << "    --output-dir <output_directory>" << std::endl;
    std::cout << "\nExample:" << std::endl;
    std::cout << "  " << programName << " \\" << std::endl;
    std::cout << "    --calib femto_bolt_CL8855300FR_calibration.bin \\" << std::endl;
    std::cout << "    --rgb-dir /home/sungwoo/yoloclaude/ysfm_new/data/rgb/ \\" << std::endl;
    std::cout << "    --depth-dir /home/sungwoo/yoloclaude/ysfm_new/data/depth/ \\" << std::endl;
    std::cout << "    --output-dir /home/sungwoo/yoloclaude/ysfm_new/data/aligned_depth/" << std::endl;
}

bool createDirectory(const std::string& path) {
    struct stat st;
    if(stat(path.c_str(), &st) == 0) {
        return S_ISDIR(st.st_mode);
    }
    return mkdir(path.c_str(), 0755) == 0;
}

std::vector<std::string> listFiles(const std::string& directory, const std::string& prefix) {
    std::vector<std::string> files;
    DIR* dir = opendir(directory.c_str());
    if(!dir) return files;

    struct dirent* entry;
    while((entry = readdir(dir)) != NULL) {
        std::string filename = entry->d_name;
        if(filename.find(prefix) == 0 && filename.find(".png") != std::string::npos) {
            files.push_back(filename);
        }
    }
    closedir(dir);

    std::sort(files.begin(), files.end());
    return files;
}

std::string extractTimestamp(const std::string& filename) {
    // Extract timestamp from camera_XXX_<timestamp>_<timestamp>.png
    size_t firstUnderscore = filename.find('_');
    if(firstUnderscore == std::string::npos) return "";

    size_t secondUnderscore = filename.find('_', firstUnderscore + 1);
    if(secondUnderscore == std::string::npos) return "";

    size_t dotPng = filename.rfind(".png");
    if(dotPng == std::string::npos) return "";

    return filename.substr(secondUnderscore + 1, dotPng - secondUnderscore - 1);
}

cv::Mat alignDepthToRGB(const cv::Mat& depthImage,
                        const OBCalibrationParam& calibParam,
                        int colorWidth, int colorHeight,
                        int& processedPixels) {

    cv::Mat alignedDepth = cv::Mat::zeros(colorHeight, colorWidth, CV_16U);
    uint16_t* depthData = (uint16_t*)depthImage.data;
    uint16_t* alignedData = (uint16_t*)alignedDepth.data;

    processedPixels = 0;

    for(int y = 0; y < depthImage.rows; y++) {
        for(int x = 0; x < depthImage.cols; x++) {
            uint16_t depthValue = depthData[y * depthImage.cols + x];
            if(depthValue == 0) continue;

            OBPoint2f depthPixel = {(float)x, (float)y};
            OBPoint2f colorPixel;

            bool success = ob::CoordinateTransformHelper::calibration2dTo2d(
                calibParam, depthPixel, (float)depthValue,
                OB_SENSOR_DEPTH, OB_SENSOR_COLOR, &colorPixel
            );

            if(success) {
                int cx = (int)(colorPixel.x + 0.5f);
                int cy = (int)(colorPixel.y + 0.5f);

                if(cx >= 0 && cx < colorWidth && cy >= 0 && cy < colorHeight) {
                    alignedData[cy * colorWidth + cx] = depthValue;
                    processedPixels++;
                }
            }
        }
    }

    return alignedDepth;
}

cv::Mat applyDenseHoleFilling(const cv::Mat& sparseDepth, int maxIterations = 200) {
    cv::Mat outputDepth = sparseDepth.clone();
    cv::Mat mask = (outputDepth == 0);

    if(cv::countNonZero(mask) == 0) {
        return outputDepth;
    }

    cv::Mat depthFloat;
    outputDepth.convertTo(depthFloat, CV_32F);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

    int iteration = 0;
    while(cv::countNonZero(mask) > 0 && iteration < maxIterations) {
        cv::Mat dilated;
        cv::dilate(depthFloat, dilated, kernel);
        dilated.copyTo(depthFloat, mask);
        mask = (depthFloat == 0);
        iteration++;
    }

    depthFloat.convertTo(outputDepth, CV_16U);
    return outputDepth;
}

int main(int argc, char** argv) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "Batch RGB-Depth Alignment Processing" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // Parse arguments
    std::string calibFile, rgbDir, depthDir, outputDir;

    for(int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if(arg == "--calib" && i+1 < argc) {
            calibFile = argv[++i];
        } else if(arg == "--rgb-dir" && i+1 < argc) {
            rgbDir = argv[++i];
        } else if(arg == "--depth-dir" && i+1 < argc) {
            depthDir = argv[++i];
        } else if(arg == "--output-dir" && i+1 < argc) {
            outputDir = argv[++i];
        } else if(arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        }
    }

    if(calibFile.empty() || rgbDir.empty() || depthDir.empty() || outputDir.empty()) {
        std::cerr << "❌ ERROR: Missing required arguments!" << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    try {
        // Step 1: Load calibration
        std::cout << "[1/6] Loading calibration..." << std::endl;
        std::ifstream calibStream(calibFile, std::ios::binary);
        if(!calibStream.is_open()) {
            std::cerr << "❌ ERROR: Cannot open calibration file" << std::endl;
            return 1;
        }

        OBCalibrationParam calibParam;
        calibStream.read((char*)&calibParam, sizeof(OBCalibrationParam));
        calibStream.close();

        int colorWidth = calibParam.intrinsics[OB_SENSOR_COLOR].width;
        int colorHeight = calibParam.intrinsics[OB_SENSOR_COLOR].height;

        std::cout << "  ✓ Calibration loaded" << std::endl;
        std::cout << "    Target resolution: " << colorWidth << "x" << colorHeight << std::endl;

        // Step 2: Create output directory
        std::cout << "\n[2/6] Creating output directory..." << std::endl;
        if(!createDirectory(outputDir)) {
            std::cerr << "❌ ERROR: Cannot create output directory" << std::endl;
            return 1;
        }
        std::cout << "  ✓ Output directory ready: " << outputDir << std::endl;

        // Step 3: Scan directories
        std::cout << "\n[3/6] Scanning image directories..." << std::endl;
        auto rgbFiles = listFiles(rgbDir, "camera_RGB_");
        auto depthFiles = listFiles(depthDir, "camera_DPT_");

        std::cout << "  ✓ Found " << rgbFiles.size() << " RGB images" << std::endl;
        std::cout << "  ✓ Found " << depthFiles.size() << " Depth images" << std::endl;

        // Step 4: Match RGB-Depth pairs
        std::cout << "\n[4/6] Matching RGB-Depth pairs..." << std::endl;
        std::vector<ImagePair> pairs;

        for(const auto& rgbFile : rgbFiles) {
            std::string timestamp = extractTimestamp(rgbFile);
            if(timestamp.empty()) continue;

            std::string expectedDepthFile = "camera_DPT_" + timestamp + ".png";

            if(std::find(depthFiles.begin(), depthFiles.end(), expectedDepthFile) != depthFiles.end()) {
                ImagePair pair;
                pair.rgbPath = rgbDir + "/" + rgbFile;
                pair.depthPath = depthDir + "/" + expectedDepthFile;
                pair.timestamp = timestamp;
                pairs.push_back(pair);
            }
        }

        std::cout << "  ✓ Matched " << pairs.size() << " RGB-Depth pairs" << std::endl;

        if(pairs.empty()) {
            std::cerr << "❌ ERROR: No matching pairs found!" << std::endl;
            return 1;
        }

        // Step 5: Process all pairs
        std::cout << "\n[5/6] Processing image pairs..." << std::endl;
        std::cout << "  Total: " << pairs.size() << " pairs" << std::endl;
        std::cout << "  Progress:" << std::endl;

        int successCount = 0;
        int failCount = 0;
        auto startTime = std::chrono::steady_clock::now();

        for(size_t i = 0; i < pairs.size(); i++) {
            const auto& pair = pairs[i];

            // Progress indicator
            if(i % 10 == 0 || i == pairs.size() - 1) {
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - startTime).count();
                float progress = 100.0f * (i + 1) / pairs.size();
                float speed = (i + 1) / (float)(elapsed + 1);
                int eta = (pairs.size() - i - 1) / (speed + 0.001f);

                std::cout << "    [" << (i+1) << "/" << pairs.size() << "] "
                          << (int)progress << "% | "
                          << "Speed: " << speed << " img/s | "
                          << "ETA: " << eta << "s | "
                          << "Success: " << successCount << " | "
                          << "Fail: " << failCount << "    \r" << std::flush;
            }

            try {
                // Load depth image
                cv::Mat depthImage = cv::imread(pair.depthPath, cv::IMREAD_UNCHANGED);
                if(depthImage.empty()) {
                    failCount++;
                    continue;
                }

                if(depthImage.type() != CV_16U) {
                    depthImage.convertTo(depthImage, CV_16U);
                }

                // Align depth to RGB
                int processedPixels;
                cv::Mat alignedDepth = alignDepthToRGB(depthImage, calibParam,
                                                       colorWidth, colorHeight,
                                                       processedPixels);

                // Apply dense hole filling
                cv::Mat denseDepth = applyDenseHoleFilling(alignedDepth, 200);

                // Save result
                std::string outputFilename = "camera_ALIGNED_" + pair.timestamp + ".png";
                std::string outputPath = outputDir + "/" + outputFilename;

                if(!cv::imwrite(outputPath, denseDepth)) {
                    failCount++;
                    continue;
                }

                successCount++;

            } catch(std::exception& e) {
                failCount++;
            }
        }

        std::cout << std::endl;

        auto totalTime = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - startTime).count();

        // Step 6: Summary
        std::cout << "\n[6/6] Processing complete!" << std::endl;
        std::cout << "  Total time: " << totalTime << " seconds" << std::endl;
        std::cout << "  Average speed: " << (pairs.size() / (float)(totalTime + 1)) << " images/sec" << std::endl;

        std::cout << "\n========================================" << std::endl;
        std::cout << "✅ BATCH PROCESSING COMPLETE!" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "\nResults:" << std::endl;
        std::cout << "  ✓ Successfully processed: " << successCount << "/" << pairs.size() << std::endl;
        std::cout << "  ✗ Failed: " << failCount << std::endl;
        std::cout << "\nOutput directory:" << std::endl;
        std::cout << "  " << outputDir << std::endl;
        std::cout << "\nOutput files:" << std::endl;
        std::cout << "  camera_ALIGNED_<timestamp>.png (3840x2160, dense depth)" << std::endl;

        return (failCount == 0) ? 0 : 1;

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
