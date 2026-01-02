#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Utils.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sys/stat.h>

struct ImagePair {
    std::string rgbPath;
    std::string alignedDepthPath;
    std::string timestamp;
};

std::string extractTimestamp(const std::string& filename) {
    size_t start = filename.find("_");
    if(start == std::string::npos) return "";
    start++;

    size_t end = filename.find("_", start);
    if(end == std::string::npos) {
        end = filename.find(".", start);
    }

    if(end == std::string::npos) return "";
    return filename.substr(start, end - start);
}

void saveRGBDPointCloudToPly(const OBColorPoint* pointCloud, uint32_t width, uint32_t height,
                              const std::string& filename, bool verbose = false) {
    std::ofstream ofs(filename);
    if(!ofs.is_open()) {
        std::cerr << "Failed to open output file: " << filename << std::endl;
        return;
    }

    // Count valid points
    uint32_t validPoints = 0;
    for(uint32_t i = 0; i < width * height; i++) {
        if(pointCloud[i].z != 0) {
            validPoints++;
        }
    }

    if(verbose) {
        std::cout << "    Valid points: " << validPoints << " / " << (width * height) << std::endl;
    }

    // Write PLY header
    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "element vertex " << validPoints << "\n";
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    ofs << "property uchar red\n";
    ofs << "property uchar green\n";
    ofs << "property uchar blue\n";
    ofs << "end_header\n";

    // Write point cloud data
    for(uint32_t i = 0; i < width * height; i++) {
        if(pointCloud[i].z != 0) {
            ofs << pointCloud[i].x << " "
                << pointCloud[i].y << " "
                << pointCloud[i].z << " "
                << (int)pointCloud[i].r << " "
                << (int)pointCloud[i].g << " "
                << (int)pointCloud[i].b << "\n";
        }
    }

    ofs.close();
}

std::vector<ImagePair> findImagePairs(const std::string& rgbDir,
                                       const std::string& alignedDepthDir) {
    std::vector<ImagePair> pairs;

    // Get all RGB files
    std::vector<std::string> rgbFiles;
    for(const auto& entry : std::filesystem::directory_iterator(rgbDir)) {
        if(entry.is_regular_file()) {
            std::string filename = entry.path().filename().string();
            if(filename.find("camera_RGB_") == 0 || filename.find("RGB_") == 0) {
                rgbFiles.push_back(entry.path().string());
            }
        }
    }

    // Get all aligned depth files
    std::vector<std::string> depthFiles;
    for(const auto& entry : std::filesystem::directory_iterator(alignedDepthDir)) {
        if(entry.is_regular_file()) {
            std::string filename = entry.path().filename().string();
            if(filename.find("aligned_depth_") == 0) {
                depthFiles.push_back(entry.path().string());
            }
        }
    }

    std::cout << "Found " << rgbFiles.size() << " RGB images" << std::endl;
    std::cout << "Found " << depthFiles.size() << " aligned depth images" << std::endl;

    // Match by timestamp
    for(const auto& rgbPath : rgbFiles) {
        std::string rgbFilename = std::filesystem::path(rgbPath).filename().string();
        std::string timestamp = extractTimestamp(rgbFilename);

        if(timestamp.empty()) continue;

        // Find matching aligned depth
        for(const auto& depthPath : depthFiles) {
            std::string depthFilename = std::filesystem::path(depthPath).filename().string();
            if(depthFilename.find(timestamp) != std::string::npos) {
                ImagePair pair;
                pair.rgbPath = rgbPath;
                pair.alignedDepthPath = depthPath;
                pair.timestamp = timestamp;
                pairs.push_back(pair);
                break;
            }
        }
    }

    // Sort by timestamp
    std::sort(pairs.begin(), pairs.end(),
              [](const ImagePair& a, const ImagePair& b) {
                  return a.timestamp < b.timestamp;
              });

    return pairs;
}

int main(int argc, char** argv) {
    if(argc != 5) {
        std::cout << "Usage: " << argv[0] << " <calibration.bin> <rgb_dir> <aligned_depth_dir> <output_dir>" << std::endl;
        std::cout << "\nExample:" << std::endl;
        std::cout << "  " << argv[0] << " femto_bolt_calibration.bin \\" << std::endl;
        std::cout << "                 /path/to/rgb \\" << std::endl;
        std::cout << "                 /path/to/aligned_depth \\" << std::endl;
        std::cout << "                 /path/to/output_pointclouds" << std::endl;
        return -1;
    }

    std::string calibFile = argv[1];
    std::string rgbDir = argv[2];
    std::string alignedDepthDir = argv[3];
    std::string outputDir = argv[4];

    std::cout << "\n========================================" << std::endl;
    std::cout << "Batch RGBD Point Cloud Generation" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // Create output directory
    mkdir(outputDir.c_str(), 0755);

    // Step 1: Load calibration
    std::cout << "[1/5] Loading calibration parameters..." << std::endl;
    std::ifstream calibStream(calibFile, std::ios::binary);
    if(!calibStream.is_open()) {
        std::cerr << "✗ Failed to open calibration file: " << calibFile << std::endl;
        return -1;
    }

    OBCalibrationParam calibParam;
    calibStream.read(reinterpret_cast<char*>(&calibParam), sizeof(OBCalibrationParam));
    calibStream.close();

    uint32_t width = calibParam.intrinsics[OB_SENSOR_COLOR].width;
    uint32_t height = calibParam.intrinsics[OB_SENSOR_COLOR].height;

    std::cout << "  RGB resolution: " << width << "x" << height << std::endl;
    std::cout << "✓ Calibration loaded\n" << std::endl;

    // Step 2: Initialize XY tables (only once for all images)
    std::cout << "[2/5] Initializing XY tables..." << std::endl;
    uint32_t tableSize = width * height * 2;
    float* xyTableData = new float[tableSize];
    OBXYTables xyTables;

    bool success = ob::CoordinateTransformHelper::transformationInitXYTables(
        calibParam,
        OB_SENSOR_COLOR,
        xyTableData,
        &tableSize,
        &xyTables
    );

    if(!success) {
        std::cerr << "✗ Failed to initialize XY tables" << std::endl;
        delete[] xyTableData;
        return -1;
    }

    std::cout << "✓ XY tables initialized\n" << std::endl;

    // Step 3: Find image pairs
    std::cout << "[3/5] Finding image pairs..." << std::endl;
    std::vector<ImagePair> pairs = findImagePairs(rgbDir, alignedDepthDir);

    if(pairs.empty()) {
        std::cerr << "✗ No matching image pairs found!" << std::endl;
        delete[] xyTableData;
        return -1;
    }

    std::cout << "✓ Found " << pairs.size() << " matching pairs\n" << std::endl;

    // Allocate point cloud buffer (reuse for all images)
    uint32_t pointCloudSize = width * height;
    OBColorPoint* pointCloud = new OBColorPoint[pointCloudSize];

    // Step 4: Process all pairs
    std::cout << "[4/5] Generating point clouds..." << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    auto startTime = std::chrono::steady_clock::now();
    int successCount = 0;
    int failCount = 0;

    for(size_t i = 0; i < pairs.size(); i++) {
        const auto& pair = pairs[i];

        // Progress
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();

        std::cout << "\r[" << (i + 1) << "/" << pairs.size() << "] ";
        std::cout << "Processing: " << pair.timestamp << "... ";
        std::cout.flush();

        // Load RGB
        cv::Mat rgbImage = cv::imread(pair.rgbPath, cv::IMREAD_COLOR);
        if(rgbImage.empty()) {
            std::cout << "✗ Failed to load RGB" << std::endl;
            failCount++;
            continue;
        }

        cv::Mat rgbImageRGB;
        cv::cvtColor(rgbImage, rgbImageRGB, cv::COLOR_BGR2RGB);

        // Load aligned depth
        cv::Mat depthImage = cv::imread(pair.alignedDepthPath, cv::IMREAD_ANYDEPTH);
        if(depthImage.empty()) {
            std::cout << "✗ Failed to load depth" << std::endl;
            failCount++;
            continue;
        }

        // Verify dimensions
        if(rgbImage.cols != (int)width || rgbImage.rows != (int)height ||
           depthImage.cols != (int)width || depthImage.rows != (int)height) {
            std::cout << "✗ Dimension mismatch" << std::endl;
            failCount++;
            continue;
        }

        // Generate point cloud
        memset(pointCloud, 0, pointCloudSize * sizeof(OBColorPoint));

        ob::CoordinateTransformHelper::transformationDepthToRGBDPointCloud(
            &xyTables,
            depthImage.data,
            rgbImageRGB.data,
            pointCloud
        );

        // Save to PLY
        std::string outputFile = outputDir + "/pointcloud_" + pair.timestamp + ".ply";
        saveRGBDPointCloudToPly(pointCloud, width, height, outputFile, false);

        std::cout << "✓";
        successCount++;

        // ETA calculation
        if(i > 0 && elapsed > 0) {
            double avgTimePerImage = (double)elapsed / (i + 1);
            int remaining = pairs.size() - (i + 1);
            int eta = (int)(avgTimePerImage * remaining);

            std::cout << " (ETA: " << eta << "s)    ";
        }
        std::cout.flush();
    }

    std::cout << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto totalTime = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();

    // Step 5: Summary
    std::cout << "\n[5/5] Summary" << std::endl;
    std::cout << "  Total pairs: " << pairs.size() << std::endl;
    std::cout << "  Success: " << successCount << std::endl;
    std::cout << "  Failed: " << failCount << std::endl;
    std::cout << "  Total time: " << totalTime << "s" << std::endl;
    std::cout << "  Output directory: " << outputDir << std::endl;

    // Cleanup
    delete[] xyTableData;
    delete[] pointCloud;

    std::cout << "\n========================================" << std::endl;
    std::cout << "✓ Batch point cloud generation complete!" << std::endl;
    std::cout << "========================================\n" << std::endl;

    return 0;
}
