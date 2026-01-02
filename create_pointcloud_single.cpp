#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Utils.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

void saveRGBDPointCloudToPly(const OBColorPoint* pointCloud, uint32_t width, uint32_t height,
                              const std::string& filename) {
    std::ofstream ofs(filename);
    if(!ofs.is_open()) {
        std::cerr << "Failed to open output file: " << filename << std::endl;
        return;
    }

    // Count valid points (non-zero depth)
    uint32_t validPoints = 0;
    for(uint32_t i = 0; i < width * height; i++) {
        if(pointCloud[i].z != 0) {
            validPoints++;
        }
    }

    std::cout << "  Valid points: " << validPoints << " / " << (width * height) << std::endl;

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

    // Write point cloud data (only valid points)
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
    std::cout << "✓ Point cloud saved: " << filename << std::endl;
}

int main(int argc, char** argv) {
    if(argc != 4) {
        std::cout << "Usage: " << argv[0] << " <calibration.bin> <rgb_image.png> <aligned_depth.png>" << std::endl;
        std::cout << "\nExample:" << std::endl;
        std::cout << "  " << argv[0] << " femto_bolt_calibration.bin camera_RGB_001.png aligned_depth_001.png" << std::endl;
        return -1;
    }

    std::string calibFile = argv[1];
    std::string rgbFile = argv[2];
    std::string depthFile = argv[3];

    std::cout << "\n========================================" << std::endl;
    std::cout << "RGBD Point Cloud Generation" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // Step 1: Load calibration parameters
    std::cout << "[1/6] Loading calibration parameters..." << std::endl;
    std::ifstream calibStream(calibFile, std::ios::binary);
    if(!calibStream.is_open()) {
        std::cerr << "✗ Failed to open calibration file: " << calibFile << std::endl;
        return -1;
    }

    OBCalibrationParam calibParam;
    calibStream.read(reinterpret_cast<char*>(&calibParam), sizeof(OBCalibrationParam));
    calibStream.close();

    std::cout << "  RGB: " << calibParam.intrinsics[OB_SENSOR_COLOR].width << "x"
              << calibParam.intrinsics[OB_SENSOR_COLOR].height << std::endl;
    std::cout << "  Depth: " << calibParam.intrinsics[OB_SENSOR_DEPTH].width << "x"
              << calibParam.intrinsics[OB_SENSOR_DEPTH].height << std::endl;
    std::cout << "✓ Calibration loaded\n" << std::endl;

    // Step 2: Load RGB image
    std::cout << "[2/6] Loading RGB image..." << std::endl;
    cv::Mat rgbImage = cv::imread(rgbFile, cv::IMREAD_COLOR);
    if(rgbImage.empty()) {
        std::cerr << "✗ Failed to load RGB image: " << rgbFile << std::endl;
        return -1;
    }

    // Convert BGR to RGB (OpenCV loads as BGR)
    cv::Mat rgbImageRGB;
    cv::cvtColor(rgbImage, rgbImageRGB, cv::COLOR_BGR2RGB);

    std::cout << "  Resolution: " << rgbImage.cols << "x" << rgbImage.rows << std::endl;
    std::cout << "✓ RGB image loaded\n" << std::endl;

    // Step 3: Load aligned depth image
    std::cout << "[3/6] Loading aligned depth image..." << std::endl;
    cv::Mat depthImage = cv::imread(depthFile, cv::IMREAD_ANYDEPTH);
    if(depthImage.empty()) {
        std::cerr << "✗ Failed to load depth image: " << depthFile << std::endl;
        return -1;
    }

    std::cout << "  Resolution: " << depthImage.cols << "x" << depthImage.rows << std::endl;
    std::cout << "  Bit depth: " << (depthImage.type() == CV_16U ? "16-bit" : "unknown") << std::endl;

    // Count valid depth pixels
    int validDepthPixels = cv::countNonZero(depthImage);
    float coverage = 100.0f * validDepthPixels / (depthImage.cols * depthImage.rows);
    std::cout << "  Valid depth pixels: " << validDepthPixels
              << " (" << std::fixed << std::setprecision(2) << coverage << "%)" << std::endl;
    std::cout << "✓ Aligned depth loaded\n" << std::endl;

    // Verify dimensions match
    if(rgbImage.cols != depthImage.cols || rgbImage.rows != depthImage.rows) {
        std::cerr << "✗ Error: RGB and depth dimensions don't match!" << std::endl;
        std::cerr << "  RGB: " << rgbImage.cols << "x" << rgbImage.rows << std::endl;
        std::cerr << "  Depth: " << depthImage.cols << "x" << depthImage.rows << std::endl;
        return -1;
    }

    uint32_t width = rgbImage.cols;
    uint32_t height = rgbImage.rows;

    // Step 4: Initialize XY tables for RGB resolution
    std::cout << "[4/6] Initializing XY tables (RGB resolution)..." << std::endl;
    uint32_t tableSize = width * height * 2;
    float* xyTableData = new float[tableSize];
    OBXYTables xyTables;

    bool success = ob::CoordinateTransformHelper::transformationInitXYTables(
        calibParam,
        OB_SENSOR_COLOR,  // Use COLOR sensor = RGB resolution
        xyTableData,
        &tableSize,
        &xyTables
    );

    if(!success) {
        std::cerr << "✗ Failed to initialize XY tables" << std::endl;
        delete[] xyTableData;
        return -1;
    }

    std::cout << "  Table size: " << tableSize << " floats" << std::endl;
    std::cout << "  Resolution: " << width << "x" << height << std::endl;
    std::cout << "✓ XY tables initialized\n" << std::endl;

    // Step 5: Generate RGBD point cloud
    std::cout << "[5/6] Generating RGBD point cloud..." << std::endl;
    uint32_t pointCloudSize = width * height;
    OBColorPoint* pointCloud = new OBColorPoint[pointCloudSize];
    memset(pointCloud, 0, pointCloudSize * sizeof(OBColorPoint));

    ob::CoordinateTransformHelper::transformationDepthToRGBDPointCloud(
        &xyTables,
        depthImage.data,      // Aligned depth (16-bit, RGB resolution)
        rgbImageRGB.data,     // RGB image (RGB888)
        pointCloud            // Output RGBD point cloud
    );

    std::cout << "✓ Point cloud generated\n" << std::endl;

    // Step 6: Save to PLY file
    std::cout << "[6/6] Saving point cloud..." << std::endl;

    // Extract base filename
    std::string outputFile = depthFile;
    size_t lastDot = outputFile.find_last_of(".");
    if(lastDot != std::string::npos) {
        outputFile = outputFile.substr(0, lastDot);
    }
    outputFile += "_pointcloud.ply";

    saveRGBDPointCloudToPly(pointCloud, width, height, outputFile);

    // Cleanup
    delete[] xyTableData;
    delete[] pointCloud;

    std::cout << "\n========================================" << std::endl;
    std::cout << "✓ Point cloud generation complete!" << std::endl;
    std::cout << "========================================\n" << std::endl;

    return 0;
}
