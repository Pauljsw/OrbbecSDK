#include <libobsensor/ObSensor.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>

/**
 * Simple calibration parameter extraction tool
 * For users new to SDK/C++/ROS
 *
 * This program:
 * 1. Connects to Femto Bolt camera
 * 2. Extracts calibration parameters
 * 3. Saves to binary file for post-processing alignment
 */

int main() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "Orbbec Femto Bolt Calibration Extractor" << std::endl;
    std::cout << "========================================\n" << std::endl;

    try {
        // Step 1: Initialize SDK context
        std::cout << "[1/6] Initializing Orbbec SDK..." << std::endl;
        ob::Context ctx;

        // Step 2: Get connected devices
        std::cout << "[2/6] Searching for connected cameras..." << std::endl;
        auto deviceList = ctx.queryDeviceList();

        if(deviceList->deviceCount() == 0) {
            std::cerr << "\n❌ ERROR: No camera found!" << std::endl;
            std::cerr << "\nPlease check:" << std::endl;
            std::cerr << "  1. Camera is plugged into USB port" << std::endl;
            std::cerr << "  2. Run: lsusb | grep -i orbbec" << std::endl;
            std::cerr << "  3. USB permissions: sudo bash misc/scripts/install_udev_rules.sh" << std::endl;
            return 1;
        }

        std::cout << "✓ Found " << deviceList->deviceCount() << " camera(s)" << std::endl;

        // Step 3: Connect to first device
        std::cout << "[3/6] Connecting to camera..." << std::endl;
        auto device = deviceList->getDevice(0);
        auto deviceInfo = device->getDeviceInfo();

        std::cout << "\n  Camera Information:" << std::endl;
        std::cout << "  - Name: " << deviceInfo->name() << std::endl;
        std::cout << "  - Serial Number: " << deviceInfo->serialNumber() << std::endl;
        std::cout << "  - Firmware: " << deviceInfo->firmwareVersion() << std::endl;
        std::cout << "  - USB Type: " << deviceInfo->usbType() << std::endl;

        // Step 4: Create pipeline and configure streams
        std::cout << "\n[4/6] Configuring camera streams..." << std::endl;
        ob::Pipeline pipeline(device);
        auto config = std::make_shared<ob::Config>();

        // Get available depth profiles
        auto depthProfiles = pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
        std::cout << "  Available Depth resolutions:" << std::endl;
        for(uint32_t i = 0; i < depthProfiles->count() && i < 5; i++) {
            auto profile = depthProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
            std::cout << "    - " << profile->width() << "x" << profile->height()
                      << " @ " << profile->fps() << "fps" << std::endl;
        }

        // Get available color profiles
        auto colorProfiles = pipeline.getStreamProfileList(OB_SENSOR_COLOR);
        std::cout << "  Available Color resolutions:" << std::endl;
        for(uint32_t i = 0; i < colorProfiles->count() && i < 5; i++) {
            auto profile = colorProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
            std::cout << "    - " << profile->width() << "x" << profile->height()
                      << " @ " << profile->fps() << "fps" << std::endl;
        }

        // Configure with your experimental resolutions
        std::cout << "\n  Configuring streams for calibration extraction:" << std::endl;

        // Depth: 512x512
        auto depthProfile = depthProfiles->getVideoStreamProfile(512, 512, OB_FORMAT_Y16, OB_FPS_ANY);
        if(!depthProfile) {
            std::cout << "  ⚠ 512x512 not available, using default depth profile" << std::endl;
            depthProfile = depthProfiles->getProfile(OB_PROFILE_DEFAULT)->as<ob::VideoStreamProfile>();
        }
        config->enableStream(depthProfile);
        std::cout << "  ✓ Depth: " << depthProfile->width() << "x" << depthProfile->height() << std::endl;

        // Color: 3840x2160
        auto colorProfile = colorProfiles->getVideoStreamProfile(3840, 2160, OB_FORMAT_RGB, OB_FPS_ANY);
        if(!colorProfile) {
            std::cout << "  ⚠ 3840x2160 not available, trying other resolutions..." << std::endl;
            colorProfile = colorProfiles->getVideoStreamProfile(1920, 1080, OB_FORMAT_RGB, OB_FPS_ANY);
            if(!colorProfile) {
                colorProfile = colorProfiles->getProfile(OB_PROFILE_DEFAULT)->as<ob::VideoStreamProfile>();
            }
        }
        config->enableStream(colorProfile);
        std::cout << "  ✓ Color: " << colorProfile->width() << "x" << colorProfile->height() << std::endl;

        // Step 5: Extract calibration parameters
        std::cout << "\n[5/6] Extracting calibration parameters..." << std::endl;

        // Must start pipeline to get calibration
        pipeline.start(config);

        // Get calibration parameters
        auto calibParam = pipeline.getCalibrationParam(config);

        // Stop pipeline (we only needed it for calibration)
        pipeline.stop();

        std::cout << "✓ Calibration parameters extracted successfully!" << std::endl;

        // Display parameters
        std::cout << "\n  === DEPTH CAMERA ===" << std::endl;
        std::cout << "  Resolution: " << calibParam.intrinsics[OB_SENSOR_DEPTH].width
                  << "x" << calibParam.intrinsics[OB_SENSOR_DEPTH].height << std::endl;
        std::cout << "  Focal length: fx=" << calibParam.intrinsics[OB_SENSOR_DEPTH].fx
                  << ", fy=" << calibParam.intrinsics[OB_SENSOR_DEPTH].fy << std::endl;
        std::cout << "  Principal point: cx=" << calibParam.intrinsics[OB_SENSOR_DEPTH].cx
                  << ", cy=" << calibParam.intrinsics[OB_SENSOR_DEPTH].cy << std::endl;

        std::cout << "\n  === COLOR CAMERA ===" << std::endl;
        std::cout << "  Resolution: " << calibParam.intrinsics[OB_SENSOR_COLOR].width
                  << "x" << calibParam.intrinsics[OB_SENSOR_COLOR].height << std::endl;
        std::cout << "  Focal length: fx=" << calibParam.intrinsics[OB_SENSOR_COLOR].fx
                  << ", fy=" << calibParam.intrinsics[OB_SENSOR_COLOR].fy << std::endl;
        std::cout << "  Principal point: cx=" << calibParam.intrinsics[OB_SENSOR_COLOR].cx
                  << ", cy=" << calibParam.intrinsics[OB_SENSOR_COLOR].cy << std::endl;

        std::cout << "\n  === EXTRINSIC (Depth to Color) ===" << std::endl;
        std::cout << "  Translation: ["
                  << calibParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].trans[0] << ", "
                  << calibParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].trans[1] << ", "
                  << calibParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].trans[2] << "] mm" << std::endl;

        // Step 6: Save to file
        std::cout << "\n[6/6] Saving calibration to file..." << std::endl;

        // Create filename with serial number
        std::string serialNumber = deviceInfo->serialNumber();
        std::string filename = "femto_bolt_" + serialNumber + "_calibration.bin";

        std::ofstream file(filename, std::ios::binary);
        if(!file.is_open()) {
            std::cerr << "❌ ERROR: Cannot create file: " << filename << std::endl;
            return 1;
        }

        file.write((char*)&calibParam, sizeof(OBCalibrationParam));
        file.close();

        std::cout << "✓ Saved to: " << filename << std::endl;
        std::cout << "  File size: " << sizeof(OBCalibrationParam) << " bytes" << std::endl;

        // Also save as JSON for human readability
        std::string jsonFilename = "femto_bolt_" + serialNumber + "_calibration.json";
        std::ofstream jsonFile(jsonFilename);
        if(jsonFile.is_open()) {
            jsonFile << std::fixed << std::setprecision(10);
            jsonFile << "{\n";
            jsonFile << "  \"device\": {\n";
            jsonFile << "    \"name\": \"" << deviceInfo->name() << "\",\n";
            jsonFile << "    \"serial_number\": \"" << serialNumber << "\",\n";
            jsonFile << "    \"firmware\": \"" << deviceInfo->firmwareVersion() << "\"\n";
            jsonFile << "  },\n";
            jsonFile << "  \"depth_camera\": {\n";
            jsonFile << "    \"width\": " << calibParam.intrinsics[OB_SENSOR_DEPTH].width << ",\n";
            jsonFile << "    \"height\": " << calibParam.intrinsics[OB_SENSOR_DEPTH].height << ",\n";
            jsonFile << "    \"fx\": " << calibParam.intrinsics[OB_SENSOR_DEPTH].fx << ",\n";
            jsonFile << "    \"fy\": " << calibParam.intrinsics[OB_SENSOR_DEPTH].fy << ",\n";
            jsonFile << "    \"cx\": " << calibParam.intrinsics[OB_SENSOR_DEPTH].cx << ",\n";
            jsonFile << "    \"cy\": " << calibParam.intrinsics[OB_SENSOR_DEPTH].cy << ",\n";
            jsonFile << "    \"distortion\": {\n";
            jsonFile << "      \"k1\": " << calibParam.distortion[OB_SENSOR_DEPTH].k1 << ",\n";
            jsonFile << "      \"k2\": " << calibParam.distortion[OB_SENSOR_DEPTH].k2 << ",\n";
            jsonFile << "      \"k3\": " << calibParam.distortion[OB_SENSOR_DEPTH].k3 << ",\n";
            jsonFile << "      \"k4\": " << calibParam.distortion[OB_SENSOR_DEPTH].k4 << ",\n";
            jsonFile << "      \"k5\": " << calibParam.distortion[OB_SENSOR_DEPTH].k5 << ",\n";
            jsonFile << "      \"k6\": " << calibParam.distortion[OB_SENSOR_DEPTH].k6 << ",\n";
            jsonFile << "      \"p1\": " << calibParam.distortion[OB_SENSOR_DEPTH].p1 << ",\n";
            jsonFile << "      \"p2\": " << calibParam.distortion[OB_SENSOR_DEPTH].p2 << "\n";
            jsonFile << "    }\n";
            jsonFile << "  },\n";
            jsonFile << "  \"color_camera\": {\n";
            jsonFile << "    \"width\": " << calibParam.intrinsics[OB_SENSOR_COLOR].width << ",\n";
            jsonFile << "    \"height\": " << calibParam.intrinsics[OB_SENSOR_COLOR].height << ",\n";
            jsonFile << "    \"fx\": " << calibParam.intrinsics[OB_SENSOR_COLOR].fx << ",\n";
            jsonFile << "    \"fy\": " << calibParam.intrinsics[OB_SENSOR_COLOR].fy << ",\n";
            jsonFile << "    \"cx\": " << calibParam.intrinsics[OB_SENSOR_COLOR].cx << ",\n";
            jsonFile << "    \"cy\": " << calibParam.intrinsics[OB_SENSOR_COLOR].cy << ",\n";
            jsonFile << "    \"distortion\": {\n";
            jsonFile << "      \"k1\": " << calibParam.distortion[OB_SENSOR_COLOR].k1 << ",\n";
            jsonFile << "      \"k2\": " << calibParam.distortion[OB_SENSOR_COLOR].k2 << ",\n";
            jsonFile << "      \"k3\": " << calibParam.distortion[OB_SENSOR_COLOR].k3 << ",\n";
            jsonFile << "      \"k4\": " << calibParam.distortion[OB_SENSOR_COLOR].k4 << ",\n";
            jsonFile << "      \"k5\": " << calibParam.distortion[OB_SENSOR_COLOR].k5 << ",\n";
            jsonFile << "      \"k6\": " << calibParam.distortion[OB_SENSOR_COLOR].k6 << ",\n";
            jsonFile << "      \"p1\": " << calibParam.distortion[OB_SENSOR_COLOR].p1 << ",\n";
            jsonFile << "      \"p2\": " << calibParam.distortion[OB_SENSOR_COLOR].p2 << "\n";
            jsonFile << "    }\n";
            jsonFile << "  },\n";
            jsonFile << "  \"extrinsic_depth_to_color\": {\n";
            jsonFile << "    \"rotation\": [\n";
            for(int i = 0; i < 3; i++) {
                jsonFile << "      ["
                         << calibParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].rot[i*3+0] << ", "
                         << calibParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].rot[i*3+1] << ", "
                         << calibParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].rot[i*3+2] << "]"
                         << (i < 2 ? "," : "") << "\n";
            }
            jsonFile << "    ],\n";
            jsonFile << "    \"translation_mm\": ["
                     << calibParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].trans[0] << ", "
                     << calibParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].trans[1] << ", "
                     << calibParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].trans[2] << "]\n";
            jsonFile << "  }\n";
            jsonFile << "}\n";
            jsonFile.close();
            std::cout << "✓ Human-readable version: " << jsonFilename << std::endl;
        }

        std::cout << "\n========================================" << std::endl;
        std::cout << "✅ SUCCESS!" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "\nNext steps:" << std::endl;
        std::cout << "1. Use the .bin file for post-processing alignment" << std::endl;
        std::cout << "2. Check the .json file to verify parameters" << std::endl;
        std::cout << "3. Keep both files in a safe place!" << std::endl;
        std::cout << "\nFiles created:" << std::endl;
        std::cout << "  - " << filename << " (for SDK)" << std::endl;
        std::cout << "  - " << jsonFilename << " (for viewing)" << std::endl;

        return 0;

    } catch(ob::Error &e) {
        std::cerr << "\n❌ SDK ERROR:" << std::endl;
        std::cerr << "  Function: " << e.getName() << std::endl;
        std::cerr << "  Message: " << e.getMessage() << std::endl;
        std::cerr << "  Type: " << e.getExceptionType() << std::endl;
        return 1;
    } catch(std::exception &e) {
        std::cerr << "\n❌ ERROR: " << e.what() << std::endl;
        return 1;
    }
}
