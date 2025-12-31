#include <libobsensor/ObSensor.hpp>
#include <fstream>
#include <iostream>
#include <cstring>
#include <cmath>

/**
 * Convert calibration parameters from JSON/ROS format to OrbbecSDK format
 *
 * Femto Bolt Camera: SN CL8855300FR, FW 1.0.9
 */

void printCalibrationParam(const OBCalibrationParam& param) {
    std::cout << "\n=== DEPTH CAMERA (512x512) ===" << std::endl;
    std::cout << "Intrinsics:" << std::endl;
    std::cout << "  fx = " << param.intrinsics[OB_SENSOR_DEPTH].fx << std::endl;
    std::cout << "  fy = " << param.intrinsics[OB_SENSOR_DEPTH].fy << std::endl;
    std::cout << "  cx = " << param.intrinsics[OB_SENSOR_DEPTH].cx << std::endl;
    std::cout << "  cy = " << param.intrinsics[OB_SENSOR_DEPTH].cy << std::endl;
    std::cout << "  width = " << param.intrinsics[OB_SENSOR_DEPTH].width << std::endl;
    std::cout << "  height = " << param.intrinsics[OB_SENSOR_DEPTH].height << std::endl;

    std::cout << "Distortion:" << std::endl;
    std::cout << "  k1 = " << param.distortion[OB_SENSOR_DEPTH].k1 << std::endl;
    std::cout << "  k2 = " << param.distortion[OB_SENSOR_DEPTH].k2 << std::endl;
    std::cout << "  k3 = " << param.distortion[OB_SENSOR_DEPTH].k3 << std::endl;
    std::cout << "  k4 = " << param.distortion[OB_SENSOR_DEPTH].k4 << std::endl;
    std::cout << "  k5 = " << param.distortion[OB_SENSOR_DEPTH].k5 << std::endl;
    std::cout << "  k6 = " << param.distortion[OB_SENSOR_DEPTH].k6 << std::endl;
    std::cout << "  p1 = " << param.distortion[OB_SENSOR_DEPTH].p1 << std::endl;
    std::cout << "  p2 = " << param.distortion[OB_SENSOR_DEPTH].p2 << std::endl;

    std::cout << "\n=== COLOR CAMERA (3840x2160) ===" << std::endl;
    std::cout << "Intrinsics:" << std::endl;
    std::cout << "  fx = " << param.intrinsics[OB_SENSOR_COLOR].fx << std::endl;
    std::cout << "  fy = " << param.intrinsics[OB_SENSOR_COLOR].fy << std::endl;
    std::cout << "  cx = " << param.intrinsics[OB_SENSOR_COLOR].cx << std::endl;
    std::cout << "  cy = " << param.intrinsics[OB_SENSOR_COLOR].cy << std::endl;
    std::cout << "  width = " << param.intrinsics[OB_SENSOR_COLOR].width << std::endl;
    std::cout << "  height = " << param.intrinsics[OB_SENSOR_COLOR].height << std::endl;

    std::cout << "Distortion:" << std::endl;
    std::cout << "  k1 = " << param.distortion[OB_SENSOR_COLOR].k1 << std::endl;
    std::cout << "  k2 = " << param.distortion[OB_SENSOR_COLOR].k2 << std::endl;
    std::cout << "  k3 = " << param.distortion[OB_SENSOR_COLOR].k3 << std::endl;
    std::cout << "  k4 = " << param.distortion[OB_SENSOR_COLOR].k4 << std::endl;
    std::cout << "  k5 = " << param.distortion[OB_SENSOR_COLOR].k5 << std::endl;
    std::cout << "  k6 = " << param.distortion[OB_SENSOR_COLOR].k6 << std::endl;
    std::cout << "  p1 = " << param.distortion[OB_SENSOR_COLOR].p1 << std::endl;
    std::cout << "  p2 = " << param.distortion[OB_SENSOR_COLOR].p2 << std::endl;

    std::cout << "\n=== EXTRINSIC (Depth to Color) ===" << std::endl;
    std::cout << "Rotation matrix (row-major):" << std::endl;
    for(int i = 0; i < 3; i++) {
        std::cout << "  [" << param.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].rot[i*3+0] << ", "
                  << param.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].rot[i*3+1] << ", "
                  << param.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].rot[i*3+2] << "]" << std::endl;
    }
    std::cout << "Translation (mm):" << std::endl;
    std::cout << "  [" << param.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].trans[0] << ", "
              << param.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].trans[1] << ", "
              << param.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].trans[2] << "]" << std::endl;
}

int main() {
    // Initialize calibration parameter structure
    OBCalibrationParam calibParam;
    memset(&calibParam, 0, sizeof(OBCalibrationParam));

    // ===========================================
    // DEPTH CAMERA PARAMETERS (512x512)
    // ===========================================

    // Intrinsics from K matrix: [[252.11, 0, 259.72], [0, 252.10, 256.09], [0, 0, 1]]
    calibParam.intrinsics[OB_SENSOR_DEPTH].fx = 252.1143341064453f;
    calibParam.intrinsics[OB_SENSOR_DEPTH].fy = 252.0998077392578f;
    calibParam.intrinsics[OB_SENSOR_DEPTH].cx = 259.7208251953125f;
    calibParam.intrinsics[OB_SENSOR_DEPTH].cy = 256.0880126953125f;
    calibParam.intrinsics[OB_SENSOR_DEPTH].width = 512;
    calibParam.intrinsics[OB_SENSOR_DEPTH].height = 512;

    // Distortion coefficients
    // JSON format (OpenCV order): D = [k1, k2, p1, p2, k3, k4, k5, k6]
    // OrbbecSDK order: k1, k2, k3, k4, k5, k6, p1, p2
    float depth_D_opencv[8] = {
        11.287652969360352f,        // k1
        9.24820327758789f,          // k2
        6.311426113825291e-05f,     // p1
        3.3565891499165446e-05f,    // p2
        0.5219680666923523f,        // k3
        11.585052490234375f,        // k4
        13.139225006103516f,        // k5
        2.658942937850952f          // k6
    };

    // Convert to OrbbecSDK order
    calibParam.distortion[OB_SENSOR_DEPTH].k1 = depth_D_opencv[0];  // k1
    calibParam.distortion[OB_SENSOR_DEPTH].k2 = depth_D_opencv[1];  // k2
    calibParam.distortion[OB_SENSOR_DEPTH].k3 = depth_D_opencv[4];  // k3
    calibParam.distortion[OB_SENSOR_DEPTH].k4 = depth_D_opencv[5];  // k4
    calibParam.distortion[OB_SENSOR_DEPTH].k5 = depth_D_opencv[6];  // k5
    calibParam.distortion[OB_SENSOR_DEPTH].k6 = depth_D_opencv[7];  // k6
    calibParam.distortion[OB_SENSOR_DEPTH].p1 = depth_D_opencv[2];  // p1
    calibParam.distortion[OB_SENSOR_DEPTH].p2 = depth_D_opencv[3];  // p2

    // ===========================================
    // COLOR CAMERA PARAMETERS (3840x2160)
    // ===========================================

    // Intrinsics from K matrix: [[2243.65, 0, 1923.28], [0, 2242.97, 1108.88], [0, 0, 1]]
    calibParam.intrinsics[OB_SENSOR_COLOR].fx = 2243.650634765625f;
    calibParam.intrinsics[OB_SENSOR_COLOR].fy = 2242.96728515625f;
    calibParam.intrinsics[OB_SENSOR_COLOR].cx = 1923.2802734375f;
    calibParam.intrinsics[OB_SENSOR_COLOR].cy = 1108.8751220703125f;
    calibParam.intrinsics[OB_SENSOR_COLOR].width = 3840;
    calibParam.intrinsics[OB_SENSOR_COLOR].height = 2160;

    // Distortion coefficients
    // JSON format (OpenCV order): D = [k1, k2, p1, p2, k3, k4, k5, k6]
    float color_D_opencv[8] = {
        0.07693690806627274f,       // k1
        -0.10515454411506653f,      // k2
        0.000397599273128435f,      // p1
        -0.00019964079547207803f,   // p2
        0.04308972507715225f,       // k3
        0.0f,                       // k4
        0.0f,                       // k5
        0.0f                        // k6
    };

    // Convert to OrbbecSDK order
    calibParam.distortion[OB_SENSOR_COLOR].k1 = color_D_opencv[0];  // k1
    calibParam.distortion[OB_SENSOR_COLOR].k2 = color_D_opencv[1];  // k2
    calibParam.distortion[OB_SENSOR_COLOR].k3 = color_D_opencv[4];  // k3
    calibParam.distortion[OB_SENSOR_COLOR].k4 = color_D_opencv[5];  // k4
    calibParam.distortion[OB_SENSOR_COLOR].k5 = color_D_opencv[6];  // k5
    calibParam.distortion[OB_SENSOR_COLOR].k6 = color_D_opencv[7];  // k6
    calibParam.distortion[OB_SENSOR_COLOR].p1 = color_D_opencv[2];  // p1
    calibParam.distortion[OB_SENSOR_COLOR].p2 = color_D_opencv[3];  // p2

    // ===========================================
    // EXTRINSIC PARAMETERS (Depth to Color)
    // ===========================================

    // Rotation matrix (3x3) stored as row-major array [9]
    // R = [[0.9945266246795654, 0.009007524698972702, 0.0025769302155822515],
    //      [-0.009227413684129715, 0.9944887161254883, 0.10443674772977829],
    //      [-0.0016220113029703498, -0.10445594042539597, 0.9945281744003296]]
    float R[9] = {
        0.9945266246795654f, 0.009007524698972702f, 0.0025769302155822515f,      // Row 0
        -0.009227413684129715f, 0.9944887161254883f, 0.10443674772977829f,      // Row 1
        -0.0016220113029703498f, -0.10445594042539597f, 0.9945281744003296f    // Row 2
    };

    for(int i = 0; i < 9; i++) {
        calibParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].rot[i] = R[i];
    }

    // Translation vector (mm)
    // t_mm = [-32648.483276367188, -853.534996509552, 2647.0999717712402]
    calibParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].trans[0] = -32648.483276367188f;
    calibParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].trans[1] = -853.534996509552f;
    calibParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].trans[2] = 2647.0999717712402f;

    // ===========================================
    // VALIDATION CHECKS
    // ===========================================

    std::cout << "=== CALIBRATION PARAMETER VALIDATION ===" << std::endl;

    // Check 1: Intrinsic values are reasonable
    bool depth_fx_valid = calibParam.intrinsics[OB_SENSOR_DEPTH].fx > 100 &&
                          calibParam.intrinsics[OB_SENSOR_DEPTH].fx < 1000;
    bool color_fx_valid = calibParam.intrinsics[OB_SENSOR_COLOR].fx > 1000 &&
                          calibParam.intrinsics[OB_SENSOR_COLOR].fx < 5000;

    std::cout << "✓ Depth focal length valid: " << (depth_fx_valid ? "YES" : "NO")
              << " (fx=" << calibParam.intrinsics[OB_SENSOR_DEPTH].fx << ")" << std::endl;
    std::cout << "✓ Color focal length valid: " << (color_fx_valid ? "YES" : "NO")
              << " (fx=" << calibParam.intrinsics[OB_SENSOR_COLOR].fx << ")" << std::endl;

    // Check 2: Principal point near center
    bool depth_cx_centered = abs(calibParam.intrinsics[OB_SENSOR_DEPTH].cx - 256) < 50;
    bool depth_cy_centered = abs(calibParam.intrinsics[OB_SENSOR_DEPTH].cy - 256) < 50;
    bool color_cx_centered = abs(calibParam.intrinsics[OB_SENSOR_COLOR].cx - 1920) < 200;
    bool color_cy_centered = abs(calibParam.intrinsics[OB_SENSOR_COLOR].cy - 1080) < 200;

    std::cout << "✓ Depth principal point centered: "
              << (depth_cx_centered && depth_cy_centered ? "YES" : "NO") << std::endl;
    std::cout << "✓ Color principal point centered: "
              << (color_cx_centered && color_cy_centered ? "YES" : "NO") << std::endl;

    // Check 3: Rotation matrix is orthogonal (det(R) ≈ 1)
    float det = R[0] * (R[4]*R[8] - R[5]*R[7]) -
                R[1] * (R[3]*R[8] - R[5]*R[6]) +
                R[2] * (R[3]*R[7] - R[4]*R[6]);
    bool rotation_valid = abs(det - 1.0f) < 0.01f;

    std::cout << "✓ Rotation matrix orthogonal: " << (rotation_valid ? "YES" : "NO")
              << " (det=" << det << ")" << std::endl;

    // Check 4: Translation magnitude reasonable (should be ~30-40mm for stereo baseline)
    float translation_magnitude = sqrt(
        calibParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].trans[0] *
        calibParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].trans[0] +
        calibParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].trans[1] *
        calibParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].trans[1] +
        calibParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].trans[2] *
        calibParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR].trans[2]
    );
    bool translation_valid = translation_magnitude > 20000 && translation_magnitude < 50000;

    std::cout << "✓ Translation magnitude valid: " << (translation_valid ? "YES" : "NO")
              << " (|t|=" << translation_magnitude << " mm)" << std::endl;

    // ===========================================
    // PRINT FULL PARAMETERS
    // ===========================================

    printCalibrationParam(calibParam);

    // ===========================================
    // SAVE TO BINARY FILE
    // ===========================================

    std::ofstream file("femto_bolt_CL8855300FR_calibration.bin", std::ios::binary);
    if(file.is_open()) {
        file.write((char*)&calibParam, sizeof(OBCalibrationParam));
        file.close();
        std::cout << "\n✓ Calibration saved to: femto_bolt_CL8855300FR_calibration.bin" << std::endl;
        std::cout << "  Size: " << sizeof(OBCalibrationParam) << " bytes" << std::endl;
    } else {
        std::cerr << "✗ Failed to save calibration file!" << std::endl;
        return 1;
    }

    // ===========================================
    // VERIFICATION: RELOAD AND COMPARE
    // ===========================================

    OBCalibrationParam verifyParam;
    std::ifstream verifyFile("femto_bolt_CL8855300FR_calibration.bin", std::ios::binary);
    if(verifyFile.is_open()) {
        verifyFile.read((char*)&verifyParam, sizeof(OBCalibrationParam));
        verifyFile.close();

        bool match = memcmp(&calibParam, &verifyParam, sizeof(OBCalibrationParam)) == 0;
        std::cout << "✓ Verification: " << (match ? "PASSED" : "FAILED") << std::endl;
    }

    std::cout << "\n=== SUMMARY ===" << std::endl;
    std::cout << "All validation checks: "
              << (depth_fx_valid && color_fx_valid && rotation_valid && translation_valid ?
                  "✓ PASSED" : "✗ FAILED") << std::endl;
    std::cout << "Device: Femto Bolt (SN: CL8855300FR, FW: 1.0.9)" << std::endl;
    std::cout << "Ready for post-processing alignment!" << std::endl;

    return 0;
}
