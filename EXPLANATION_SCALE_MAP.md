# Scale Map Generation - Technical Explanation

## Pinhole Camera Geometry

```
Side view of pinhole camera:

Physical space          Image plane

    |                      |
    |                      |
    | h (object)           | h' (image)
    |                      |
    |______________________|_________
         ←---- Z ----→  ←f→

    Camera center (C)      Image
```

Similar triangles:
```
h' / f = h / Z

Therefore:
h = (Z / f) · h'
```

Where:
- **Z**: Distance from camera to object (depth)
- **f**: Focal length
- **h**: Physical size of object
- **h'**: Size on image plane (in pixels)

**Key insight**: Physical size per pixel = Z / f

### X-direction scale (horizontal):

```
S_x = Z / f_x  [meters/pixel]
```

### Y-direction scale (vertical):

```
S_y = Z / f_y  [meters/pixel]
```

Convert to mm/pixel:
```
S_x = (Z / f_x) × 1000  [mm/pixel]
S_y = (Z / f_y) × 1000  [mm/pixel]
```

## Isotropic Scale Map

Since most cameras have f_x ≈ f_y (nearly square pixels), we compute the **average scale**:

```
S_iso = (S_x + S_y) / 2
      = (Z / f_x + Z / f_y) / 2 × 1000
      = Z × (1/f_x + 1/f_y) / 2 × 1000  [mm/pixel]
```

Equivalently:
```
f_avg = (f_x + f_y) / 2

S_iso = Z / f_avg × 1000  [mm/pixel]
```

## Why Isotropic Scale?

### Anisotropic case (S_x ≠ S_y):
- Non-square pixels
- Different scale in X and Y directions
- Complicates geometric calculations

### Isotropic case (S_iso):
- **Single scale value** per pixel
- Easier for distance/area measurements
- Valid when f_x ≈ f_y (typically < 1% difference)

## Example Calculation

Given RGB camera intrinsics:
```
f_x = 2771.23 pixels
f_y = 2768.45 pixels
f_avg = 2769.84 pixels
```

At different depths:

| Depth Z (mm) | S_iso (mm/px) | Meaning |
|--------------|---------------|---------|
| 500 | 0.18 | 1 pixel = 0.18mm |
| 1000 | 0.36 | 1 pixel = 0.36mm |
| 1500 | 0.54 | 1 pixel = 0.54mm |
| 2000 | 0.72 | 1 pixel = 0.72mm |

**Observation**: Scale increases linearly with depth

## Implementation

```cpp
cv::Mat computeScaleMapIso(const cv::Mat& alignedDepthMM,
                           float fx, float fy) {
    cv::Mat scaleMapIso = cv::Mat::zeros(
        alignedDepthMM.rows,
        alignedDepthMM.cols,
        CV_32F
    );

    for(int y = 0; y < alignedDepthMM.rows; y++) {
        for(int x = 0; x < alignedDepthMM.cols; x++) {
            uint16_t depthMM = alignedDepthMM.at<uint16_t>(y, x);

            if(depthMM > 0) {
                float depthM = depthMM / 1000.0f;

                // Per-direction scales
                float mmPerPxX = (depthM / fx) * 1000.0f;
                float mmPerPxY = (depthM / fy) * 1000.0f;

                // Isotropic (averaged) scale
                float mmPerPxIso = 0.5f * (mmPerPxX + mmPerPxY);

                scaleMapIso.at<float>(y, x) = mmPerPxIso;
            }
        }
    }

    return scaleMapIso;
}
```

## Applications

1. **Object measurement**:
   - Measure width in pixels: w_px
   - Physical width: w_mm = w_px × S_iso[u,v]

2. **Area calculation**:
   - Area in pixels: A_px
   - Physical area: A_mm² = A_px × S_iso[u,v]²

3. **SLAM/Odometry**:
   - Convert pixel displacement to metric displacement
   - Essential for visual odometry

4. **Robotics**:
   - Grasp planning
   - Navigation
   - Collision avoidance
