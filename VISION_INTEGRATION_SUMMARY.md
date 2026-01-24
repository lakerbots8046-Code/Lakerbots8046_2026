# Vision Integration Summary

## Overview
Successfully integrated PhotonVision dual-camera system from the 2025 Offseason project into the 2026 Robot Code. The system uses two back-mounted cameras (CAM_BL and CAM_BR) for AprilTag detection and pose estimation.

## Files Created/Modified

### New Files Created:
1. **vendordeps/photonlib.json** - PhotonVision library dependency (v2026.1.1-rc-2)
2. **src/main/java/frc/robot/subsystems/VisionSubsystem.java** - Main vision subsystem managing both cameras
3. **src/main/java/frc/robot/VisionSim.java** - Vision simulation support for testing

### Modified Files:
1. **src/main/java/frc/robot/Constants.java** - Added Vision constants class
2. **src/main/java/frc/robot/Robot.java** - Added vision updates and camera streaming
3. **src/main/java/frc/robot/RobotContainer.java** - Integrated vision subsystem and Field2d visualization

## Key Features Implemented

### 1. Dual Camera System
- **CAM_BL (Back Left)**: Back-left mounted camera
- **CAM_BR (Back Right)**: Back-right mounted camera
- Both cameras use IP: 10.80.46.11 (ports 1181 and 1184)

### 2. Vision Capabilities
- AprilTag detection and tracking
- Multi-tag pose estimation for improved accuracy
- Automatic pose correction using vision measurements
- Tag selection via SmartDashboard chooser
- Real-time telemetry for both cameras

### 3. Dashboard Integration
- Field2d visualization showing robot pose and detected tags
- Comprehensive SmartDashboard outputs:
  - Individual camera status (BL/BR)
  - Detected tag IDs
  - Target yaw, pitch, area, and distance
  - Pose estimation data
  - Camera connection status
  - Vision measurement status

### 4. Pose Estimation
- Integrates vision measurements into drivetrain odometry
- Uses different standard deviations for single-tag vs multi-tag detection
- Automatic fallback strategies for best accuracy

## Camera Configuration

### Current Camera Transforms (NEED TO BE UPDATED!)
The camera transforms in `Constants.Vision` are **placeholders** based on last season's front-mounted cameras. You **MUST** update these values to match your new back-mounted camera positions:

```java
// In Constants.Vision class:
public static final Transform3d kRobotToCamBR = new Transform3d(
    new Translation3d(
        Units.inchesToMeters(-2.5),  // X: Backward from robot center (MEASURE THIS!)
        Units.inchesToMeters(-9.75), // Y: Right from robot center (MEASURE THIS!)
        Units.inchesToMeters(10.25)  // Z: Up from robot center (MEASURE THIS!)
    ),
    new Rotation3d(0, -camPitch, Units.degreesToRadians(180 + 12)) // Rotation (ADJUST THIS!)
);

public static final Transform3d kRobotToCamBL = new Transform3d(
    new Translation3d(
        Units.inchesToMeters(-2.5),  // X: Backward from robot center (MEASURE THIS!)
        Units.inchesToMeters(9.75),  // Y: Left from robot center (MEASURE THIS!)
        Units.inchesToMeters(10.25)  // Z: Up from robot center (MEASURE THIS!)
    ),
    new Rotation3d(0, -camPitch, Units.degreesToRadians(180 - 12)) // Rotation (ADJUST THIS!)
);
```

### How to Measure Camera Positions:
1. **X (Forward/Backward)**: Measure from robot center to camera lens (negative = backward)
2. **Y (Left/Right)**: Measure from robot centerline to camera lens (positive = left, negative = right)
3. **Z (Up/Down)**: Measure from ground to camera lens height
4. **Rotation**: Measure camera angle relative to robot forward direction (180° = facing backward)

## Next Steps

### 1. Build the Project
Run the Gradle build to download PhotonVision dependencies:
```bash
./gradlew build
```

### 2. Update Camera Transforms
Measure and update the camera positions in `Constants.Vision`:
- `kRobotToCamBL` - Back Left camera transform
- `kRobotToCamBR` - Back Right camera transform

### 3. Verify Camera Connections
- Ensure both cameras are accessible at IP 10.80.46.11
- BL Camera stream: http://10.80.46.11:1181/stream.mjpg
- BR Camera stream: http://10.80.46.11:1184/stream.mjpg

### 4. Test Vision System
1. Deploy code to robot
2. Open SmartDashboard or Elastic
3. Check camera connection status
4. Verify AprilTag detection
5. Monitor pose estimation accuracy

### 5. Tune Standard Deviations (Optional)
Based on actual performance, you may want to adjust:
- `kSingleTagStdDevs` - Trust level for single-tag detections
- `kMultiTagStdDevs` - Trust level for multi-tag detections

Lower values = trust vision more
Higher values = trust odometry more

## SmartDashboard Outputs

### Camera-Specific Data:
- `BL Target Visible` / `BR Target Visible` - Whether target tag is visible
- `BL Detected Tag ID` / `BR Detected Tag ID` - ID of detected tag
- `BL Target Yaw (deg)` / `BR Target Yaw (deg)` - Horizontal angle to target
- `BL Target Pitch (deg)` / `BR Target Pitch (deg)` - Vertical angle to target
- `BL Target Area (%)` / `BR Target Area (%)` - Target size in camera view
- `BL Approx Distance` / `BR Approx Distance` - Estimated distance to target
- `BL Camera Connected` / `BR Camera Connected` - Camera connection status

### Combined Status:
- `Target Tag ID` - Currently selected tag to track
- `Detected Tags` - Summary of detected tags from both cameras
- `Target Tag Found` - Whether the selected tag is visible
- `Target Visible On` - Which camera(s) see the target

### Pose Estimation:
- `Vision/BL/Pose Valid` / `Vision/BR/Pose Valid` - Pose estimate available
- `Vision/BL/Pose X/Y/Rotation` - Estimated robot position from BL camera
- `Vision/BR/Pose X/Y/Rotation` - Estimated robot position from BR camera
- `Vision/BL/Tags Used` / `Vision/BR/Tags Used` - Number of tags used for pose
- `Vision/Any Pose Valid` - At least one camera has valid pose
- `Vision/Total Tags Used` - Total tags used across both cameras

### Field Visualization:
- `Field2d` - Shows robot position and detected AprilTags on field map

## Controls

### Left Bumper
Resets field-centric heading and updates vision pose estimators with current position.

## Troubleshooting

### Build Errors
If you see PhotonVision import errors, run:
```bash
./gradlew build
```
This will download the PhotonVision library.

### No Camera Connection
1. Verify Orange Pi is powered and connected
2. Check IP address (should be 10.80.46.11)
3. Verify camera names in PhotonVision match "CAM_BL" and "CAM_BR"
4. Check network connectivity

### Poor Pose Estimation
1. Verify camera transforms are accurate
2. Ensure cameras have clear view of AprilTags
3. Check lighting conditions
4. Adjust standard deviations if needed
5. Verify AprilTag field layout is correct for current season

### Tags Not Detected
1. Check camera exposure settings in PhotonVision
2. Verify AprilTags are in camera field of view
3. Check for obstructions
4. Ensure adequate lighting

## Technical Details

### Pose Estimation Strategy
- Primary: `MULTI_TAG_PNP_ON_COPROCESSOR` - Uses multiple tags when available
- Fallback: `LOWEST_AMBIGUITY` - Uses single tag with best confidence

### Vision Update Rate
- Updates every robot periodic cycle (20ms / 50Hz)
- Pose estimates integrated into drivetrain odometry automatically

### Coordinate System
- X: Forward (positive) / Backward (negative)
- Y: Left (positive) / Right (negative)  
- Z: Up (positive) / Down (negative)
- Rotation: Counter-clockwise positive (standard WPILib convention)

## Camera Names Changed
**Important**: Camera names were updated from last season:
- `CAM_FL` → `CAM_BL` (Front Left → Back Left)
- `CAM_FR` → `CAM_BR` (Front Right → Back Right)

Make sure to update camera names in PhotonVision configuration to match!

## Additional Resources
- PhotonVision Documentation: https://docs.photonvision.org
- WPILib AprilTag Documentation: https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/index.html
- CTRE Phoenix 6 Documentation: https://v6.docs.ctr-electronics.com/

---

**Created**: Vision integration completed successfully
**Status**: Ready for camera transform calibration and testing
