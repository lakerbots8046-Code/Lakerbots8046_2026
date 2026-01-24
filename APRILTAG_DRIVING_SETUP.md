# AprilTag Driving Setup Guide

## Overview
This document describes the vision-based AprilTag driving system implemented for the 2026 robot, including camera configurations, PathPlanner integration, and usage instructions.

## Camera Configuration

### Camera Positions (Updated)
Both cameras are mounted at the back of the robot:

**CAM_BL (Back Left Camera)**
- Position: X=-9.704571", Y=+10.8977515" (left), Z=8.264031"
- Rotation: Roll=0°, Pitch=65.405233° (tilted up), Yaw=170°
- Stream URL: http://photonvision.local:1184/stream.mjpg

**CAM_BR (Back Right Camera)**
- Position: X=-9.704571", Y=-10.8977517" (right), Z=8.264031"
- Rotation: Roll=0°, Pitch=65.405233° (tilted up), Yaw=190°
- Stream URL: http://photonvision.local:1182/stream.mjpg

### Coordinate System
- **X-axis**: Forward = positive, Backward = negative
- **Y-axis**: Left = positive, Right = negative
- **Z-axis**: Up = positive

## Features Implemented

### 1. Vision Pose Estimation
- Dual-camera AprilTag detection using PhotonVision
- Automatic pose fusion with swerve drivetrain odometry
- Multi-tag and single-tag pose estimation with different confidence levels
- Real-time pose updates published to SmartDashboard

### 2. Two Driving Modes

#### Mode A: Basic PID Control (`DriveToAprilTag`)
- Simple PID-based approach to drive to AprilTags
- Direct control of X, Y, and rotation
- Good for simple, direct approaches
- **Controller Binding**: Right Trigger (>50%)

#### Mode B: PathPlanner Integration (`DriveToAprilTagWithPathPlanner`)
- Advanced path planning with obstacle avoidance
- Smooth trajectory generation
- On-the-fly path creation from current position to target
- Better for complex navigation scenarios
- **Controller Bindings**: 
  - Right Bumper (drives to selected tag)
  - POV Buttons (drives to specific tags)

### 3. Controller Bindings

| Button | Function |
|--------|----------|
| **Right Bumper** | Drive to selected tag (PathPlanner mode) |
| **Right Trigger** | Drive to selected tag (PID mode) |
| **POV Up** | Drive to Tag 14 (PathPlanner) |
| **POV Right** | Drive to Tag 15 (PathPlanner) |
| **POV Down** | Drive to Tag 16 (PathPlanner) |
| **POV Left** | Drive to Tag 13 (PathPlanner) |
| **Left Bumper** | Reset field-centric heading |

### 4. SmartDashboard Integration

#### Vision Data
- `Vision/BL/Pose Valid` - Whether BL camera has valid pose
- `Vision/BR/Pose Valid` - Whether BR camera has valid pose
- `Vision/BL/Pose X/Y/Rotation` - BL camera pose estimate
- `Vision/BR/Pose X/Y/Rotation` - BR camera pose estimate
- `Vision/BL/Tags Used` - Number of tags used by BL camera
- `Vision/BR/Tags Used` - Number of tags used by BR camera

#### PathPlanner Status
- `PathPlanner/Status` - Current status of path following
- `PathPlanner/Target Tag` - Target AprilTag ID
- `PathPlanner/Distance to Target` - Distance remaining
- `PathPlanner/Rotation Error` - Rotation error in degrees

#### Basic PID Status
- `DriveToTag/Status` - Current status
- `DriveToTag/X Error` - X position error
- `DriveToTag/Y Error` - Y position error
- `DriveToTag/Rotation Error` - Rotation error

## Configuration Constants

### Vision Constants (Constants.java)
```java
// Target positioning
kTargetDistanceMeters = 1.0  // Distance to stop from tag
kPositionToleranceMeters = 0.05  // Position tolerance
kRotationToleranceDegrees = 2.0  // Rotation tolerance

// PID Gains (for basic mode)
kDriveP = 2.0
kStrafeP = 2.0
kRotationP = 3.0

// PathPlanner Constraints
kPathPlannerMaxVelocity = 3.0 m/s
kPathPlannerMaxAcceleration = 2.0 m/s²
kPathPlannerMaxAngularVelocity = π rad/s
kPathPlannerMaxAngularAcceleration = π rad/s²
```

## Setup Instructions

### 1. Install Dependencies
The following vendordeps are required:
- ✅ PhotonVision (`photonlib.json`)
- ✅ PathPlanner (`PathplannerLib.json`)
- ✅ Phoenix 6 (`Phoenix6-26.1.0.json`)

### 2. Configure PathPlanner
PathPlanner requires a robot configuration file. Create `pathplanner/settings.json` in the deploy directory with your robot's physical properties:
- Robot mass
- MOI (moment of inertia)
- Wheel base
- Track width
- Wheel radius
- Gear ratio

### 3. Camera Calibration
1. Ensure cameras are properly mounted and secured
2. Verify camera names in PhotonVision match Constants.java
3. Calibrate cameras using PhotonVision calibration tool
4. Test AprilTag detection before attempting autonomous driving

### 4. Test Procedure
1. **Static Test**: Verify cameras detect AprilTags
2. **Pose Estimation Test**: Check pose estimates on SmartDashboard
3. **Basic PID Test**: Test simple driving with Right Trigger
4. **PathPlanner Test**: Test advanced driving with Right Bumper
5. **Field Test**: Test on actual field with multiple tags

## Troubleshooting

### Camera Not Detecting Tags
- Check camera connection and power
- Verify camera name in PhotonVision matches Constants
- Check lighting conditions
- Ensure AprilTag is in camera field of view

### Poor Pose Estimation
- Recalibrate cameras
- Check camera mounting (ensure rigid, no vibration)
- Verify transform values in Constants.java
- Check for lens distortion

### PathPlanner Errors
- Ensure `pathplanner/settings.json` exists
- Verify robot configuration values
- Check for PathPlanner version compatibility
- Review console output for specific errors

### Robot Not Driving to Tag
- Verify pose estimation is working
- Check PID/PathPlanner constants
- Ensure target tag exists in field layout
- Check for controller input conflicts

## Future Improvements

### Potential Enhancements
1. **Dynamic target distance** based on game piece or scoring location
2. **Multi-tag selection** for optimal approach angle
3. **Obstacle avoidance** using additional sensors
4. **Auto-alignment** for precise scoring
5. **Vision-based odometry** as primary localization

### Tuning Recommendations
1. Adjust PID gains based on robot performance
2. Tune PathPlanner constraints for smoother paths
3. Optimize vision standard deviations for better fusion
4. Test different target distances for various game tasks

## Files Modified/Created

### New Files
- `src/main/java/frc/robot/commands/DriveToAprilTag.java`
- `src/main/java/frc/robot/commands/DriveToAprilTagWithPathPlanner.java`
- `vendordeps/PathplannerLib.json`
- `APRILTAG_DRIVING_SETUP.md` (this file)

### Modified Files
- `src/main/java/frc/robot/Constants.java` - Added camera transforms and driving constants
- `src/main/java/frc/robot/RobotContainer.java` - Added command bindings
- `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java` - Added PathPlanner configuration

## References
- [PhotonVision Documentation](https://docs.photonvision.org/)
- [PathPlanner Documentation](https://pathplanner.dev/)
- [WPILib AprilTag Documentation](https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/index.html)
- [FRC 2026 Game Manual](https://www.firstinspires.org/resource-library/frc/competition-manual-qa-system)

## Support
For issues or questions:
1. Check console output for error messages
2. Review SmartDashboard values for debugging
3. Consult team programming lead
4. Reference PhotonVision/PathPlanner documentation
