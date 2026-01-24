# SmartDashboard Reference Guide

## Overview
This document lists all SmartDashboard keys published by the robot code for monitoring and control.

## Camera Feeds

### Camera Stream URLs
To view camera feeds in SmartDashboard or Elastic:

| Key | Value | Description |
|-----|-------|-------------|
| `Camera/BL/Stream URL` | `http://photonvision.local:1184/stream.mjpg` | Back-Left camera MJPEG stream |
| `Camera/BR/Stream URL` | `http://photonvision.local:1182/stream.mjpg` | Back-Right camera MJPEG stream |
| `Camera/BL/Name` | `CAM_BL` | Back-Left camera name |
| `Camera/BR/Name` | `CAM_BR` | Back-Right camera name |

### How to Add Camera Feeds to Elastic Dashboard

**The camera streams are automatically published to NetworkTables!**

1. **In Elastic Dashboard:**
   - Look for `CameraPublisher/CAM_BL` in NetworkTables
   - Look for `CameraPublisher/CAM_BR` in NetworkTables
   - Add a "Camera Stream" widget
   - Select the camera from the dropdown (CAM_BL or CAM_BR)
   - The stream will automatically connect

2. **Alternative - Manual MJPEG Stream:**
   - Add an "MJPEG Stream" widget
   - Enter the stream URL:
     - BL: `http://photonvision.local:1184/stream.mjpg`
     - BR: `http://photonvision.local:1182/stream.mjpg`

3. **In SmartDashboard (Classic):**
   - The cameras appear under "CameraPublisher" in NetworkTables
   - Add a "Camera Stream" widget
   - Select CAM_BL or CAM_BR from the dropdown

## Vision System

### AprilTag Selector
| Key | Type | Description |
|-----|------|-------------|
| `Vision Tag Selector` | SendableChooser | Dropdown to select target AprilTag (1-24) |

### Target Information
| Key | Type | Description |
|-----|------|-------------|
| `Target Tag ID` | Number | Currently selected target tag ID |
| `Detected Tags` | String | Tags detected by both cameras |
| `Target Tag Found` | Boolean | Whether the selected tag is visible |
| `Target Visible On` | String | Which camera(s) see the target |

### Back-Left Camera (BL)
| Key | Type | Description |
|-----|------|-------------|
| `BL Camera Connected` | Boolean | Camera connection status |
| `BL Target Visible` | Boolean | Whether target is visible |
| `BL Detected Tag ID` | Number | ID of detected tag (-1 if none) |
| `BL Target Yaw (deg)` | Number | Horizontal angle to target |
| `BL Target Pitch (deg)` | Number | Vertical angle to target |
| `BL Target Area (%)` | Number | Target area as % of image |
| `BL Approx Distance` | Number | Estimated distance to target |
| `BL Total Targets` | Number | Total AprilTags visible |

### Back-Right Camera (BR)
| Key | Type | Description |
|-----|------|-------------|
| `BR Camera Connected` | Boolean | Camera connection status |
| `BR Target Visible` | Boolean | Whether target is visible |
| `BR Detected Tag ID` | Number | ID of detected tag (-1 if none) |
| `BR Target Yaw (deg)` | Number | Horizontal angle to target |
| `BR Target Pitch (deg)` | Number | Vertical angle to target |
| `BR Target Area (%)` | Number | Target area as % of image |
| `BR Approx Distance` | Number | Estimated distance to target |
| `BR Total Targets` | Number | Total AprilTags visible |

### Pose Estimation - Back-Left Camera
| Key | Type | Description |
|-----|------|-------------|
| `Vision/BL/Pose Valid` | Boolean | Whether pose estimate is valid |
| `Vision/BL/Pose X` | Number | Estimated X position (meters) |
| `Vision/BL/Pose Y` | Number | Estimated Y position (meters) |
| `Vision/BL/Pose Rotation` | Number | Estimated rotation (degrees) |
| `Vision/BL/Pose Timestamp` | Number | Timestamp of pose estimate |
| `Vision/BL/Tags Used` | Number | Number of tags used for estimate |
| `Vision/BL/Tags Used IDs` | String | Comma-separated list of tag IDs |

### Pose Estimation - Back-Right Camera
| Key | Type | Description |
|-----|------|-------------|
| `Vision/BR/Pose Valid` | Boolean | Whether pose estimate is valid |
| `Vision/BR/Pose X` | Number | Estimated X position (meters) |
| `Vision/BR/Pose Y` | Number | Estimated Y position (meters) |
| `Vision/BR/Pose Rotation` | Number | Estimated rotation (degrees) |
| `Vision/BR/Pose Timestamp` | Number | Timestamp of pose estimate |
| `Vision/BR/Tags Used` | Number | Number of tags used for estimate |
| `Vision/BR/Tags Used IDs` | String | Comma-separated list of tag IDs |

### Combined Vision Status
| Key | Type | Description |
|-----|------|-------------|
| `Vision/Any Pose Valid` | Boolean | Whether any camera has valid pose |
| `Vision/Total Tags Used` | Number | Total tags used by both cameras |
| `Vision/BL/Measurement Status` | String | BL camera measurement status |
| `Vision/BR/Measurement Status` | String | BR camera measurement status |

## PathPlanner Status

### Path Following
| Key | Type | Description |
|-----|------|-------------|
| `PathPlanner/Status` | String | Current path following status |
| `PathPlanner/Target Tag` | Number | Target AprilTag ID |
| `PathPlanner/Target X` | Number | Target X position (meters) |
| `PathPlanner/Target Y` | Number | Target Y position (meters) |
| `PathPlanner/Target Rotation` | Number | Target rotation (degrees) |
| `PathPlanner/Distance to Target` | Number | Distance remaining (meters) |
| `PathPlanner/Rotation Error (deg)` | Number | Rotation error (degrees) |
| `PathPlanner/At Target` | Boolean | Whether robot is at target |

## Basic PID Drive Status

### Drive to Tag (PID Mode)
| Key | Type | Description |
|-----|------|-------------|
| `DriveToTag/Status` | String | Current command status |
| `DriveToTag/Target Tag` | Number | Target AprilTag ID |
| `DriveToTag/X Error` | Number | X position error (meters) |
| `DriveToTag/Y Error` | Number | Y position error (meters) |
| `DriveToTag/Rotation Error (deg)` | Number | Rotation error (degrees) |
| `DriveToTag/Distance to Target` | Number | Distance to target (meters) |
| `DriveToTag/At Target` | Boolean | Whether at target position |
| `DriveToTag/Target X` | Number | Target X position (meters) |
| `DriveToTag/Target Y` | Number | Target Y position (meters) |
| `DriveToTag/Target Rotation` | Number | Target rotation (degrees) |

## Field Visualization

### Field2d Widget
| Key | Type | Description |
|-----|------|-------------|
| `Field2d` | Field2d | Field visualization with robot pose |
| `Field-Centric Status` | String | Field-centric reset status |

The Field2d widget shows:
- Current robot position and orientation
- Detected AprilTag positions (BL camera in one color, BR in another)
- Path planning trajectories (when using PathPlanner)

## Legacy Compatibility Keys

These keys are maintained for backward compatibility:

| Key | Type | Description |
|-----|------|-------------|
| `AprilTag [ID] Yaw BL` | Number | Yaw to selected tag (BL camera) |
| `AprilTag [ID] Yaw BR` | Number | Yaw to selected tag (BR camera) |
| `Vision Target Visible BL` | Boolean | Target visible (BL camera) |
| `Vision Target Visible BR` | Boolean | Target visible (BR camera) |

## Recommended Dashboard Layout

### Main Tab
1. **Camera Feeds** (top row)
   - BL Camera Stream (left)
   - BR Camera Stream (right)

2. **Vision Status** (middle row)
   - Vision Tag Selector (dropdown)
   - Target Tag Found (boolean indicator)
   - Detected Tags (text display)
   - Target Visible On (text display)

3. **Robot Position** (bottom row)
   - Field2d widget (large)
   - Vision/Any Pose Valid (boolean indicator)

### Detailed Vision Tab
1. **BL Camera Data**
   - All BL camera metrics
   - BL pose estimation data

2. **BR Camera Data**
   - All BR camera metrics
   - BR pose estimation data

3. **Combined Status**
   - Total tags used
   - Measurement status for both cameras

### Autonomous Tab
1. **Path Following**
   - PathPlanner status
   - Target information
   - Error metrics
   - Field2d widget

2. **Drive Commands**
   - DriveToTag status
   - Error values
   - Target position

## Tips for Using SmartDashboard

### Viewing Camera Feeds
1. Add MJPEG Stream widgets
2. Use the stream URLs from `Camera/BL/Stream URL` and `Camera/BR/Stream URL`
3. Adjust widget size for optimal viewing

### Monitoring Vision
1. Watch `Target Tag Found` to see if your selected tag is visible
2. Check `Detected Tags` to see all visible tags
3. Monitor pose estimation validity with `Vision/Any Pose Valid`

### Debugging Path Following
1. Watch `PathPlanner/Status` for current state
2. Monitor error values to tune PID constants
3. Use Field2d to visualize the path

### Selecting Target Tags
1. Use the `Vision Tag Selector` dropdown
2. Common tags for 2026 season:
   - Tags 13-16: Scoring positions
   - Tags 1-8: Field perimeter
   - Tags 9-12: Additional field markers

## Troubleshooting

### Camera Feed Not Showing
- Check `Camera/BL/Stream URL` and `Camera/BR/Stream URL` values
- Verify PhotonVision is running on the coprocessor
- Ensure network connectivity to `photonvision.local`

### No Pose Estimates
- Check `BL Camera Connected` and `BR Camera Connected`
- Verify AprilTags are visible in camera feeds
- Check lighting conditions
- Ensure camera calibration is complete

### Path Following Issues
- Monitor `PathPlanner/Status` for error messages
- Check `PathPlanner/Distance to Target` and error values
- Verify robot configuration in PathPlanner settings
- Review console output for detailed errors

## Additional Resources
- See `APRILTAG_DRIVING_SETUP.md` for setup instructions
- See `SMARTDASHBOARD_INTEGRATION.md` for integration details
- PhotonVision docs: https://docs.photonvision.org/
- WPILib SmartDashboard docs: https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/
