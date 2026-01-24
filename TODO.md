# Vision Integration TODO List

## ✅ Completed
- [x] Add PhotonVision dependency (photonlib.json)
- [x] Create VisionSubsystem with dual camera support (CAM_BL, CAM_BR)
- [x] Create VisionSim for simulation support
- [x] Add Vision constants to Constants.java
- [x] Integrate vision updates in Robot.java
- [x] Add vision measurement updates in RobotContainer.java
- [x] Add Field2d visualization
- [x] Configure camera streaming

## 🔧 Required Before Testing

### 1. Update Camera Transforms ⚠️ CRITICAL
**Location**: `src/main/java/frc/robot/Constants.java` (Vision class)

You MUST measure and update these values for your new camera mounting positions:

```java
// Back Right Camera (CAM_BR)
public static final Transform3d kRobotToCamBR = new Transform3d(
    new Translation3d(
        Units.inchesToMeters(-2.5),  // ← MEASURE: X distance (backward from center)
        Units.inchesToMeters(-9.75), // ← MEASURE: Y distance (right from center)
        Units.inchesToMeters(10.25)  // ← MEASURE: Z height (up from ground)
    ),
    new Rotation3d(0, -camPitch, Units.degreesToRadians(180 + 12)) // ← ADJUST: Rotation
);

// Back Left Camera (CAM_BL)
public static final Transform3d kRobotToCamBL = new Transform3d(
    new Translation3d(
        Units.inchesToMeters(-2.5),  // ← MEASURE: X distance (backward from center)
        Units.inchesToMeters(9.75),  // ← MEASURE: Y distance (left from center)
        Units.inchesToMeters(10.25)  // ← MEASURE: Z height (up from ground)
    ),
    new Rotation3d(0, -camPitch, Units.degreesToRadians(180 - 12)) // ← ADJUST: Rotation
);
```

**Measurement Guide**:
- X: Distance from robot center to camera (negative = backward)
- Y: Distance from robot centerline to camera (positive = left, negative = right)
- Z: Height from ground to camera lens
- Rotation: Camera angle (180° = facing backward, adjust ±12° for angled mounting)

### 2. Configure PhotonVision Camera Names
**On the Orange Pi / PhotonVision web interface**:
- [ ] Rename cameras to match code:
  - Camera 1: `CAM_BL` (Back Left)
  - Camera 2: `CAM_BR` (Back Right)
- [ ] Verify IP address: 10.80.46.11
- [ ] Verify stream ports: 1181 (BL), 1184 (BR)

### 3. Build and Deploy
- [ ] Run `./gradlew build` to compile (should be running now)
- [ ] Fix any compilation errors if they appear
- [ ] Deploy to robot: `./gradlew deploy`

## 🧪 Testing Checklist

### Initial Testing
- [ ] Verify robot code deploys successfully
- [ ] Check Driver Station for any errors
- [ ] Open SmartDashboard or Elastic dashboard

### Camera Connection Test
- [ ] Verify `BL Camera Connected` shows true
- [ ] Verify `BR Camera Connected` shows true
- [ ] Check camera streams are visible
- [ ] Verify no NetworkTables errors

### AprilTag Detection Test
- [ ] Place robot in view of AprilTags
- [ ] Check `Detected Tags` shows tag IDs
- [ ] Verify `BL Target Visible` / `BR Target Visible` update correctly
- [ ] Test tag selection using "Vision Tag Selector" chooser
- [ ] Verify yaw, pitch, area values update in real-time

### Pose Estimation Test
- [ ] Check `Vision/BL/Pose Valid` and `Vision/BR/Pose Valid`
- [ ] Verify pose X, Y, Rotation values are reasonable
- [ ] Check `Vision/Tags Used` shows number of tags
- [ ] Verify `Vision/Any Pose Valid` is true when tags visible
- [ ] Watch Field2d visualization for robot position updates

### Odometry Integration Test
- [ ] Drive robot around while viewing Field2d
- [ ] Verify robot position updates smoothly
- [ ] Check that vision corrections improve accuracy
- [ ] Test field-centric reset (Left Bumper)
- [ ] Verify pose doesn't jump erratically

## 🔍 Troubleshooting

### If cameras don't connect:
- [ ] Ping Orange Pi: `ping 10.80.46.11`
- [ ] Check PhotonVision web interface: http://10.80.46.11:5800
- [ ] Verify camera names match exactly: `CAM_BL`, `CAM_BR`
- [ ] Check network switch/connections
- [ ] Restart Orange Pi

### If tags aren't detected:
- [ ] Check camera exposure in PhotonVision settings
- [ ] Verify AprilTags are in camera field of view
- [ ] Check lighting conditions (not too bright/dark)
- [ ] Verify correct AprilTag field layout loaded
- [ ] Test with different tags

### If pose estimation is inaccurate:
- [ ] Double-check camera transform measurements
- [ ] Verify camera pitch angle is correct
- [ ] Check for camera lens distortion
- [ ] Adjust standard deviations in Constants.Vision
- [ ] Ensure multiple tags are visible for better accuracy

## 🎯 Optional Enhancements

### Performance Tuning
- [ ] Tune `kSingleTagStdDevs` based on actual single-tag accuracy
- [ ] Tune `kMultiTagStdDevs` based on actual multi-tag accuracy
- [ ] Adjust camera exposure for optimal tag detection
- [ ] Test different pose estimation strategies if needed

### Additional Features
- [ ] Add auto-alignment commands using vision
- [ ] Create vision-based autonomous routines
- [ ] Add vision-assisted driver controls
- [ ] Implement tag-specific behaviors

### Dashboard Improvements
- [ ] Customize Elastic dashboard layout for vision data
- [ ] Add graphs for pose estimation accuracy
- [ ] Create custom widgets for camera views
- [ ] Add alerts for vision system issues

## 📝 Notes

### Camera IP Configuration
- Orange Pi IP: 10.80.46.11
- BL Camera Stream: http://10.80.46.11:1181/stream.mjpg
- BR Camera Stream: http://10.80.46.11:1184/stream.mjpg
- PhotonVision UI: http://10.80.46.11:5800

### Important Files
- Vision Constants: `src/main/java/frc/robot/Constants.java`
- Vision Subsystem: `src/main/java/frc/robot/subsystems/VisionSubsystem.java`
- Vision Integration: `src/main/java/frc/robot/RobotContainer.java`
- Camera Streaming: `src/main/java/frc/robot/Robot.java`

### Key Changes from Last Season
- Camera names: FL/FR → BL/BR (Front → Back)
- Camera positions: Front-mounted → Back-mounted
- Same IP addresses and ports maintained

---

**Priority**: Update camera transforms FIRST before any testing!
**Status**: Integration complete, awaiting calibration and testing
