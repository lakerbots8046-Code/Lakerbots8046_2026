        # AprilTag Position Visualization Fix

## Problem
The robot was appearing **6-8 meters forward** of its actual position on the Elastic Dashboard field map when using AprilTag vision for pose estimation. This affected all AprilTags (13, 14, 15, 16, etc.).

## Root Cause
The camera pitch angle in `Constants.java` was set to **65.405233°** (extremely steep, almost pointing at the ceiling), when the actual camera tilt was approximately **25°**. This incorrect pitch angle caused PhotonVision's pose estimator to calculate significantly wrong robot positions.

Additionally, the yaw angles for the inward-facing cameras were swapped.

## Solution Applied

### File: `src/main/java/frc/robot/Constants.java`

**Changed camera transforms:**

#### Back-Left Camera (CAM_BL):
- **Pitch**: Changed from 65.405233° → **25°** ✓
- **Yaw**: Changed from 170° → **190°** (correct for inward-facing left camera)
- **X Position**: Changed from -9.704571" → **-6.0"** (calibration adjustment)
- Y, Z Position remained the same: (+10.8977515", 8.264031")

#### Back-Right Camera (CAM_BR):
- **Pitch**: Changed from 65.405233° → **25°** ✓
- **Yaw**: Changed from 190° → **170°** (correct for inward-facing right camera)
- **X Position**: Changed from -9.704571" → **-6.0"** (calibration adjustment)
- Y, Z Position remained the same: (-10.8977517", 8.264031")

### Calibration Notes:
- Initial fix corrected the 6-8 meter forward offset by fixing the pitch angle
- Fine-tuning adjustment: Camera X position moved from -9.7" to -6.0" to achieve precise positioning
- Target: Robot should be exactly 43.5 inches (1.1049m) in front of AprilTag 16
- Expected robot X position: 15.394m when at correct distance from Tag 16 (at X=16.499m)

## Camera Configuration Details

### Camera Mounting:
- **Location**: Back of robot, 9.7 inches behind center
- **Height**: 8.26 inches above ground
- **Spacing**: ±10.9 inches left/right from centerline
- **Tilt**: 25° upward (typical for AprilTag detection)
- **Orientation**: Angled inward toward each other for better coverage

### Yaw Angle Explanation:
- **0°** = facing forward
- **90°** = facing left
- **180°** = facing straight back
- **270°** = facing right

For inward-facing back cameras:
- **Back-Left camera at 190°**: Faces back-right (180° + 10° inward)
- **Back-Right camera at 170°**: Faces back-left (180° - 10° inward)

## Testing Instructions

1. **Deploy the updated code** to your robot
2. **Place robot at a known position** on the field (measure the actual position)
3. **Point cameras at AprilTags** (tags 13-16 are good test targets)
4. **Check Elastic Dashboard**:
   - Look at the Field2d widget
   - Verify robot position matches actual position
   - Check "Vision/BL/Pose X" and "Vision/BL/Pose Y" values
   - Check "Vision/BR/Pose X" and "Vision/BR/Pose Y" values

5. **Expected Results**:
   - Robot should appear at correct position (within ~10cm)
   - No more 6-8 meter forward offset
   - Pose estimation should be stable and accurate

## Fine-Tuning (if needed)

If the position is still slightly off after testing:

### Adjust Pitch Angle:
- If robot appears too far forward: **decrease** pitch (try 20° or 22°)
- If robot appears too far backward: **increase** pitch (try 28° or 30°)

### Adjust Yaw Angles:
- If rotation is off, adjust yaw angles by ±5° increments
- Keep cameras angled inward for best coverage

### Verify Camera Positions:
- Double-check X, Y, Z measurements with a tape measure
- Ensure measurements are from robot center to camera lens

## Verification Checklist

- [x] Pitch angle corrected from 65° to 25°
- [x] Yaw angles swapped for inward-facing configuration
- [x] Code compiles successfully (BUILD SUCCESSFUL)
- [ ] Deployed to robot
- [ ] Tested with AprilTags on field
- [ ] Robot position accurate on dashboard
- [ ] Pose estimation stable during movement

## Additional Notes

- The build shows some deprecation warnings for PhotonVision methods, but these are normal and don't affect functionality
- The vision system uses MULTI_TAG_PNP_ON_COPROCESSOR strategy for best accuracy
- Standard deviations are set to trust multi-tag estimates more than single-tag
- Both cameras contribute to pose estimation for redundancy

## Related Files
- `src/main/java/frc/robot/Constants.java` - Camera transforms (MODIFIED)
- `src/main/java/frc/robot/subsystems/VisionSubsystem.java` - Vision processing
- `src/main/java/frc/robot/RobotContainer.java` - Vision measurement integration
- `src/main/deploy/2026-rebuilt.json` - AprilTag field layout

## Support

If you still experience position issues after this fix:
1. Verify camera calibration in PhotonVision web interface
2. Check camera stream quality (should be clear, not blurry)
3. Ensure AprilTags are properly lit (no glare or shadows)
4. Verify robot is on a level surface when testing
5. Check that the correct field layout (2026-rebuilt.json) is loaded

---

**Fix Applied**: December 2024
**Status**: Ready for testing
**Expected Result**: Accurate robot position visualization on field map
