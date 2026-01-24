# Camera Measurement Guide

## Overview
To accurately integrate vision-based pose estimation, you need to measure the exact position and orientation of each camera relative to the robot's center. This guide will help you take the correct measurements.

## What You Need to Measure

For each camera (CAM_BL and CAM_BR), you need **6 measurements**:

### Position (3 measurements):
1. **X** - Forward/Backward distance from robot center
2. **Y** - Left/Right distance from robot center  
3. **Z** - Height above ground

### Rotation (3 measurements):
4. **Roll** - Rotation around X-axis (usually 0° for most cameras)
5. **Pitch** - Tilt up/down angle
6. **Yaw** - Horizontal rotation angle

---

## Step-by-Step Measurement Instructions

### Step 1: Find Your Robot Center
1. Measure the robot's wheelbase (front-to-back distance between wheel centers)
2. Measure the robot's track width (left-to-right distance between wheel centers)
3. The robot center is at the intersection of these two centerlines
4. Mark this point on your robot for reference

### Step 2: Measure Camera Positions

#### X Position (Forward/Backward):
- **What to measure**: Distance from robot center to camera lens (along the robot's forward/backward axis)
- **How to measure**: 
  1. Use a tape measure or ruler
  2. Measure from the robot center mark to the camera lens
  3. **Positive** = camera is forward of center
  4. **Negative** = camera is backward of center (your back cameras should be negative)
- **Example**: If camera is 2.5 inches behind robot center, X = -2.5 inches

#### Y Position (Left/Right):
- **What to measure**: Distance from robot centerline to camera lens (along the robot's left/right axis)
- **How to measure**:
  1. Measure from the robot's centerline to the camera lens
  2. **Positive** = camera is to the LEFT of centerline
  3. **Negative** = camera is to the RIGHT of centerline
- **Example**: 
  - CAM_BL (Back Left) at 9.75 inches left of center: Y = +9.75 inches
  - CAM_BR (Back Right) at 9.75 inches right of center: Y = -9.75 inches

#### Z Position (Height):
- **What to measure**: Height of camera lens above the ground
- **How to measure**:
  1. Measure from the floor to the center of the camera lens
  2. Always **positive** (unless camera is below ground, which shouldn't happen!)
- **Example**: If camera is 10.25 inches above ground, Z = 10.25 inches

### Step 3: Measure Camera Rotations

#### Roll (Rotation around X-axis):
- **What it is**: Camera tilting left or right
- **Typical value**: 0° (cameras are usually mounted level)
- **When to adjust**: Only if camera is intentionally tilted to one side

#### Pitch (Rotation around Y-axis):
- **What it is**: Camera tilting up or down
- **How to measure**:
  1. Use a protractor or angle finder
  2. Measure the angle between the camera's viewing direction and horizontal
  3. **Positive** = tilted UP
  4. **Negative** = tilted DOWN
- **Example**: If camera is tilted 15° upward, Pitch = +15°

#### Yaw (Rotation around Z-axis):
- **What it is**: Camera's horizontal rotation angle
- **How to measure**:
  1. Measure the angle between the camera's viewing direction and the robot's forward direction
  2. **0°** = camera faces forward
  3. **90°** = camera faces left
  4. **180°** = camera faces backward
  5. **270°** (or -90°) = camera faces right
- **For back cameras**:
  - If camera faces straight back: 180°
  - If camera is angled slightly outward for wider view:
    - Back-left camera: 180° - angle (e.g., 168° for 12° outward)
    - Back-right camera: 180° + angle (e.g., 192° for 12° outward)

---

## Measurement Template

Fill in these measurements for each camera:

### CAM_BL (Back Left Camera):
```
Position:
- X (Forward/Backward): _______ inches (should be negative for back-mounted)
- Y (Left/Right): _______ inches (should be positive for left side)
- Z (Height): _______ inches (always positive)

Rotation:
- Roll: _______ degrees (usually 0°)
- Pitch: _______ degrees (positive = tilted up)
- Yaw: _______ degrees (180° = straight back)
```

### CAM_BR (Back Right Camera):
```
Position:
- X (Forward/Backward): _______ inches (should be negative for back-mounted)
- Y (Left/Right): _______ inches (should be negative for right side)
- Z (Height): _______ inches (always positive)

Rotation:
- Roll: _______ degrees (usually 0°)
- Pitch: _______ degrees (positive = tilted up)
- Yaw: _______ degrees (180° = straight back)
```

---

## How to Update Constants.java

Once you have your measurements, update the camera transforms in `src/main/java/frc/robot/Constants.java`:

```java
public static class Vision {
    // ... other constants ...
    
    // Camera pitch angle (adjust based on your mounting)
    private static final double camPitch = Units.degreesToRadians(YOUR_PITCH_ANGLE);
    
    // Back Right Camera Transform
    public static final Transform3d kRobotToCamBR =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(YOUR_X_MEASUREMENT),  // X position
                Units.inchesToMeters(YOUR_Y_MEASUREMENT),  // Y position
                Units.inchesToMeters(YOUR_Z_MEASUREMENT)   // Z position (height)
            ),
            new Rotation3d(
                Units.degreesToRadians(YOUR_ROLL_ANGLE),   // Roll
                -camPitch,                                  // Pitch (negative because of coordinate system)
                Units.degreesToRadians(YOUR_YAW_ANGLE)     // Yaw
            )
        );
    
    // Back Left Camera Transform
    public static final Transform3d kRobotToCamBL =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(YOUR_X_MEASUREMENT),  // X position
                Units.inchesToMeters(YOUR_Y_MEASUREMENT),  // Y position
                Units.inchesToMeters(YOUR_Z_MEASUREMENT)   // Z position (height)
            ),
            new Rotation3d(
                Units.degreesToRadians(YOUR_ROLL_ANGLE),   // Roll
                -camPitch,                                  // Pitch (negative because of coordinate system)
                Units.degreesToRadians(YOUR_YAW_ANGLE)     // Yaw
            )
        );
}
```

---

## Example Measurements

Here's an example for back-mounted cameras:

### Example CAM_BL (Back Left):
```java
Position: X = -2.5", Y = +9.75", Z = 10.25"
Rotation: Roll = 0°, Pitch = 15° (tilted up), Yaw = 168° (12° outward from straight back)

Transform3d kRobotToCamBL = new Transform3d(
    new Translation3d(
        Units.inchesToMeters(-2.5),   // 2.5" behind center
        Units.inchesToMeters(9.75),   // 9.75" left of center
        Units.inchesToMeters(10.25)   // 10.25" above ground
    ),
    new Rotation3d(
        0,                                      // No roll
        Units.degreesToRadians(-15),           // 15° pitch up (negative in code)
        Units.degreesToRadians(168)            // Facing back-left
    )
);
```

### Example CAM_BR (Back Right):
```java
Position: X = -2.5", Y = -9.75", Z = 10.25"
Rotation: Roll = 0°, Pitch = 15° (tilted up), Yaw = 192° (12° outward from straight back)

Transform3d kRobotToCamBR = new Transform3d(
    new Translation3d(
        Units.inchesToMeters(-2.5),   // 2.5" behind center
        Units.inchesToMeters(-9.75),  // 9.75" right of center
        Units.inchesToMeters(10.25)   // 10.25" above ground
    ),
    new Rotation3d(
        0,                                      // No roll
        Units.degreesToRadians(-15),           // 15° pitch up (negative in code)
        Units.degreesToRadians(192)            // Facing back-right
    )
);
```

---

## Tips for Accurate Measurements

1. **Use metric if possible**: Measurements in millimeters are often more precise than inches
2. **Measure to the lens center**: Not the camera housing, but the actual lens
3. **Double-check**: Measure twice to ensure accuracy
4. **Use a level**: Ensure your robot is level when measuring heights
5. **Document**: Take photos of your measurements for future reference
6. **Test and adjust**: After initial measurements, test on the field and fine-tune if needed

---

## Verification

After updating your measurements:

1. Deploy code to robot
2. Place robot at a known position on the field
3. Point cameras at AprilTags
4. Check if the estimated pose matches the actual robot position
5. If off, adjust camera transforms slightly and retest

---

## Common Mistakes to Avoid

❌ **Wrong sign on Y coordinate**: Remember left is positive, right is negative
❌ **Forgetting negative X for back cameras**: Back-mounted cameras should have negative X
❌ **Measuring to camera housing instead of lens**: Always measure to the lens center
❌ **Not accounting for pitch**: If camera is tilted, you must include the pitch angle
❌ **Using wrong units**: Make sure to convert inches to meters using `Units.inchesToMeters()`

---

## Need Help?

If you're unsure about any measurements:
1. Take photos of your camera mounting
2. Measure what you can
3. Test with approximate values first
4. Fine-tune based on actual performance on the field

The vision system will still work with approximate values, but accurate measurements will give you the best pose estimation!
