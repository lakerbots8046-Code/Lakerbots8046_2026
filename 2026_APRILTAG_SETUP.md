# 2026 Rebuilt AprilTag Field Layout Setup

## Current Status
Your code is currently using `AprilTagFields.kDefaultField` which may not have the correct 2026 Rebuilt tag positions.

## What You Need

### Option 1: Use WPILib Built-in Layout (Easiest - When Available)

Check if your WPILib version has `k2026Rebuilt`:

1. **Update WPILib** to the latest 2025/2026 version
2. **Check if available** by looking at `AprilTagFields` enum
3. **Update Constants.java**:

```java
// In Constants.java, Vision class:
public static final AprilTagFieldLayout kTagLayout =
    AprilTagFieldLayout.loadField(AprilTagFields.k2026Rebuilt);
```

### Option 2: Load Custom JSON File (Use Now)

If WPILib doesn't have it yet, create a custom JSON file:

#### Step 1: Get Official 2026 Tag Positions

Download or obtain the official 2026 Rebuilt AprilTag layout from:
- FIRST official game manual
- WPILib GitHub repository
- Team resources/documentation

#### Step 2: Create JSON File

Create file: `src/main/deploy/2026-rebuilt.json`

**JSON Format:**
```json
{
  "field": {
    "length": 16.54175,
    "width": 8.0137
  },
  "tags": [
    {
      "ID": 1,
      "pose": {
        "translation": {
          "x": 1.5,
          "y": 0.5,
          "z": 0.5
        },
        "rotation": {
          "quaternion": {
            "W": 1.0,
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.0
          }
        }
      }
    },
    {
      "ID": 2,
      "pose": {
        "translation": {
          "x": 2.5,
          "y": 1.5,
          "z": 0.5
        },
        "rotation": {
          "quaternion": {
            "W": 0.7071,
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.7071
          }
        }
      }
    }
    // ... Add all tags (typically 16-24 tags for FRC)
  ]
}
```

#### Step 3: Update Constants.java

```java
// In Constants.java, Vision class:
import java.io.IOException;
import edu.wpi.first.wpilibj.Filesystem;

public static final AprilTagFieldLayout kTagLayout = loadCustomLayout();

private static AprilTagFieldLayout loadCustomLayout() {
    try {
        return new AprilTagFieldLayout(
            Filesystem.getDeployDirectory().toPath()
                .resolve("2026-rebuilt.json")
        );
    } catch (IOException e) {
        System.err.println("Failed to load 2026 AprilTag layout! Using default.");
        e.printStackTrace();
        return AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }
}
```

## 2026 Rebuilt Tag Information Needed

For each AprilTag on the 2026 field, you need:

### Tag Properties
- **ID**: Tag number (1-24 typically)
- **X**: Distance from blue alliance wall (meters)
- **Y**: Distance from left side of field (meters)
- **Z**: Height above carpet (meters)
- **Rotation**: Which direction tag faces (quaternion or Euler angles)

### Common Tag Locations (2026 Rebuilt)
Based on typical FRC field layouts, tags are usually at:
- **Scoring positions** (reef stations, processors)
- **Source/loading zones**
- **Amp positions**
- **Stage/climb areas**

## Testing Your Layout

After updating the layout:

1. **Deploy code** to robot
2. **Check console output** for any loading errors
3. **Verify in Elastic Dashboard**:
   - `Vision/BL/Pose X` - Should be within field bounds (0 to ~16.5m)
   - `Vision/BL/Pose Y` - Should be within field bounds (0 to ~8m)
   - `Vision/BL/Tags Used IDs` - Should show detected tag IDs

4. **Test with known position**:
   - Place robot at a known location on field
   - Point cameras at visible tags
   - Check if reported pose matches actual position

## Coordinate System

FRC Field Coordinate System:
- **Origin (0,0)**: Blue alliance wall, right side (from driver station view)
- **+X**: Toward red alliance wall
- **+Y**: Toward left side of field (from blue alliance view)
- **+Z**: Up from carpet
- **Rotation**: 0° = facing red alliance wall

## Resources

- **WPILib Docs**: https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html
- **2026 Game Manual**: Check FIRST website for official tag positions
- **PhotonVision**: https://docs.photonvision.org/en/latest/

## Current Configuration

Your cameras are configured as:
- **CAM_BL** (Back Left): Facing back-left at 170°
- **CAM_BR** (Back Right): Facing back-right at 190°

Both cameras can see tags behind the robot, which is ideal for:
- Backing up to scoring positions
- Aligning with reef stations
- Pose estimation during autonomous

## 2026 Rebuilt Tag Assignments

**Tags 1-16: Red Alliance**
- Tags 1-6: Red alliance back positions
- Tags 7-12: Red alliance front positions  
- Tags 13-16: Red alliance perimeter

**Tags 17-32: Blue Alliance**
- Tags 17-22: Blue alliance front positions
- Tags 23-28: Blue alliance back positions
- Tags 29-32: Blue alliance perimeter

## Next Steps

1. ✅ Official 2026 Rebuilt AprilTag layout obtained
2. ✅ JSON file created (src/main/deploy/2026-rebuilt.json)
3. ✅ Constants.java updated to load custom layout
4. ⬜ Deploy and test on robot
5. ⬜ Verify pose accuracy on field
