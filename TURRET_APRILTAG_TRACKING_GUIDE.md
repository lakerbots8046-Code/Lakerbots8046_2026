# Turret AprilTag Tracking Guide

## Overview

The turret now has AprilTag tracking functionality that allows it to automatically follow and stay locked onto an AprilTag while the robot is moving. The system intelligently handles rotation limits to prevent wiring damage by automatically wrapping around (rotating 360° the opposite direction) when necessary.

## Features

### 1. **Continuous AprilTag Tracking**
- Tracks the currently selected AprilTag (or a specific tag ID)
- Uses PID control for smooth, accurate tracking
- Maintains "locked in" state when aligned within tolerance
- Works with both back cameras (BL and BR)

### 2. **Rotation Limit Protection**
- **Limits**: -175° to +175° (350° total range with 10° safety margin)
- Prevents continuous rotation that could damage wiring
- Automatically detects when approaching limits

### 3. **Intelligent Wrap-Around**
- When the turret reaches a limit while tracking, it automatically rotates 360° the opposite way
- Seamlessly continues tracking after wrap-around
- Example: If at +175° and target moves further right, turret wraps to -170° and continues tracking

## How It Works

### Tracking Logic

1. **Target Detection**: Uses vision subsystem to detect the selected AprilTag
2. **Angle Calculation**: Calculates desired turret angle based on target yaw
3. **Limit Checking**: Checks if the desired angle would exceed rotation limits
4. **Wrap-Around Decision**: If near a limit and target is on opposite side, initiates wrap-around
5. **PID Control**: Smoothly rotates turret to align with target
6. **Lock Status**: Reports when turret is "locked in" (within tolerance)

### Wrap-Around Behavior

```
Scenario: Turret at +170°, target moves to require +180°

1. Detects near max limit (+175°)
2. Target requires rotation beyond limit
3. Initiates wrap-around to -170°
4. Rotates 360° counterclockwise
5. Arrives at -170° on opposite side
6. Resumes normal tracking
```

## Usage

### Controller Bindings

- **Y Button**: Hold to enable AprilTag tracking
  - Tracks the currently selected tag from the dashboard
  - Release to stop tracking

- **X Button**: Manual turret spin (for testing)
  - Hold to spin at dashboard-specified speed
  - Release to stop

### Dashboard Controls

#### SmartDashboard/Shuffleboard Outputs

**Turret Status:**
- `Turret/Status`: Current tracking status
  - "Tracking: LOCKED" - Aligned with target
  - "Tracking: Error X.X°" - Tracking but not aligned
  - "Tracking: No Target" - Target not visible
  - "Tracking: Wrapping Around" - Performing wrap-around rotation
  - "Tracking: Stopped" - Not tracking

**Tracking Metrics:**
- `Turret/Tracking Locked`: Boolean - true when locked onto target
- `Turret/Tracking Error`: Current error in degrees
- `Turret/Current Angle`: Current turret angle (-175° to +175°)
- `Turret/Target Angle`: Desired target angle
- `Turret/At Limit`: Boolean - true if at rotation limit

**Motor Telemetry:**
- `Turret/Motor Current (A)`: Current draw
- `Turret/Motor Temp (C)`: Motor temperature

**Vision Integration:**
- `Target Tag ID`: Currently selected tag to track
- `Target Tag Found`: Boolean - true if target is visible
- `Target Visible On`: Which camera sees the target

## Configuration

### Constants (in `Constants.java`)

#### Rotation Limits
```java
kMinRotationDegrees = -175.0;  // Minimum rotation angle
kMaxRotationDegrees = 175.0;   // Maximum rotation angle
kWrapAroundThreshold = 160.0;  // When to trigger wrap-around
```

#### Tracking PID
```java
kTrackingP = 0.02;   // Proportional gain (tune for responsiveness)
kTrackingI = 0.0;    // Integral gain (usually 0)
kTrackingD = 0.001;  // Derivative gain (tune for smoothness)
```

#### Tracking Behavior
```java
kTrackingToleranceDegrees = 3.0;  // "Locked in" tolerance
kMaxTrackingSpeed = 0.5;          // Max speed during tracking (50%)
kMinTrackingOutput = 0.05;        // Min output to overcome friction
```

#### Wrap-Around
```java
kWrapAroundSpeed = 0.3;           // Speed during wrap-around
kWrapAroundTargetOffset = 170.0;  // Target angle after wrap
```

## Tuning Guide

### PID Tuning

1. **Start with low P gain** (0.01)
   - Increase until turret responds quickly but doesn't oscillate
   - Current value: 0.02

2. **Add D gain if needed** (0.001)
   - Helps dampen oscillations
   - Increase if turret overshoots

3. **Avoid I gain** unless necessary
   - Can cause instability in tracking applications

### Speed Limits

- **kMaxTrackingSpeed**: Reduce if tracking is too aggressive
- **kMinTrackingOutput**: Increase if turret doesn't move when close to target

### Tolerance

- **kTrackingToleranceDegrees**: 
  - Smaller = more precise but may never "lock"
  - Larger = easier to lock but less precise
  - Current: 3.0° is a good balance

## Testing Procedure

### 1. Basic Tracking Test
1. Place an AprilTag in view of cameras
2. Select the tag ID on dashboard
3. Hold Y button on controller
4. Verify turret rotates to center on tag
5. Move robot slowly - turret should follow tag

### 2. Limit Test
1. Manually rotate turret to near +170°
2. Position tag to require further rotation
3. Hold Y button
4. Verify turret wraps around to negative side
5. Verify tracking continues after wrap

### 3. Lock Status Test
1. Enable tracking on a stationary tag
2. Wait for "Tracking: LOCKED" status
3. Verify `Turret/Tracking Locked` is true
4. Move robot slightly - should re-lock quickly

### 4. Multi-Camera Test
1. Position tag visible to both cameras
2. Enable tracking
3. Verify tracking works with either camera
4. Check dashboard shows which camera is being used

## Troubleshooting

### Turret doesn't track
- **Check**: Is target tag visible? (Check `Target Tag Found`)
- **Check**: Is correct tag selected on dashboard?
- **Check**: Are cameras connected? (Check camera status)
- **Solution**: Verify vision subsystem is working

### Turret oscillates around target
- **Cause**: P gain too high or D gain too low
- **Solution**: Reduce `kTrackingP` or increase `kTrackingD`

### Turret moves too slowly
- **Cause**: P gain too low or max speed too low
- **Solution**: Increase `kTrackingP` or `kMaxTrackingSpeed`

### Turret doesn't move when close to target
- **Cause**: Minimum output too low (friction)
- **Solution**: Increase `kMinTrackingOutput`

### Wrap-around happens too early
- **Cause**: Threshold too conservative
- **Solution**: Increase `kWrapAroundThreshold` (closer to 175°)

### Wrap-around doesn't complete
- **Cause**: Target offset too close to limit
- **Solution**: Adjust `kWrapAroundTargetOffset` to be further from limits

## Safety Features

1. **Rotation Limits**: Hard limits prevent wiring damage
2. **Automatic Stop**: Tracking stops if target is lost
3. **Emergency Stop**: Can be interrupted at any time by releasing Y button
4. **Brake Mode**: Motor uses brake mode for safety when stopped

## Code Structure

### Files Modified/Created

1. **Constants.java**
   - Added `TurretConstants` for limits and tracking parameters

2. **TurretSubsystem.java**
   - Added limit checking methods
   - Added wrap-around detection
   - Added angle normalization and clamping

3. **TrackAprilTagCommand.java** (NEW)
   - Main tracking command
   - PID control implementation
   - Wrap-around logic

4. **RobotContainer.java**
   - Added Y button binding for tracking
   - Imported TrackAprilTagCommand

## Future Enhancements

Possible improvements:
- Predictive tracking (lead the target based on robot velocity)
- Multi-tag tracking (switch between multiple tags)
- Auto-wrap based on robot heading (wrap before hitting limit)
- Field-relative tracking (account for robot rotation)
- Tracking profiles (aggressive vs smooth)

## Notes

- The turret uses the **currently selected tag** from the dashboard by default
- Tracking works while the robot is driving - the turret compensates for robot movement
- The wrap-around is automatic and requires no driver input
- PID values may need tuning based on your specific turret mechanism
- Always test rotation limits carefully before competition use
