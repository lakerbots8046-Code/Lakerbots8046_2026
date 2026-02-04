# Intake Subsystem Updates - 2026 Robot Code

## Summary
Successfully enhanced the Intake subsystem with telemetry, helper methods, command factory methods, and centralized constants based on best practices from the 2025 code.

## Changes Made

### 1. Constants.java - Added IntakeConstants Section
**Location:** `src/main/java/frc/robot/Constants.java`

Added a new `IntakeConstants` class containing:

#### Motor Configuration
- `kIntakeCollectMotorID = 3` - Collection motor CAN ID
- `kIntakePivotMotorID = 2` - Pivot motor CAN ID
- `kCANBusName = "rio"` - CAN bus name

#### Pivot Position Setpoints (in rotations)
- `kPivotStowedPosition = 0.0` - Fully retracted/stowed
- `kPivotCollectPosition = 0.25` - Extended for collecting game pieces
- `kPivotScoreHighPosition = 0.15` - Position for high scoring
- `kPivotScoreLowPosition = 0.05` - Position for low scoring

#### Collection Velocities (in rotations per second)
- `kCollectIntakeVelocity = 30.0` - Speed when intaking game pieces
- `kCollectOuttakeVelocity = -20.0` - Speed when ejecting game pieces
- `kCollectHoldVelocity = 5.0` - Low speed to hold game piece
- `kCollectStopVelocity = 0.0` - Stop the collection motor

#### Control Parameters
- `kPivotPositionTolerance = 0.02` - Tolerance in rotations (±0.02)
- `kSensorToMechanismRatio = 12.8` - Gear ratio

#### SmartDashboard Keys
All telemetry keys are centralized with prefix `"Intake/"`:
- Pivot Position, Target, Error, At Target
- Collect Velocity, Current
- Pivot Current, Temperature
- Collect Temperature
- Status

### 2. Intake.java - Complete Subsystem Overhaul
**Location:** `src/main/java/frc/robot/subsystems/Intake.java`

#### Motor Initialization
- Motors now initialized using constants from `IntakeConstants`
- Proper final field initialization in constructor

#### New Helper Methods

**Pivot Position Control:**
- `setPivotPosition(double)` - Set pivot to specific position
- `getPivotPosition()` - Get current pivot position
- `getPivotVelocity()` - Get current pivot velocity
- `getPivotError()` - Get error between target and current position
- `isPivotAtTarget()` - Check if at target within tolerance

**Collection Velocity Control:**
- `setCollectVelocity(double)` - Set collection motor velocity
- `stopCollect()` - Stop collection motor
- `getCollectVelocity()` - Get current collection velocity

**Monitoring Methods:**
- `getPivotCurrent()` - Get pivot motor current draw
- `getCollectCurrent()` - Get collection motor current draw
- `getPivotTemperature()` - Get pivot motor temperature
- `getCollectTemperature()` - Get collection motor temperature

#### Command Factory Methods

**Pivot Positioning Commands:**
- `stowIntake()` - Move to stowed position
- `extendForCollect()` - Move to collect position
- `positionForScoreHigh()` - Move to high scoring position
- `positionForScoreLow()` - Move to low scoring position
- `goToPivotPosition(double)` - Move to custom position and wait

**Collection Commands:**
- `runIntake()` - Run collection at intake speed
- `runOuttake()` - Run collection at outtake speed
- `holdGamePiece()` - Hold game piece with low speed
- `stopCollection()` - Stop collection motor

#### SmartDashboard Integration
The `periodic()` method now publishes comprehensive telemetry:
- Pivot position, target, error, and at-target status
- Collection velocity
- Motor currents (both motors)
- Motor temperatures (both motors)
- Status string with formatted output
- Mechanism visualization updates

### 3. Removed Intake2025.java
**Deleted:** `src/main/java/frc/robot/subsystems/Intake2025.java`

This file was actually elevator code (copied from 2025 elevator subsystem) and not relevant to the intake mechanism. It has been removed to avoid confusion.

## Benefits of These Changes

### 1. **Centralized Configuration**
All intake-related constants are now in one place (`Constants.IntakeConstants`), making it easy to tune and adjust values.

### 2. **Comprehensive Telemetry**
Real-time monitoring of:
- Position and velocity
- Motor currents and temperatures
- Error tracking
- Status indicators

### 3. **Easy Command Integration**
Command factory methods make it simple to use the intake in autonomous and teleop:
```java
// Example usage in RobotContainer
intake.extendForCollect().andThen(intake.runIntake());
intake.stowIntake().andThen(intake.stopCollection());
```

### 4. **Safety and Monitoring**
- Current monitoring helps detect mechanical issues
- Temperature monitoring prevents motor damage
- Position tolerance checking ensures accurate control

### 5. **Code Clarity**
- Well-documented methods with JavaDoc
- Organized into logical sections
- Follows FRC best practices

## Next Steps

### Tuning Required
The following placeholder values should be tuned on the actual robot:

1. **Pivot Positions** - Adjust based on physical mechanism:
   - `kPivotStowedPosition`
   - `kPivotCollectPosition`
   - `kPivotScoreHighPosition`
   - `kPivotScoreLowPosition`

2. **Collection Velocities** - Adjust based on game piece and testing:
   - `kCollectIntakeVelocity`
   - `kCollectOuttakeVelocity`
   - `kCollectHoldVelocity`

3. **Tolerance** - Fine-tune based on mechanism precision:
   - `kPivotPositionTolerance`

### Integration with RobotContainer
Add intake commands to button bindings in `RobotContainer.java`:
```java
// Example button bindings
joystick.a().onTrue(intake.extendForCollect());
joystick.b().onTrue(intake.stowIntake());
joystick.x().whileTrue(intake.runIntake());
joystick.y().whileTrue(intake.runOuttake());
```

### Testing Checklist
- [ ] Verify motor IDs are correct (3 for collect, 2 for pivot)
- [ ] Test pivot positions are safe and functional
- [ ] Tune collection velocities for optimal performance
- [ ] Monitor currents during operation
- [ ] Check temperature limits under load
- [ ] Verify SmartDashboard outputs are correct
- [ ] Test all command factory methods
- [ ] Validate position tolerance is appropriate

## SmartDashboard Monitoring

All intake data will appear under the `Intake/` prefix:
- `Intake/Pivot Position` - Current pivot position
- `Intake/Pivot Target` - Target pivot position
- `Intake/Pivot Error` - Position error
- `Intake/Pivot At Target` - Boolean indicator
- `Intake/Collect Velocity` - Collection motor speed
- `Intake/Collect Current (A)` - Collection motor current
- `Intake/Pivot Current (A)` - Pivot motor current
- `Intake/Pivot Temp (C)` - Pivot motor temperature
- `Intake/Collect Temp (C)` - Collection motor temperature
- `Intake/Status` - Formatted status string

## Files Modified
1. ✅ `src/main/java/frc/robot/Constants.java` - Added IntakeConstants
2. ✅ `src/main/java/frc/robot/subsystems/Intake.java` - Complete enhancement
3. ✅ `src/main/java/frc/robot/subsystems/Intake2025.java` - Deleted (was elevator code)

## Comparison: Before vs After

### Before
- Hardcoded motor IDs
- Minimal helper methods
- No SmartDashboard integration
- No command factory methods
- Commented out position/velocity getters
- Unused example methods

### After
- Centralized constants
- Comprehensive helper methods
- Full SmartDashboard telemetry
- Rich command factory methods
- Working position/velocity getters
- Clean, production-ready code

---

**Created:** 2026
**Author:** BLACKBOXAI
**Status:** Ready for testing and tuning
