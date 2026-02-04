# ⚠️ DEPRECATED - Test Motor Subsystem Guide

**This guide is deprecated. The TestMotorSubsystem has been renamed to TurretSubsystem.**

**Please refer to [TURRET_SUBSYSTEM_GUIDE.md](TURRET_SUBSYSTEM_GUIDE.md) for current documentation.**

---

# Turret Subsystem (CAN ID 6 on RIO Bus) - LEGACY DOCUMENTATION

## Overview
Turret subsystem for controlling a TalonFX motor on CAN ID 6 (RIO bus) with SmartDashboard/Shuffleboard and controller controls.

**NOTE:** This documentation refers to the old `TestMotorSubsystem` class which has been renamed to `TurretSubsystem`. All functionality remains the same.

## Features
- **Dashboard Controls**: Start/Stop buttons
- **Adjustable Speed**: Set spin speed from dashboard
- **Real-time Monitoring**: Current, temperature, and status
- **Safety**: Brake mode enabled, emergency stop available

## Dashboard Controls

### In SmartDashboard/Shuffleboard:

**Control Buttons:**
- `Turret/Start Spin` - Click to start turret spinning
- `Turret/Stop Spin` - Click to stop turret

**Speed Control:**
- `Turret/Spin Speed` - Set desired speed (-1.0 to 1.0)
  - Default: 0.3 (30% power)
  - Positive = forward, Negative = reverse

**Status Displays:**
- `Turret/Is Spinning` - Shows if turret is currently running
- `Turret/Current Speed` - Current turret speed setting
- `Turret/Motor Current (A)` - Motor current draw in amps
- `Turret/Motor Temp (C)` - Motor temperature in Celsius
- `Turret/Status` - Text status (e.g., "Spinning at 30.0%" or "Stopped")

## How to Use

### Basic Operation:
1. **Deploy code** to robot
2. **Open SmartDashboard** or Shuffleboard
3. **Set speed** using `Turret/Spin Speed` slider/input
4. **Click** `Turret/Start Spin` button
5. **Monitor** current and temperature
6. **Click** `Turret/Stop Spin` when done

### Safety Notes:
- Motor is in **brake mode** (will resist movement when stopped)
- Speed is **clamped** between -1.0 and 1.0
- Monitor **current draw** to avoid overheating
- Watch **temperature** - stop if it gets too high (>80°C)

## Code Integration

### Already Integrated:
✅ Subsystem created: `TestMotorSubsystem.java`
✅ Added to RobotContainer
✅ Periodic updates automatic
✅ Dashboard controls initialized

### No Additional Setup Needed!
The subsystem is ready to use. Just deploy and test.

## Troubleshooting

**Motor doesn't spin:**
- Check CAN ID 6 is correct device on RIO bus
- Verify motor is connected to RIO CAN bus
- Check Phoenix Tuner to see if device appears
- Ensure robot is enabled

**Dashboard buttons don't work:**
- Refresh SmartDashboard/Shuffleboard
- Check robot code is deployed
- Verify NetworkTables connection

**High current draw:**
- Check for mechanical binding
- Reduce speed setting
- Inspect motor and gearbox

## Advanced Usage

### Programmatic Control:
```java
// In a command or other code:
turretSubsystem.startSpin();  // Start at dashboard speed
turretSubsystem.stopSpin();   // Stop turret
turretSubsystem.setSpeed(0.5); // Set to 50% power
turretSubsystem.emergencyStop(); // Immediate stop
```

### Get Status:
```java
boolean spinning = turretSubsystem.isSpinning();
double speed = turretSubsystem.getSpeed();
```

## Configuration

### Change CAN ID:
Edit `TestMotorSubsystem.java`:
```java
turretMotor = new TalonFX(6, "rio"); // Change 6 to your CAN ID, "rio" for RIO bus
```

### Change Default Speed:
Edit `TestMotorSubsystem.java`:
```java
private static final double DEFAULT_SPIN_SPEED = 0.3; // Change 0.3 to desired default
```

### Change Neutral Mode:
Edit `TestMotorSubsystem.java`:
```java
testMotor.setNeutralMode(NeutralModeValue.Coast); // Change to Coast mode
```

## Autonomous Command

### AimTurretAuto Command

A command for PathPlanner autonomous routines that aims the turret to 87 degrees.

**Usage in PathPlanner:**
1. Add "AimTurretAuto" as a named command in PathPlanner
2. The command will automatically:
   - Rotate turret from 0° to 87°
   - Finish when target is reached (within 2° tolerance)
   - Can be interrupted if needed

**Usage in Code:**
```java
// In RobotContainer or autonomous command groups
new AimTurretAuto(turretSubsystem)
```

**Features:**
- Target angle: 87 degrees
- Tolerance: ±2 degrees
- Uses position control (not speed control)
- Finishes automatically when target reached
- Console logging for debugging

**Important Notes:**
- Requires encoder to be zeroed at starting position (0°)
- Uses TalonFX built-in position control
- Adjust GEAR_RATIO in TestMotorSubsystem.java if needed

## Next Steps

1. **Zero the encoder** at turret's 0° position using `turretSubsystem.resetEncoder()`
2. **Test basic operation** with dashboard controls
3. **Test position control** with AimTurretAuto command
4. **Monitor telemetry** during operation
5. **Adjust GEAR_RATIO** if turret doesn't reach correct angle
6. **Add to PathPlanner** autonomous routines

---

**Created:** 2025
**Motor Controller:** CTRE TalonFX (Phoenix 6)
**CAN ID:** 6 (RIO Bus)
**Controller:** X Button (Hold to spin, release to stop)
