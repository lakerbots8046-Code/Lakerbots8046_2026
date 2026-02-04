# Turret Subsystem Guide

## Overview

The Turret subsystem provides precise control and positioning for the robot's turret mechanism. It supports both manual speed control and autonomous position-based aiming.

**Hardware:**
- Motor: TalonFX (Kraken X60)
- CAN ID: 6
- CAN Bus: RIO
- Neutral Mode: Brake (for safety)

## Features

### 1. Speed Control (Manual/Testing)
- Spin the turret at a specified speed (-100% to +100%)
- Dashboard controls for start/stop
- Adjustable speed via SmartDashboard

### 2. Position Control (Autonomous)
- Set turret to specific angles (0-360 degrees)
- Built-in position tolerance checking
- Encoder feedback for accurate positioning

### 3. SmartDashboard Integration
- Real-time telemetry (speed, current, temperature)
- Interactive controls (start/stop buttons)
- Status monitoring

### 4. Safety Features
- Emergency stop capability
- Speed clamping (prevents over-speed)
- Brake mode when stopped

## Configuration

### Constants (in `Constants.java`)

All turret configuration is centralized in `Constants.TurretConstants`:

```java
public static class TurretConstants {
    // Motor CAN ID
    public static final int kTurretMotorID = 6;
    public static final String kCANBusName = "rio";
    
    // Gear Ratio (adjust based on your turret gearing)
    public static final double kGearRatio = 1.0;
    
    // Speed Control
    public static final double kDefaultSpinSpeed = 0.3; // 30% power
    public static final double kMaxSpinSpeed = 1.0;
    public static final double kMinSpinSpeed = -1.0;
    
    // Position Control
    public static final double kPositionToleranceDegrees = 2.0;
}
```

### Adjusting Gear Ratio

If your turret has gearing, update `kGearRatio` in `Constants.java`:

```java
// Example: 10:1 reduction (10 motor rotations = 1 turret rotation)
public static final double kGearRatio = 10.0;
```

### Changing Default Speed

Modify `kDefaultSpinSpeed` for different test speeds:

```java
// Example: 50% power
public static final double kDefaultSpinSpeed = 0.5;
```

## Usage

### SmartDashboard Controls

The turret provides several SmartDashboard controls:

**Control Buttons:**
- `Turret/Start Spin` - Start spinning at configured speed
- `Turret/Stop Spin` - Stop the turret

**Configuration:**
- `Turret/Spin Speed` - Adjust speed (-1.0 to 1.0)

**Status Displays:**
- `Turret/Is Spinning` - Boolean indicator
- `Turret/Current Speed` - Current motor speed
- `Turret/Motor Current (A)` - Current draw in amps
- `Turret/Motor Temp (C)` - Motor temperature
- `Turret/Status` - Text status message

### Controller Bindings

**X Button (Xbox Controller):**
- Hold: Turret spins
- Release: Turret stops

Configured in `RobotContainer.java`:
```java
joystick.x().whileTrue(
    turretSubsystem.run(() -> turretSubsystem.startSpin())
).onFalse(
    turretSubsystem.runOnce(() -> turretSubsystem.stopSpin())
);
```

### Programming Interface

#### Speed Control Methods

```java
// Start spinning at dashboard-configured speed
turretSubsystem.startSpin();

// Stop spinning
turretSubsystem.stopSpin();

// Set specific speed (-1.0 to 1.0)
turretSubsystem.setSpeed(0.5); // 50% forward

// Get current speed
double speed = turretSubsystem.getSpeed();

// Check if spinning
boolean spinning = turretSubsystem.isSpinning();

// Emergency stop
turretSubsystem.emergencyStop();
```

#### Position Control Methods

```java
// Set turret to specific angle
turretSubsystem.setAngle(90.0); // 90 degrees

// Get current angle
double angle = turretSubsystem.getAngle();

// Check if at target angle (within default tolerance)
boolean atTarget = turretSubsystem.atAngle(90.0);

// Check with custom tolerance
boolean atTarget = turretSubsystem.atAngle(90.0, 5.0); // ±5 degrees

// Reset encoder to zero
turretSubsystem.resetEncoder();
```

## Autonomous Usage

### Using AimTurretAuto Command

The `AimTurretAuto` command spins the turret for 1 second during autonomous:

```java
// In autonomous routine
new AimTurretAuto(turretSubsystem)
```

### PathPlanner Integration

The turret command is registered as a named command for PathPlanner:

```java
NamedCommands.registerCommand("AimTurretAuto", 
    new AimTurretAuto(turretSubsystem));
```

Use it in PathPlanner autonomous routines by adding the `AimTurretAuto` command at any point in your path.

### Creating Custom Turret Commands

Example: Aim to specific angle and wait

```java
public class AimToAngle extends Command {
    private final TurretSubsystem turret;
    private final double targetAngle;
    
    public AimToAngle(TurretSubsystem turret, double angle) {
        this.turret = turret;
        this.targetAngle = angle;
        addRequirements(turret);
    }
    
    @Override
    public void initialize() {
        turret.setAngle(targetAngle);
    }
    
    @Override
    public boolean isFinished() {
        return turret.atAngle(targetAngle);
    }
}
```

## Troubleshooting

### Turret Not Responding

1. **Check CAN ID:** Verify motor is on CAN ID 6
2. **Check CAN Bus:** Ensure motor is on "rio" bus
3. **Check Phoenix Tuner:** Use Phoenix Tuner to verify motor connection
4. **Check Dashboard:** Look for initialization message in console

### Turret Spinning Wrong Direction

Reverse the speed in code:
```java
turretSubsystem.setSpeed(-0.5); // Negative for opposite direction
```

Or invert the motor in TurretSubsystem constructor:
```java
turretMotor.setInverted(true);
```

### Position Control Not Working

1. **Check Gear Ratio:** Verify `kGearRatio` is correct
2. **Reset Encoder:** Call `turretSubsystem.resetEncoder()` at known position
3. **Check Tolerance:** Increase `kPositionToleranceDegrees` if needed
4. **Verify Feedback:** Check encoder values in SmartDashboard

### Motor Overheating

1. **Reduce Speed:** Lower `kDefaultSpinSpeed`
2. **Check Mechanical:** Ensure no binding or excessive friction
3. **Monitor Current:** Watch `Turret/Motor Current (A)` on dashboard
4. **Add Cooling:** Consider adding cooling if sustained high-speed operation needed

## Safety Notes

⚠️ **Important Safety Considerations:**

1. **Brake Mode:** Turret uses brake mode by default - motor will resist movement when stopped
2. **Emergency Stop:** Use `emergencyStop()` method in emergency situations
3. **Speed Limits:** Speeds are automatically clamped to safe ranges
4. **Current Monitoring:** Monitor motor current to prevent damage
5. **Temperature Monitoring:** Watch motor temperature during extended operation

## Advanced Configuration

### Changing Neutral Mode

To change from Brake to Coast mode:

```java
// In TurretSubsystem constructor
turretMotor.setNeutralMode(NeutralModeValue.Coast);
```

### Adding Current Limits

Add current limiting for safety:

```java
// In TurretSubsystem constructor
var currentConfigs = new CurrentLimitsConfigs();
currentConfigs.SupplyCurrentLimit = 40.0; // 40A limit
currentConfigs.SupplyCurrentLimitEnable = true;
turretMotor.getConfigurator().apply(currentConfigs);
```

### Tuning Position Control

Adjust PID values for better position control:

```java
// In TurretSubsystem constructor
var slot0Configs = new Slot0Configs();
slot0Configs.kP = 24.0; // Increase for more aggressive positioning
slot0Configs.kI = 0.0;
slot0Configs.kD = 0.2;
turretMotor.getConfigurator().apply(slot0Configs);
```

## Migration from TestMotorSubsystem

If you're migrating from the old `TestMotorSubsystem`:

✅ **What Changed:**
- Class renamed: `TestMotorSubsystem` → `TurretSubsystem`
- File renamed: `TestMotorSubsystem.java` → `TurretSubsystem.java`
- Constants added: All configuration moved to `Constants.TurretConstants`
- Imports updated in `RobotContainer.java` and `AimTurretAuto.java`

✅ **What Stayed the Same:**
- All method signatures (API compatible)
- SmartDashboard key structure
- Controller bindings
- Autonomous command functionality

✅ **No Code Changes Required:**
- Existing commands still work
- PathPlanner integration unchanged
- Dashboard layouts compatible

## Support

For issues or questions:
1. Check console output for initialization messages
2. Verify all constants in `Constants.TurretConstants`
3. Use Phoenix Tuner for hardware diagnostics
4. Review SmartDashboard telemetry for real-time status
