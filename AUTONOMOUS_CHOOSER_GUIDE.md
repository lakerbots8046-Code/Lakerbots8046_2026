# Autonomous Chooser Guide

## Overview
An autonomous chooser has been added to the robot code, allowing you to select different autonomous routines from the dashboard (SmartDashboard, Shuffleboard, or Elastic Dashboard).

## Available Autonomous Options

### 1. **Drive Forward** (Default)
- **Description**: Simple autonomous routine that drives the robot forward
- **Behavior**: 
  - Resets field-centric heading to 0°
  - Drives forward at 0.5 m/s for 5 seconds
  - Then idles for the remainder of autonomous period
- **Use Case**: Basic testing, default safe option

### 2. **New Auto (PathPlanner)**
- **Description**: PathPlanner-based autonomous routine
- **Behavior**:
  - Executes the "New Auto" PathPlanner autonomous file
  - Includes: AimTurretAuto command (spins turret for 1 second) + Left Side Part 1 path
  - Resets odometry to the starting position defined in PathPlanner
- **Use Case**: Competition autonomous with complex paths

### 3. **Do Nothing**
- **Description**: No autonomous action
- **Behavior**: Robot remains idle during autonomous period
- **Use Case**: Testing, troubleshooting, or when you don't want any autonomous movement

## How to Use

### On SmartDashboard/Shuffleboard:
1. Deploy your robot code
2. Open SmartDashboard or Shuffleboard
3. Look for the widget labeled **"Auto Chooser"**
4. Use the dropdown menu to select your desired autonomous routine
5. The selected routine will run when you enable autonomous mode

### On Elastic Dashboard:
1. Deploy your robot code
2. Open Elastic Dashboard
3. Navigate to the NetworkTables view
4. Find **"SmartDashboard/Auto Chooser"**
5. Select your desired autonomous routine from the available options
6. The selected routine will run when you enable autonomous mode

## Implementation Details

### Code Location
- **File**: `src/main/java/frc/robot/RobotContainer.java`
- **Chooser Field**: `private final SendableChooser<Command> autoChooser;`
- **Initialization**: `buildAutoChooser()` method in constructor
- **Publishing**: Published to SmartDashboard as "Auto Chooser"

### Key Methods
- `buildAutoChooser()`: Creates and populates the chooser with autonomous options
- `getDriveForwardAuto()`: Returns the simple drive forward command
- `getAutonomousCommand()`: Returns the currently selected autonomous command

### PathPlanner Integration
The chooser uses PathPlanner's `AutoBuilder.buildAuto()` to load autonomous routines:
```java
Command pathPlannerAuto = AutoBuilder.buildAuto("New Auto");
```

If the PathPlanner auto fails to load, a fallback option is added with a "FAILED TO LOAD" suffix.

## Adding More Autonomous Routines

### To add a new simple autonomous:
1. Create a new method in `RobotContainer.java` (similar to `getDriveForwardAuto()`)
2. Add it to the chooser in `buildAutoChooser()`:
```java
chooser.addOption("My New Auto", getMyNewAuto());
```

### To add a new PathPlanner autonomous:
1. Create the autonomous routine in PathPlanner GUI
2. Save it in `src/main/deploy/pathplanner/autos/`
3. Add it to the chooser in `buildAutoChooser()`:
```java
try {
    Command myAuto = AutoBuilder.buildAuto("My Auto Name");
    chooser.addOption("My Auto Name", myAuto);
} catch (Exception e) {
    System.err.println("Failed to load PathPlanner auto: " + e.getMessage());
}
```

## Troubleshooting

### Chooser not appearing on dashboard:
- Verify robot code is deployed successfully
- Check that SmartDashboard/Shuffleboard is connected to the robot
- Look for "Auto Chooser" in the SmartDashboard tab

### PathPlanner auto shows "FAILED TO LOAD":
- Check that the `.auto` file exists in `src/main/deploy/pathplanner/autos/`
- Verify the auto name matches exactly (case-sensitive)
- Check console output for error messages
- Ensure all named commands used in the auto are registered

### Selected auto doesn't run:
- Verify you've selected an option before enabling autonomous
- Check Driver Station for any error messages
- Ensure the robot is in autonomous mode (not teleop or test)

## Testing Recommendations

1. **Always test in a safe environment first**
2. Start with "Do Nothing" to verify the chooser works
3. Test "Drive Forward" in an open area
4. Test PathPlanner autos with plenty of space
5. Verify the correct auto is selected before each match

## Notes

- The default autonomous is "Drive Forward" - this will run if no selection is made
- The chooser selection persists between robot reboots (stored in NetworkTables)
- You can change the selection at any time, but it only takes effect when autonomous is initialized
- Named commands (like "AimTurretAuto") must be registered before PathPlanner autos can use them
