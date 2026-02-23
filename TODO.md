# LaunchFromTower Command - TODO

## Task: Add `launchFromTower` shoot command bound to A button

### Motor Assignments (all on RIO CAN bus)
| Motor         | CAN ID | Voltage (temp test) |
|---------------|--------|----------------------|
| Spindexer     | 4      | -0.2V                |
| StarFeeder    | 5      | +0.2V                |
| Feeder        | 6      | +0.2V                |
| Launcher      | 8      | +0.2V                |

### Steps

- [x] Read and understand Launcher.java, Spindexer.java, RobotContainer.java, Constants.java
- [x] Spindexer.java — Add `VoltageOut` control request + `launchFromTower()` command
- [x] Launcher.java — Add `VoltageOut` control request + `launchFromTowerLauncher()` command
- [x] RobotContainer.java — Replace A button binding with parallel launchFromTower command

### Notes
- Command runs while A is held, stops all motors on release (whileTrue)
- Uses proper CTRE Phoenix 6 `VoltageOut` control (not duty cycle)
- Temp safe test values used; update to final values after testing
