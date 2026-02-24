# Shoot-On-Arc Feature TODO

## Overview
Implement a "shoot-on-the-move" system using AprilTag tower positions.
- Button press → robot drives to nearest arc position around tower
- Turret continuously aims using field-relative pose (not camera yaw)
- Launcher pre-spins to distance-based velocity during drive
- Left/right joystick slides robot along arc while shooting
- Fires automatically when turret is aimed AND launcher is at speed

## Steps

- [x] 1. Read and understand all relevant source files
- [x] 2. Add `ShootingArc` constants class to `Constants.java`
- [x] 3. Create `src/main/java/frc/robot/util/ShootingArcManager.java`
- [x] 4. Create `src/main/java/frc/robot/commands/ShootOnMoveCommand.java`
- [x] 5. Add direct motor control methods to `Spindexer.java`
- [x] 6. Update `RobotContainer.java` — Y button binding + alliance-aware tag selection
- [ ] 7. Test and tune on robot (lookup tables, PID gains, distances)

## Tuning Notes (post-deploy)
- `Constants.ShootingArc.kLauncherRPSLookup` — tune flywheel RPS per distance
- `Constants.ShootingArc.kHoodAngleLookup`   — tune hood rotations per distance
- `Constants.ShootingArc.kPreferredShootingDistance` — adjust arc radius
- `Constants.ShootingArc.kArcSlideSpeed`     — adjust left/right slide speed
- `Constants.ShootingArc.kArcRotationP`      — tune tower-facing rotation response
- `TurretConstants.kTrackingP/D`             — tune turret aim PID
