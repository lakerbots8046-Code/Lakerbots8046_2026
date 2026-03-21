# TODO - ShootOnMoveCommand Update

- [x] Refactor `ShootOnMoveCommand` to match `ShootFromPointCommand` firing logic:
  - [x] turret + hood + launcher readiness gating (all three must be ready before firing)
  - [x] launcher spool timing aligned to 2.0s (matching ShootFromPointCommand)
  - [x] consistent turret control flow and tolerances
- [x] Add velocity-compensated trajectory/aiming in `ShootOnMoveCommand`:
  - [x] compute field-relative robot velocity from drivetrain chassis speeds
  - [x] compensate shot target/aim using robot velocity × estimated flight time
  - [x] use compensated aim for turret targeting
- [x] Remove arc-driving behavior from `ShootOnMoveCommand`:
  - [x] removed SwerveRequest.FieldCentric arc drive request
  - [x] removed drivetrain from addRequirements (read-only now)
  - [x] no arc translation/rotation commanded in execute
- [x] Update telemetry in `ShootOnMoveCommand` to reflect compensated aiming and unified readiness states
  - [x] added Robot Vx/Vy, Flight Time, Comp Dx/Dy, Hood At Target, Ready To Fire outputs
  - [x] dashboard namespace changed to `ShootOnMove/`
- [x] Bind driver right trigger (0.75) to `ShootOnMoveCommand` in `RobotContainer`
- [x] Instantiate `LEDSubsystem` in `RobotContainer` so its `periodic()` runs (LEDs were not animating)
- [x] Run build check — BUILD SUCCESSFUL
