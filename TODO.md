# Dashboard Decimal Rounding - TODO

## Goal
Round long-decimal outputs on the Elastic dashboard to 2-3 decimal places for cleaner display.

## Steps

- [x] Gather understanding of all files publishing numeric data
- [x] Create edit plan and get user approval
- [x] Edit `Telemetry.java` — round pose array [X, Y, Rotation]
- [x] Edit `VisionSubsystem.java` — round all putNumber calls (pose, yaw, pitch, area, distance, ambiguity)
- [x] Edit `CenterOnAprilTagCommand.java` — round distance, yaw error, speeds
- [x] Edit `DriveToAprilTag.java` — round X/Y errors, rotation error, distance, target pose
- [x] Edit `DriveToAprilTagWithPathPlanner.java` — round target pose, distance, rotation error
