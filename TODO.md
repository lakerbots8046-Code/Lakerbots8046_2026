
# Translation2d → Translation3d Conversion

## Steps

- [x] Plan confirmed
- [x] 1. `Constants.java` — add `kTurretOffsetZ = Units.inchesToMeters(14.0)` to `TurretConstants`
- [x] 2. `ShootingArcManager.java` — full Translation3d conversion:
  - [x] Add `Translation3d` import
  - [x] Change fallback constants to `Translation3d` (z = 1.12395 m from field layout)
  - [x] `getTowerCenter()` → return `Translation3d`, add `goalZ = tagPose.get().getZ()`
  - [x] `getNearestShootingPose()` → use `towerCenter.toTranslation2d()` for 2D ops
  - [x] `calculateArcVelocity()` → parameter `Translation3d`, use `.toTranslation2d()` internally
  - [x] `calculateTargetHeading()` → parameter `Translation3d`
  - [x] `getTurretFieldPosition()` → return `Translation3d`, add Z from `kTurretOffsetZ`
  - [x] `calculateTurretAngleRaw()` → parameter `Translation3d`
  - [x] `calculateTurretAngle()` → parameter `Translation3d`
  - [x] `calculateDistance()` → parameter `Translation3d` (true 3D distance)
- [x] 3. `ShootOnMoveCommand.java` — add `Translation3d` import, change `towerCenter` type
- [x] 4. `ShootFromPointCommand.java` — change import + `towerCenter` type
- [x] 5. `FeedFromCenterCommand.java` — add `Translation3d` import, wrap feed target in Translation3d for calculateTurretAngleRaw
- [x] 6. `RobotContainer.java` — no changes needed (uses `var`, auto-infers Translation3d)
