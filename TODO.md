# Vision Ambiguity Filter + 3D Processing Fix

## Task
Filter out AprilTag detections with ambiguity score > 0.3 from the pose estimator
to prevent the autonomous path planner from going wild on bad tag readings.
Also ensure 3D (PnP) processing is explicitly configured for single-tag fallback.

## Steps

- [x] Read and understand VisionSubsystem.java, RobotContainer.java, Constants.java
- [x] **Step 1** — `Constants.java`: Add `kMaxAmbiguity = 0.3` constant to `Vision` class
- [x] **Step 2** — `VisionSubsystem.java`:
    - Set `CLOSEST_TO_REFERENCE_POSE` as the multi-tag fallback strategy on both
      pose estimators (ensures best 3D single-tag pose is used when multi-tag fails)
    - Add `bestAmbiguityBF` / `bestAmbiguityFF` fields
    - Populate ambiguity fields from the selected target each loop
    - Publish `BF Best Ambiguity` / `FF Best Ambiguity` to SmartDashboard (throttled)
- [x] **Step 3** — `RobotContainer.java` → `updateVisionMeasurements()`:
    - For BF camera: reject measurement if ANY target in `targetsUsed` has
      `getPoseAmbiguity() >= 0 && > Constants.Vision.kMaxAmbiguity`
    - For FF camera: same check
    - Update SmartDashboard status to show `"Rejected (High Ambiguity)"` when filtered
- [x] **Step 4** — Verify build compiles cleanly — BUILD SUCCESSFUL (11 pre-existing deprecation warnings, 0 errors)
