# Shoot-on-Move Directional Miss + Gain Effectiveness TODO

- [x] Analyze shoot-on-move lead compensation implementation and identify likely sign issue.
- [x] Build and confirm edit plan with user.
- [x] Update `ShootOnMoveCommand` lead compensation direction to oppose robot motion by default.
- [x] Keep dashboard invert behavior as an override with corrected default baseline.
- [x] Update `ShootOnMoveMathTest` to match corrected sign convention and add directional regression coverage.
- [x] Run tests (`gradlew test`) and verify changes.
- [x] Summarize changes and validation results.

- [x] Diagnose why changing LeadComp gains appears to have no effect.
- [ ] Update `ShootOnMoveCommand` so global `Lead Gain` multiplies axis gains.
- [ ] Add telemetry to expose lead saturation/clamping behavior.
- [ ] Update constants/comments to sane tuning defaults and explain clamp interaction.
- [ ] Re-run focused tests (`.\gradlew.bat test --tests frc.robot.ShootOnMoveMathTest`).
- [ ] Summarize gain-effectiveness fix and tuning guidance.
