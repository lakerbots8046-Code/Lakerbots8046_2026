# FeedFromCenter Hood Offset Implementation

## Plan Steps
- [x] User approved plan to add +2 hood offset constant for FeedFromCenter only
- [ ] Add `kFeedHoodOffsetRotations = 2.0` to Constants.java (FeedFromCenter class)
- [ ] Update `interpolateHoodPosition()` in FeedFromCenterCommand.java to add offset unconditionally
- [ ] Verify changes (read files, check compilation if possible)
- [ ] Test: Dashboard shows +2 rot across distances; no table changes
- [ ] Complete task

## Current Task Steps
- [ ] Update Launcher.turretGoHome() to be a finite wait-until-home command with timeout safety.
- [ ] Update ComplexCommands.playDefense() to run strict sequence: rollers off -> turret home complete -> intake stow.
- [ ] Run build validation (`gradlew build`) and confirm no compile errors.
