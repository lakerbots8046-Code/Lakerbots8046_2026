# Robot Code TODO List

## ✅ Completed Tasks

### Turret AprilTag Tracking Implementation
- [x] Added rotation limit constants (-175° to +175°)
- [x] Added AprilTag tracking PID constants
- [x] Added wrap-around behavior constants
- [x] Enhanced TurretSubsystem with limit checking methods
- [x] Enhanced TurretSubsystem with wrap-around detection
- [x] Created TrackAprilTagCommand for continuous tracking
- [x] Implemented PID control for smooth tracking
- [x] Implemented automatic wrap-around when limits are reached
- [x] Added Y button binding for AprilTag tracking
- [x] Created comprehensive documentation (TURRET_APRILTAG_TRACKING_GUIDE.md)

**Features Implemented:**
- Continuous AprilTag tracking with PID control
- Rotation limits to prevent wiring damage (-175° to +175°)
- Automatic 360° wrap-around when reaching limits
- "Locked in" status when aligned with target
- Dashboard integration for monitoring and debugging
- Support for both back cameras (BL and BR)

**Files Modified:**
1. `src/main/java/frc/robot/Constants.java` - Added turret tracking constants
2. `src/main/java/frc/robot/subsystems/TurretSubsystem.java` - Added limit checking and wrap-around logic
3. `src/main/java/frc/robot/RobotContainer.java` - Added Y button binding for tracking

**Files Created:**
1. `src/main/java/frc/robot/commands/TrackAprilTagCommand.java` - Main tracking command
2. `TURRET_APRILTAG_TRACKING_GUIDE.md` - Complete documentation

## 🔧 Pending Tasks

### Testing & Tuning
- [ ] Test rotation limits on actual hardware
- [ ] Tune PID constants for smooth tracking
- [ ] Test wrap-around behavior when limits are reached
- [ ] Verify tracking maintains lock during robot movement
- [ ] Test with different AprilTag distances
- [ ] Test with both cameras (BL and BR)

### Optional Enhancements
- [ ] Add predictive tracking (lead target based on velocity)
- [ ] Add multi-tag tracking capability
- [ ] Add field-relative tracking mode
- [ ] Add tracking profiles (aggressive vs smooth)
- [ ] Add auto-wrap based on robot heading

## 📝 Notes

### Controller Button Mapping
- **Y Button**: AprilTag tracking (hold to track, release to stop)
- **X Button**: Manual turret spin (for testing)
- **Left Trigger**: Intake control
- **Right Bumper**: Drive to AprilTag (PathPlanner)
- **Right Trigger**: Drive to AprilTag (Basic PID)
- **POV Buttons**: Quick access to specific tags

### Important Constants to Tune
1. **kTrackingP** (0.02) - Increase for faster response, decrease if oscillating
2. **kTrackingD** (0.001) - Increase to reduce overshoot
3. **kMaxTrackingSpeed** (0.5) - Maximum speed during tracking
4. **kTrackingToleranceDegrees** (3.0) - How close to be "locked in"

### Safety Reminders
- Always test rotation limits carefully before competition
- Monitor motor temperature during extended tracking
- Verify wiring can handle full rotation range
- Test emergency stop functionality
