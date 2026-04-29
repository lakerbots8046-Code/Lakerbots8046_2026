package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.FeedFromCenter;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Spindexer;
import frc.robot.util.ShootingArcManager;

/**
 * Feed-from-center command — aims the turret at the midpoint between two
 * feed-station AprilTags (offset a fixed depth behind the midpoint), sets the
 * hood to a fixed position, spins the flywheel, and fires via
 * {@link LaunchSequenceOneCommand} once the flywheel is at speed.
 *
 * <h3>Alliance-aware target selection</h3>
 * <ul>
 *   <li>Red  alliance: tags {1, 3} or {4, 6}  — whichever pair's midpoint is closer</li>
 *   <li>Blue alliance: tags {17, 19} or {20, 22} — whichever pair's midpoint is closer</li>
 * </ul>
 *
 * <h3>Target depth offset</h3>
 * The aiming target is placed {@link FeedFromCenter#kFeedTargetDepthMeters} (2 ft)
 * BEHIND the tag-pair midpoint in the direction from the robot toward the midpoint.
 * This ensures the game piece travels into the feed station rather than hitting the
 * front face of the wall.
 *
 * <h3>Subsystem requirements</h3>
 * {@code launcher} — turret (CAN 7), flywheel (CAN 8), hood (CAN 9).
 * {@code spindexer} is owned by {@link LaunchSequenceOneCommand} (scheduled separately).
 * {@code drivetrain} is read-only (pose only) — NOT a requirement, robot does not move.
 */
public class FeedFromCenterCommand extends Command {

    // ── Subsystems ────────────────────────────────────────────────────────────
    /** Read-only — used for pose only. NOT in addRequirements(). */
    private final CommandSwerveDrivetrain drivetrain;
    /** Controls turret (CAN 7), flywheel (CAN 8), and hood (CAN 9). */
    private final Launcher                launcher;
    /** Passed to LaunchSequenceOneCommand — NOT in addRequirements() here. */
    //private final Spindexer               spindexer;

    // ── LaunchSequenceOne (scheduled when flywheel is at speed) ───────────────
    private final LaunchSequenceOneCommand launchSequenceOne;
    /** Tracks whether launchSequenceOne is currently scheduled. */
    private boolean launchSequenceScheduled = false;

    // ── Flywheel spool-up timer ───────────────────────────────────────────────
    /**
     * FPGA timestamp (seconds) when the flywheel was first commanded this cycle.
     * -1 means the flywheel has not been commanded yet this cycle.
     */
    private double flywheelStartTimestamp = -1.0;

    /** Time (seconds) to wait after the flywheel starts before considering it "at speed". */
    private static final double FLYWHEEL_SPOOL_TIME_SECONDS = 1.5;

    // ── Hood deadband ─────────────────────────────────────────────────────────
    /**
     * Last hood position (motor rotations) sent to the motor.
     * Initialized to a sentinel value so the first call always updates.
     * Only updates when the new target differs by more than kHoodDeadbandRotations,
     * preventing constant Motion Magic profile restarts.
     */
    private double lastCommandedHoodRotations = Double.MAX_VALUE;

    /** Minimum change in hood target (rotations) required to re-send a setHoodPosition() command. */
    private static final double kHoodDeadbandRotations = 0.1;

    private boolean isFiring = false;

    // ── Turret wrap / swing safety interlock ─────────────────────────────────
    /** Previous measured turret angle in degrees (wrapped to [-180, 180]). */
    private Double lastTurretAngleDeg = null;
    /** FPGA timestamp until which feed is inhibited after unsafe swing/wrap event. */
    private double inhibitFeedUntilTimestamp = 0.0;
    /** Operator-selected holdoff after wrap/swing detection. */
    private static final double TURRET_WRAP_HOLDOFF_SECONDS = 0.1;
    /** If turret angular rate exceeds this, inhibit feed. */
    private static final double TURRET_RATE_INHIBIT_DEG_PER_SEC = 540.0;
    /** Near-boundary band to detect -180/180 wrap crossing events robustly. */
    private static final double TURRET_WRAP_BAND_DEG = 170.0;

    // ── Dashboard throttle ────────────────────────────────────────────────────
    /** Counts execute() calls; telemetry is written every 5 calls (~100 ms). */
    private int dashboardCounter = 0;

    // ── Dashboard key prefix ──────────────────────────────────────────────────
    private static final String DASH = "FeedFromCenter/";

    // ── Debug toggle key ───────────────────────────────────────────────────────
    private static final String kDisableLeadCompKey = DASH + "Disable Lead Compensation";

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * Creates a new FeedFromCenterCommand.
     *
     * @param drivetrain Swerve drivetrain (read-only — pose only, not required)
     * @param launcher   Launcher subsystem (turret + flywheel + hood)
     * @param spindexer  Spindexer subsystem (passed to LaunchSequenceOneCommand)
     */
    public FeedFromCenterCommand(
            CommandSwerveDrivetrain drivetrain,
            Launcher                launcher,
            Spindexer               spindexer) {

        this.drivetrain = drivetrain;
        this.launcher   = launcher;
        //this.spindexer  = spindexer;

        // LaunchSequenceOneCommand owns the spindexer requirement.
        // Scheduled/cancelled based on flywheel readiness — not added to this
        // command's requirements so both can coexist without conflict.
        this.launchSequenceOne = new LaunchSequenceOneCommand(
                spindexer,
                launcher::getCollectVelocity);

        // drivetrain is intentionally NOT listed — robot does not move.
        // spindexer is intentionally NOT listed — LaunchSequenceOneCommand owns it.
        addRequirements(launcher);
    }

    // =========================================================================
    // Command lifecycle
    // =========================================================================

    @Override
    public void initialize() {
        isFiring                = false;
        launchSequenceScheduled = false;
        dashboardCounter        = 0;
        flywheelStartTimestamp  = -1.0;
        lastCommandedHoodRotations = Double.MAX_VALUE; // force update on first execute()
        lastTurretAngleDeg = null;
        inhibitFeedUntilTimestamp = 0.0;

        SmartDashboard.putString( DASH + "Status", "Initializing");
        SmartDashboard.putBoolean(DASH + "Firing",  false);
        SmartDashboard.putBoolean(kDisableLeadCompKey, false);
    }

    @Override
    public void execute() {
        // ── 0. Gather context ─────────────────────────────────────────────────
        var           robotPose = drivetrain.getState().Pose;
        Translation2d robotPos  = robotPose.getTranslation();

        // ── 1. Determine alliance and select fixed field target by robot Y ───
        var allianceOpt = DriverStation.getAlliance();
        boolean isRed   = !allianceOpt.isPresent() || allianceOpt.get() == Alliance.Red;
        boolean isLowY  = robotPos.getY() < 4.0;

        // Requested mapping:
        // Red : Y<4 -> (15,2), Y>=4 -> (15,6)
        // Blue: Y<4 -> (2,1),  Y>=4 -> (6,2)
        Translation2d target;
        String pairLabel;
        if (isRed) {
            if (isLowY) {
                target = new Translation2d(16.0, 2.0); // (15,2)
                pairLabel = "Red Fixed (15,2)";
            } else {
                target = new Translation2d(16.0, 6.0); // (15,6)
                pairLabel = "Red Fixed (15,6)";
            }
        } else {
            if (isLowY) {
                target = new Translation2d(1.0, 2.0); // (2,2)
                pairLabel = "Blue Fixed (2,2)";
            } else {
                target = new Translation2d(1.0, 6.0); // (2,6)
                pairLabel = "Blue Fixed (2,6)";
            }
        }

        // Preserve debug fields for dashboard compatibility.
        Translation2d midpointA = null;
        Translation2d midpointB = null;
        Translation2d targetA   = null;
        Translation2d targetB   = null;

        // ── 5. Distance to target (for initial flywheel bootstrap) ────────────
        double distance = robotPos.getDistance(target);

        // ── 5b. Velocity lead compensation ────────────────────────────────────
        // When the robot is moving, the game piece is launched from a moving
        // platform and drifts in the direction of robot motion during flight.
        // We compute a "virtual target" — the point we must aim at so the ball
        // arrives at the actual target after accounting for robot velocity.
        //
        // Algorithm (one-iteration bootstrap):
        //   1. Estimate initial RPS from actual distance.
        //   2. Compute exit speed and flight time.
        //   3. virtualTarget = target − (robotVelocity × flightTime)
        //   4. Recompute RPS from virtual distance (one iteration is sufficient).
        //
        // Robot chassis speeds are robot-relative; rotate by heading to get
        // field-relative velocity components.
        ChassisSpeeds chassisSpeeds = drivetrain.getState().Speeds;
        double heading = robotPose.getRotation().getRadians();
        double fieldVx = chassisSpeeds.vxMetersPerSecond * Math.cos(heading)
                       - chassisSpeeds.vyMetersPerSecond * Math.sin(heading);
        double fieldVy = chassisSpeeds.vxMetersPerSecond * Math.sin(heading)
                       + chassisSpeeds.vyMetersPerSecond * Math.cos(heading);
        Translation2d robotVelocity = new Translation2d(fieldVx, fieldVy);

        // Velocity lead compensation is toggleable:
        // - ON  => compensate while moving (better shot consistency on the move)
        // - OFF => aim exact fixed ground coordinate (X,Y,Z=0)
        double flightTime = 0.0;
        Translation2d leadOffset = new Translation2d();
        Translation2d virtualTarget = target;
        double virtualDistance = distance;
        double robotSpeed = robotVelocity.getNorm(); // for dashboard

        boolean disableLeadComp = !FeedFromCenter.kUseMotionCompensation
                || SmartDashboard.getBoolean(kDisableLeadCompKey, false);

        if (!disableLeadComp) {
            final int kLeadIterations = 3;
            for (int i = 0; i < kLeadIterations; i++) {
                double iterRPS = ShootingArcManager.calculateLauncherRPS(virtualDistance);
                double iterExitSpeed = FeedFromCenter.kBallExitSpeedPerRPS * Math.abs(iterRPS);
                flightTime = (iterExitSpeed > 0.1) ? virtualDistance / iterExitSpeed : 0.0;

                Translation2d iterLeadOffset = robotVelocity
                        .times(flightTime * FeedFromCenter.kFeedFromCenterMotionCompensationGain);
                Translation2d iterVirtualTarget = target.minus(iterLeadOffset);
                virtualDistance = robotPos.getDistance(iterVirtualTarget);
            }

            leadOffset = robotVelocity
                    .times(flightTime * FeedFromCenter.kFeedFromCenterMotionCompensationGain);
            virtualTarget = target.minus(leadOffset);
            virtualDistance = robotPos.getDistance(virtualTarget);
        } else {
            leadOffset = new Translation2d();
            virtualTarget = target;
            virtualDistance = distance;
            flightTime = 0.0;
        }

        SmartDashboard.putBoolean(kDisableLeadCompKey, disableLeadComp);

        // ── 6. Spin flywheel using velocity closed-loop (VelocityVoltage) ─────
        // Uses virtual distance so flywheel compensates for robot moving
        // toward or away from the feed station during the shot.
        double baseLauncherRPS = ShootingArcManager.calculateLauncherRPS(virtualDistance)
                * FeedFromCenter.kFeedFromCenterRpsScale;

        // Radial velocity compensation:
        // +radialVelocity => moving toward target (can reduce energy slightly)
        // -radialVelocity => moving away/backward from target (needs more energy)
        Translation2d toTarget = target.minus(robotPos);
        double toTargetNorm = toTarget.getNorm();
        double radialVelocity = 0.0;
        if (toTargetNorm > 1e-6) {
            Translation2d toTargetUnit = toTarget.div(toTargetNorm);
            radialVelocity = robotVelocity.getX() * toTargetUnit.getX()
                           + robotVelocity.getY() * toTargetUnit.getY();
        }

        // Tuneable linear scale with safety clamps.
        final double kRadialCompGainPerMps = 0.30; // +30% RPS per 1 m/s retreat
        final double kMinRpsMotionScale = 0.90;    // at most -10% when strongly approaching
        final double kMaxRpsMotionScale = 2.00;    // at most +30% when strongly retreating
        double rpsMotionScale = 1.0 - (kRadialCompGainPerMps * radialVelocity);
        rpsMotionScale = Math.max(kMinRpsMotionScale, Math.min(kMaxRpsMotionScale, rpsMotionScale));

        double targetLauncherRPS = baseLauncherRPS * rpsMotionScale;
        double directionSign = Math.signum(targetLauncherRPS);
        if (Math.abs(directionSign) < 1e-6) {
            directionSign = -1.0; // default to shooting direction used by this robot
        }
        targetLauncherRPS += directionSign * FeedFromCenter.kFeedVelocityOffsetRps;
        if (targetLauncherRPS > -0.95){
            targetLauncherRPS = -0.95;
        }

        launcher.setCollectVelocity(targetLauncherRPS);

        // Record the timestamp the first time the flywheel is commanded this cycle.
        if (flywheelStartTimestamp < 0) {
            flywheelStartTimestamp = Timer.getFPGATimestamp();
        }

        // ── 7. Set hood to feed position based on distance (with deadband) ────
        // Use distance-interpolated hood target for smoother feed consistency across range.
        double hoodTarget = Constants.FeedFromCenter.kHoodFeedPosition;
        //double hoodTarget = interpolateHoodPosition(virtualDistance);
        if (Math.abs(hoodTarget - lastCommandedHoodRotations) > kHoodDeadbandRotations) {
            launcher.setHoodPosition(hoodTarget);
            lastCommandedHoodRotations = hoodTarget;
        }

        // ── 8. Aim turret at VIRTUAL target (velocity-compensated) ────────────
        // Uses the virtual target so the turret leads the feed station by the
        // amount the robot moves during ball flight. When the robot is stationary
        // the virtual target equals the actual target (no change in behavior).
        // calculateTurretAngleRaw takes Translation3d; Z=0.0 is safe because the
        // angle calculation only uses X/Y.
        double rawTurretAngle     = ShootingArcManager.calculateTurretAngleRaw(
                robotPose, new Translation3d(virtualTarget.getX(), virtualTarget.getY(), 0.0));
        double adjustedTurretAngle = rawTurretAngle - TurretConstants.kTurretZeroOffsetDegrees;
        while (adjustedTurretAngle >  180.0) adjustedTurretAngle -= 360.0;
        while (adjustedTurretAngle < -180.0) adjustedTurretAngle += 360.0;

        double currentTurretDeg = launcher.getTurretPositionDegrees();
        double turretError      = adjustedTurretAngle - currentTurretDeg;
        while (turretError >  180.0) turretError -= 360.0;
        while (turretError < -180.0) turretError += 360.0;

        double targetRawRotations = adjustedTurretAngle * TurretConstants.kRotationsPerDegree;

        // ── Limit enforcement (PHYSICAL HARD STOP: ±18 raw motor rotations) ───
        boolean turretAtLimit    = !launcher.isTurretWithinLimits();
        boolean targetOutOfRange = Math.abs(targetRawRotations) > TurretConstants.kPhysicalLimitRotations;
        double  clampedRaw       = Math.max(-TurretConstants.kPhysicalLimitRotations,
                                   Math.min( TurretConstants.kPhysicalLimitRotations, targetRawRotations));

        // 0.5° deadband prevents constant profile resets from tiny pose jitter.
        final double kDeadbandDeg = 0.5;

        final String turretLimitStatus;
        if (turretAtLimit) {
            // PAST LIMIT — hold at current position
            launcher.setTurretPosition(launcher.getTurretPosition());
            turretLimitStatus = "PAST LIMIT - STOPPED";
        } else if (targetOutOfRange) {
            // OUT OF RANGE — drive to nearest limit
            launcher.setTurretPosition(clampedRaw);
            turretLimitStatus = String.format("Out of range (%.0f deg)", adjustedTurretAngle);
        } else if (Math.abs(turretError) > kDeadbandDeg) {
            // NORMAL — update Motion Magic target
            launcher.setTurretPosition(targetRawRotations);
            turretLimitStatus = "OK";
        } else {
            turretLimitStatus = "OK (holding)";
        }
        // else: within deadband — hold last commanded position (no profile reset)

        // ── 9. Evaluate fire conditions ───────────────────────────────────────
        boolean turretAimed = !turretAtLimit && !targetOutOfRange
                && Math.abs(turretError) < Constants.FeedFromCenter.kTurretAimToleranceDeg;

        // ── Turret wrap/swing inhibit logic ───────────────────────────────────
        double now = Timer.getFPGATimestamp();
        boolean wrapCrossDetected = false;
        boolean highTurretRateDetected = false;
        double turretRateDegPerSec = 0.0;

        if (lastTurretAngleDeg != null) {
            double prev = lastTurretAngleDeg;
            double curr = currentTurretDeg;

            // Detect signed shortest angular delta in degrees (normalized to [-180, 180]).
            double delta = curr - prev;
            while (delta > 180.0) delta -= 360.0;
            while (delta < -180.0) delta += 360.0;

            // Approximate rate using nominal 20 ms loop period.
            turretRateDegPerSec = delta / 0.02;

            // Detect crossing near ±180 boundary (both samples near opposite ends).
            wrapCrossDetected =
                    (Math.abs(prev) > TURRET_WRAP_BAND_DEG)
                    && (Math.abs(curr) > TURRET_WRAP_BAND_DEG)
                    && (Math.signum(prev) != Math.signum(curr));

            highTurretRateDetected = Math.abs(turretRateDegPerSec) > TURRET_RATE_INHIBIT_DEG_PER_SEC;

            if (wrapCrossDetected || highTurretRateDetected) {
                inhibitFeedUntilTimestamp = Math.max(
                        inhibitFeedUntilTimestamp,
                        now + TURRET_WRAP_HOLDOFF_SECONDS);
            }
        }

        lastTurretAngleDeg = currentTurretDeg;
        boolean turretSwingInhibitActive = now < inhibitFeedUntilTimestamp;

        double actualRPS    = Math.abs(launcher.getCollectVelocity());
        double targetRPS    = Math.abs(targetLauncherRPS);
        double timeSpooling = (flywheelStartTimestamp >= 0)
                ? Timer.getFPGATimestamp() - flywheelStartTimestamp
                : 0.0;
        // Primary trigger: time elapsed. Fallback: encoder velocity (if reporting).
        boolean launcherAtSpeed =
                (timeSpooling >= FLYWHEEL_SPOOL_TIME_SECONDS)
                || (actualRPS > 5.0 && Math.abs(actualRPS - targetRPS)
                        < FeedFromCenter.kLauncherVelocityToleranceRps);

        boolean hoodAtTarget = launcher.isHoodAtTarget();
        boolean readyToFire = launcherAtSpeed && turretAimed && hoodAtTarget && !turretSwingInhibitActive;

        // ── 10. Trigger/cancel feed sequence directly (pre-state-machine behavior) ───
        if (readyToFire) {
            if (!launchSequenceScheduled) {
                CommandScheduler.getInstance().schedule(launchSequenceOne);
                launchSequenceScheduled = true;
            }
            isFiring = true;
        } else {
            if (launchSequenceScheduled) {
                launchSequenceOne.cancel();
                launchSequenceScheduled = false;
            }
            isFiring = false;
        }

        // ── 11. Dashboard telemetry (throttled to every 5 loops ~100 ms) ──────
        // Kept minimal — only driver-relevant values to reduce NT load.
        dashboardCounter++;
        if (dashboardCounter >= 5) {
            dashboardCounter = 0;

            SmartDashboard.putString( DASH + "Active Pair",         pairLabel);
            SmartDashboard.putNumber( DASH + "Distance (m)",        distance);
            SmartDashboard.putNumber( DASH + "Virtual Dist (m)",    virtualDistance);

            SmartDashboard.putBoolean(DASH + "Lead Disabled",       disableLeadComp);

            SmartDashboard.putBoolean(DASH + "MidpointA Valid",     midpointA != null);
            SmartDashboard.putBoolean(DASH + "MidpointB Valid",     midpointB != null);
            SmartDashboard.putBoolean(DASH + "TargetA Valid",       targetA != null);
            SmartDashboard.putBoolean(DASH + "TargetB Valid",       targetB != null);

            SmartDashboard.putNumber( DASH + "MidpointA X (m)",     midpointA != null ? midpointA.getX() : -1.0);
            SmartDashboard.putNumber( DASH + "MidpointA Y (m)",     midpointA != null ? midpointA.getY() : -1.0);
            SmartDashboard.putNumber( DASH + "MidpointB X (m)",     midpointB != null ? midpointB.getX() : -1.0);
            SmartDashboard.putNumber( DASH + "MidpointB Y (m)",     midpointB != null ? midpointB.getY() : -1.0);

            SmartDashboard.putNumber( DASH + "TargetA X (m)",       targetA != null ? targetA.getX() : -1.0);
            SmartDashboard.putNumber( DASH + "TargetA Y (m)",       targetA != null ? targetA.getY() : -1.0);
            SmartDashboard.putNumber( DASH + "TargetB X (m)",       targetB != null ? targetB.getX() : -1.0);
            SmartDashboard.putNumber( DASH + "TargetB Y (m)",       targetB != null ? targetB.getY() : -1.0);

            SmartDashboard.putNumber( DASH + "Selected Target X (m)", target.getX());
            SmartDashboard.putNumber( DASH + "Selected Target Y (m)", target.getY());
            SmartDashboard.putNumber( DASH + "Virtual Target X (m)",  virtualTarget.getX());
            SmartDashboard.putNumber( DASH + "Virtual Target Y (m)",  virtualTarget.getY());
            SmartDashboard.putNumber( DASH + "Robot Speed (m/s)",   robotSpeed);
            SmartDashboard.putNumber( DASH + "Flight Time (s)",     flightTime);
            SmartDashboard.putNumber( DASH + "Lead Offset (m)",     leadOffset.getNorm());
            SmartDashboard.putNumber( DASH + "Lead Gain",           FeedFromCenter.kFeedFromCenterMotionCompensationGain);
            SmartDashboard.putNumber( DASH + "RPS Scale",           FeedFromCenter.kFeedFromCenterRpsScale);
            SmartDashboard.putNumber( DASH + "Radial Velocity (m/s)", radialVelocity);
            SmartDashboard.putNumber( DASH + "RPS Motion Scale",      rpsMotionScale);
            SmartDashboard.putNumber( DASH + "Turret Error (deg)",  turretError);
            SmartDashboard.putBoolean(DASH + "Turret Aimed",        turretAimed);
            SmartDashboard.putString( DASH + "Turret Limit",        turretLimitStatus);
            SmartDashboard.putNumber( DASH + "Turret Rate (deg/s)", turretRateDegPerSec);
            SmartDashboard.putBoolean(DASH + "Wrap Cross Detected", wrapCrossDetected);
            SmartDashboard.putBoolean(DASH + "High Rate Detected",  highTurretRateDetected);
            SmartDashboard.putBoolean(DASH + "Swing Inhibit",       turretSwingInhibitActive);
            SmartDashboard.putNumber( DASH + "Inhibit Time Left (s)",
                    Math.max(0.0, inhibitFeedUntilTimestamp - now));
            SmartDashboard.putBoolean(DASH + "Hood At Target",      hoodAtTarget);
            SmartDashboard.putBoolean(DASH + "At Speed",            launcherAtSpeed);
            SmartDashboard.putBoolean(DASH + "Ready To Fire",       readyToFire);
            SmartDashboard.putBoolean(DASH + "Firing",              isFiring);
            SmartDashboard.putString( DASH + "Status",
                    isFiring           ? "FIRING"
                    : !launcherAtSpeed ? String.format("Spooling (%.1f s)", timeSpooling)
                    : (turretAtLimit || targetOutOfRange) ? "At Speed - Turret Wrap/Limit"
                    : turretSwingInhibitActive ? "At Speed - Turret Swing Inhibit"
                    : !hoodAtTarget    ? "At Speed - Adjusting Hood"
                                       : "At Speed - Ready");
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Cancel LaunchSequenceOneCommand if still running.
        // Its own end() will call spindexer.stopFeedMotorsDirect().
        if (launchSequenceScheduled) {
            launchSequenceOne.cancel();
            launchSequenceScheduled = false;
        }

        // Stop turret and flywheel. Hood stays at last position — default command retracts it.
        launcher.stopTurretDirect();
        launcher.stopLauncher();

        isFiring = false;
        SmartDashboard.putBoolean(DASH + "Firing",  false);
        SmartDashboard.putString( DASH + "Status", interrupted ? "Interrupted" : "Complete");
    }

    @Override
    public boolean isFinished() {
        // Runs until interrupted (right trigger released below 75%)
        return false;
    }

    // =========================================================================
    // Private helpers
    // =========================================================================

    /**
     * Returns the field-relative midpoint between two AprilTags.
     * Returns {@code null} if either tag is not found in the field layout.
     *
     * @param tagA First tag ID
     * @param tagB Second tag ID
     * @return Midpoint Translation2d, or null if tags are unavailable
     */
    private Translation2d getTagMidpoint(int tagA, int tagB) {
        try {
            var poseA = Constants.Vision.kTagLayout.getTagPose(tagA);
            var poseB = Constants.Vision.kTagLayout.getTagPose(tagB);
            if (poseA.isPresent() && poseB.isPresent()) {
                double x = (poseA.get().getX() + poseB.get().getX()) / 2.0;
                double y = (poseA.get().getY() + poseB.get().getY()) / 2.0;
                return new Translation2d(x, y);
            }
        } catch (Exception e) {
            System.err.println("FeedFromCenterCommand: Could not get midpoint for tags "
                    + tagA + " & " + tagB + ": " + e.getMessage());
        }
        return null;
    }

    /**
     * Offsets the midpoint by {@link FeedFromCenter#kFeedTargetDepthMeters} toward the
     * alliance feed zone interior using a fixed field-axis direction:
     * <ul>
     *   <li>Red alliance: +X from midpoint</li>
     *   <li>Blue alliance: -X from midpoint</li>
     * </ul>
     * This avoids direction flips that can occur when using robot-relative vectors.
     *
     * @param robotPos Robot position on the field (unused; kept for call-site compatibility)
     * @param midpoint Raw tag-pair midpoint
     * @return Adjusted target Translation2d
     */
    private Translation2d adjustTarget(Translation2d robotPos, Translation2d midpoint) {
        // Push target "behind" the wall midpoint toward the alliance feed zone, not based on robot side.
        // Using robot->midpoint can flip direction when approaching from different field positions.
        // Field layout here is symmetric around Y=4.021 m:
        //   Red feed station is at larger X than the midpoint -> push +X
        //   Blue feed station is at smaller X than the midpoint -> push -X
        // This keeps feed-from-center aiming consistent and prevents drifting toward field-center/hub side.
        var allianceOpt = DriverStation.getAlliance();
        boolean isRed = !allianceOpt.isPresent() || allianceOpt.get() == Alliance.Red;

        double xOffset = isRed
                ? FeedFromCenter.kFeedTargetDepthMeters
                : -FeedFromCenter.kFeedTargetDepthMeters;

        return midpoint.plus(new Translation2d(xOffset, 0.0));
    }

    /**
     * Returns hood position in motor rotations using distance-based interpolation
     * from ShootingArc hood lookup, with FeedFromCenter near/far fallback bounds.
     */
    private double interpolateHoodPosition(double distanceMeters) {
        double[][] table = Constants.ShootingArc.kHoodAngleLookup;
        double hood;

        if (table == null || table.length == 0) {
            hood = (distanceMeters < FeedFromCenter.kHoodDistanceThresholdMeters)
                    ? FeedFromCenter.kHoodFeedPosition
                    : FeedFromCenter.kHoodPositionFar;
            if (distanceMeters >= FeedFromCenter.kFarHoodDistanceMeters) {
                hood += FeedFromCenter.kFarHoodExtraRotations;
            }
            return hood + FeedFromCenter.kFeedHoodOffsetRotations;
        }

        if (distanceMeters <= table[0][0]) {
            hood = table[0][1];
        } else if (distanceMeters >= table[table.length - 1][0]) {
            hood = table[table.length - 1][1];
            if (distanceMeters >= FeedFromCenter.kFarHoodDistanceMeters) {
                hood += FeedFromCenter.kFarHoodExtraRotations;
            }
        } else {
            hood = (distanceMeters < FeedFromCenter.kHoodDistanceThresholdMeters)
                    ? FeedFromCenter.kHoodFeedPosition
                    : FeedFromCenter.kHoodPositionFar;

            for (int i = 0; i < table.length - 1; i++) {
                double x0 = table[i][0];
                double y0 = table[i][1];
                double x1 = table[i + 1][0];
                double y1 = table[i + 1][1];
                if (distanceMeters >= x0 && distanceMeters <= x1) {
                    double t = (distanceMeters - x0) / (x1 - x0);
                    hood = y0 + (y1 - y0) * t;
                    if (distanceMeters >= FeedFromCenter.kFarHoodDistanceMeters) {
                        hood += FeedFromCenter.kFarHoodExtraRotations;
                    }
                    break;
                }
            }
        }

        return hood + FeedFromCenter.kFeedHoodOffsetRotations;
    }
}
