package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
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

    // ── State ─────────────────────────────────────────────────────────────────
    private boolean isFiring = false;

    // ── Dashboard throttle ────────────────────────────────────────────────────
    /** Counts execute() calls; telemetry is written every 5 calls (~100 ms). */
    private int dashboardCounter = 0;

    // ── Dashboard key prefix ──────────────────────────────────────────────────
    private static final String DASH = "FeedFromCenter/";

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
        SmartDashboard.putString( DASH + "Status", "Initializing");
        SmartDashboard.putBoolean(DASH + "Firing",  false);
    }

    @Override
    public void execute() {
        // ── 0. Gather context ─────────────────────────────────────────────────
        var           robotPose = drivetrain.getState().Pose;
        Translation2d robotPos  = robotPose.getTranslation();

        // ── 1. Determine alliance and select tag pairs ────────────────────────
        var allianceOpt = DriverStation.getAlliance();
        boolean isRed   = !allianceOpt.isPresent() || allianceOpt.get() == Alliance.Red;

        int[] pairA = isRed ? FeedFromCenter.kRedPairA  : FeedFromCenter.kBluePairA;
        int[] pairB = isRed ? FeedFromCenter.kRedPairB  : FeedFromCenter.kBluePairB;

        // ── 2. Compute tag-pair midpoints ─────────────────────────────────────
        Translation2d midpointA = getTagMidpoint(pairA[0], pairA[1]);
        Translation2d midpointB = getTagMidpoint(pairB[0], pairB[1]);

        if (midpointA == null && midpointB == null) {
            // No valid tag data — hold current state and report on dashboard
            SmartDashboard.putString(DASH + "Status", "No tag data — holding");
            return;
        }

        // ── 3. Offset each midpoint behind the wall ───────────────────────────
        // The adjusted target is kFeedTargetDepthMeters further from the robot
        // than the raw midpoint, placing the aim point inside the feed station.
        Translation2d targetA = (midpointA != null) ? adjustTarget(robotPos, midpointA) : null;
        Translation2d targetB = (midpointB != null) ? adjustTarget(robotPos, midpointB) : null;

        // ── 4. Pick the closer adjusted target ───────────────────────────────
        Translation2d target;
        String        pairLabel;
        if (targetA == null) {
            target    = targetB;
            pairLabel = isRed ? "Red B (4&6)" : "Blue B (20&22)";
        } else if (targetB == null) {
            target    = targetA;
            pairLabel = isRed ? "Red A (1&3)" : "Blue A (17&19)";
        } else {
            double distA = robotPos.getDistance(targetA);
            double distB = robotPos.getDistance(targetB);
            if (distA <= distB) {
                target    = targetA;
                pairLabel = isRed ? "Red A (1&3)" : "Blue A (17&19)";
            } else {
                target    = targetB;
                pairLabel = isRed ? "Red B (4&6)" : "Blue B (20&22)";
            }
        }

        // ── 5. Distance to target (for flywheel lookup) ───────────────────────
        double distance = robotPos.getDistance(target);

        // ── 6. Spin flywheel open-loop (DutyCycleOut) ─────────────────────────
        // Uses the same distance-based RPS lookup as ShootFromPointCommand.
        // dutyCycle = targetRPS / 100  (Kraken X60 free speed ≈ 100 RPS at 12V)
        double targetLauncherRPS = ShootingArcManager.calculateLauncherRPS(distance);
        launcher.setCollectDutyCycle(targetLauncherRPS);

        // Record the timestamp the first time the flywheel is commanded this cycle.
        if (flywheelStartTimestamp < 0) {
            flywheelStartTimestamp = Timer.getFPGATimestamp();
        }

        // ── 7. Set hood to fixed feed position (with deadband) ────────────────
        double hoodTarget = FeedFromCenter.kHoodPosition;
        if (Math.abs(hoodTarget - lastCommandedHoodRotations) > kHoodDeadbandRotations) {
            launcher.setHoodPosition(hoodTarget);
            lastCommandedHoodRotations = hoodTarget;
        }

        // ── 8. Aim turret at adjusted target ──────────────────────────────────
        // Uses calculateTurretAngleRaw() (no compiled offset) then applies the
        // compiled zero-offset constant. Same approach as ShootFromPointCommand.
        double rawTurretAngle     = ShootingArcManager.calculateTurretAngleRaw(robotPose, target);
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

        if (turretAtLimit) {
            // PAST LIMIT — hold at current position
            launcher.setTurretPosition(launcher.getTurretPosition());
        } else if (targetOutOfRange) {
            // OUT OF RANGE — drive to nearest limit
            launcher.setTurretPosition(clampedRaw);
        } else if (Math.abs(turretError) > kDeadbandDeg) {
            // NORMAL — update Motion Magic target
            launcher.setTurretPosition(targetRawRotations);
        }
        // else: within deadband — hold last commanded position (no profile reset)

        // ── 9. Evaluate fire conditions ───────────────────────────────────────
        boolean turretAimed = !turretAtLimit && !targetOutOfRange
                && Math.abs(turretError) < Constants.ShootingArc.kTurretAimToleranceDeg;

        double actualRPS    = Math.abs(launcher.getCollectVelocity());
        double targetRPS    = Math.abs(targetLauncherRPS);
        double timeSpooling = (flywheelStartTimestamp >= 0)
                ? Timer.getFPGATimestamp() - flywheelStartTimestamp
                : 0.0;
        // Primary trigger: time elapsed. Fallback: encoder velocity (if reporting).
        boolean launcherAtSpeed =
                (timeSpooling >= FLYWHEEL_SPOOL_TIME_SECONDS)
                || (actualRPS > 5.0 && Math.abs(actualRPS - targetRPS)
                        < Constants.ShootingArc.kLauncherVelocityTolerance);

        // ── 10. Schedule / cancel LaunchSequenceOneCommand ────────────────────
        if (launcherAtSpeed && !launchSequenceScheduled) {
            CommandScheduler.getInstance().schedule(launchSequenceOne);
            launchSequenceScheduled = true;
            isFiring = true;
        } else if (!launcherAtSpeed && launchSequenceScheduled) {
            launchSequenceOne.cancel();
            launchSequenceScheduled = false;
            isFiring = false;
        }

        // ── 11. Dashboard telemetry (throttled to every 5 loops ~100 ms) ──────
        // Kept minimal — only driver-relevant values to reduce NT load.
        dashboardCounter++;
        if (dashboardCounter >= 5) {
            dashboardCounter = 0;

            SmartDashboard.putString( DASH + "Active Pair",    pairLabel);
            SmartDashboard.putNumber( DASH + "Distance (m)",   distance);
            SmartDashboard.putNumber( DASH + "Turret Error (deg)", turretError);
            SmartDashboard.putBoolean(DASH + "Turret Aimed",   turretAimed);
            SmartDashboard.putBoolean(DASH + "At Speed",       launcherAtSpeed);
            SmartDashboard.putBoolean(DASH + "Firing",         isFiring);
            SmartDashboard.putString( DASH + "Status",
                    isFiring           ? "FIRING"
                    : !launcherAtSpeed ? String.format("Spooling (%.1f s)", timeSpooling)
                    : !turretAimed     ? "At Speed - Aiming"
                                       : "At Speed - Firing");
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
     * Offsets the midpoint by {@link FeedFromCenter#kFeedTargetDepthMeters} in the
     * direction from the robot toward the midpoint — placing the aiming target
     * "a few feet behind" the tag-pair midpoint (deeper into the feed station).
     *
     * @param robotPos Robot position on the field
     * @param midpoint Raw tag-pair midpoint
     * @return Adjusted target Translation2d
     */
    private Translation2d adjustTarget(Translation2d robotPos, Translation2d midpoint) {
        Translation2d toMidpoint = midpoint.minus(robotPos);
        double dist = toMidpoint.getNorm();
        if (dist < 0.01) return midpoint; // robot is at midpoint — no offset
        Translation2d unitDir = toMidpoint.div(dist);
        return midpoint.plus(unitDir.times(FeedFromCenter.kFeedTargetDepthMeters));
    }
}
