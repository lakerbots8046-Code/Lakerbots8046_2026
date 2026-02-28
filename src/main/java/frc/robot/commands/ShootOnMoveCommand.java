package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Spindexer;
import frc.robot.util.ShootingArcManager;

/**
 * Shoot-on-move command — Phase 2 of the shooting arc sequence.
 *
 * <p>Assumes the robot is already near the shooting arc (Phase 1 drove it there
 * via PathPlanner). This command then:
 * <ol>
 *   <li>Continuously aims the turret (CAN ID 7, in {@link Launcher}) at the tower
 *       using <em>field-relative</em> robot pose via Motion Magic position control.</li>
 *   <li>Sets launcher flywheel velocity and hood angle based on current distance
 *       to the tower (interpolated from lookup tables).</li>
 *   <li>Fires (runs spindexer + feeder) automatically when the turret is within
 *       aim tolerance AND the launcher is at target speed.</li>
 *   <li>Allows the driver to slide left/right along the arc by pushing the left
 *       joystick X axis, while the robot heading is locked to face the tower.</li>
 * </ol>
 *
 * <p>This command runs until interrupted (Y button released).
 *
 * <h3>Turret motor</h3>
 * CAN ID 7 (in {@code Launcher} subsystem). Uses Motion Magic position control —
 * no external PID needed; the CTRE closed-loop handles acceleration and settling.
 *
 * <h3>Subsystem requirements</h3>
 * {@code drivetrain}, {@code launcher}, {@code spindexer}
 */
public class ShootOnMoveCommand extends Command {

    // ── Subsystems ────────────────────────────────────────────────────────────
    private final CommandSwerveDrivetrain drivetrain;
    /** Controls turret (CAN 7), flywheel (CAN 8), and hood (CAN 9). */
    private final Launcher                launcher;
    //private final Spindexer               spindexer;

    // ── Suppliers ─────────────────────────────────────────────────────────────
    /** Supplies the active tower tag ID (alliance-aware, vision-preferred). */
    private final IntSupplier    towerTagIdSupplier;
    /** Supplies left/right joystick input in [-1, 1] for arc sliding. */
    private final DoubleSupplier lateralInputSupplier;

    // ── Swerve request for arc sliding ────────────────────────────────────────
    private final SwerveRequest.FieldCentric arcDriveRequest;

    // ── LaunchSequenceOne (scheduled when flywheel is at speed) ───────────────
    private final LaunchSequenceOneCommand launchSequenceOne;
    /** Tracks whether launchSequenceOne is currently scheduled. */
    private boolean launchSequenceScheduled = false;

    // ── Flywheel spool-up timer ───────────────────────────────────────────────
    /**
     * FPGA timestamp (seconds) when the flywheel was first commanded this cycle.
     * -1 means the flywheel has not been commanded yet this cycle.
     *
     * <p>Because the launcher motor encoder does not reliably report velocity
     * (getCollectVelocity() returns ~0 even when the motor is spinning), we use
     * a fixed spool-up delay instead of a velocity threshold to determine when
     * the flywheel is ready to fire.
     */
    private double flywheelStartTimestamp = -1.0;

    /**
     * Time (seconds) to wait after the flywheel starts before scheduling
     * LaunchSequenceOneCommand. Tune this to match the physical spool-up time.
     */
    private static final double FLYWHEEL_SPOOL_TIME_SECONDS = 1.5;

    // ── Dashboard throttle ────────────────────────────────────────────────────
    /** Counts execute() calls; telemetry is written every 5 calls (~100ms). */
    private int dashboardCounter = 0;

    // ── Hood deadband ─────────────────────────────────────────────────────────
    /**
     * Last hood position (motor rotations) sent to the motor.
     * Initialized to a sentinel value so the first call always updates.
     * Only updates when the new target differs by more than kHoodDeadbandRotations,
     * preventing constant Motion Magic profile restarts from tiny pose jitter.
     */
    private double lastCommandedHoodRotations = Double.MAX_VALUE;

    /**
     * Minimum change in hood target (motor rotations) required to re-send a
     * setHoodPosition() command. Prevents profile restarts from sub-millimeter
     * pose jitter that would otherwise cause the hood to oscillate.
     * 0.1 rot ≈ 0.35° of hood angle — well below the 0.2-rotation tolerance.
     */
    private static final double kHoodDeadbandRotations = 0.1;

    // ── Tag locking ───────────────────────────────────────────────────────────
    /** Tower tag locked in on initialize(). Prevents oscillation when multiple tags visible. */
    private int lockedTagId = -1;
    /** Candidate replacement tag being evaluated for stability before switching. */
    private int pendingTagId = -1;
    /** Consecutive loops the supplier has returned pendingTagId instead of lockedTagId. */
    private int pendingTagCounter = 0;
    /** Loops a different tag must be consistently returned before switching (~500 ms). */
    private static final int TAG_SWITCH_STABILITY_LOOPS = 25;

    // ── State ─────────────────────────────────────────────────────────────────
    private boolean isFiring = false;

    // ── Dashboard key prefix ──────────────────────────────────────────────────
    private static final String DASH = "ShootArc/";

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * Creates a new ShootOnMoveCommand.
     *
     * @param drivetrain           Swerve drivetrain subsystem
     * @param launcher             Launcher subsystem (turret CAN 7 + flywheel + hood)
     * @param spindexer            Spindexer subsystem (feed motors)
     * @param towerTagIdSupplier   Supplies the active tower AprilTag ID at runtime
     * @param lateralInputSupplier Supplies left/right joystick axis value [-1, 1]
     */
    public ShootOnMoveCommand(
            CommandSwerveDrivetrain drivetrain,
            Launcher                launcher,
            Spindexer               spindexer,
            IntSupplier             towerTagIdSupplier,
            DoubleSupplier          lateralInputSupplier) {

        this.drivetrain           = drivetrain;
        this.launcher             = launcher;
        //this.spindexer            = spindexer;
        this.towerTagIdSupplier   = towerTagIdSupplier;
        this.lateralInputSupplier = lateralInputSupplier;

        // Field-centric swerve request (no deadband — we compute exact velocities)
        this.arcDriveRequest = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // LaunchSequenceOneCommand owns the spindexer requirement.
        // It is scheduled/cancelled here based on flywheel speed — not added to
        // this command's requirements so both can coexist without conflict.
        this.launchSequenceOne = new LaunchSequenceOneCommand(
                spindexer,
                launcher::getCollectVelocity);

        // spindexer is intentionally NOT listed here — LaunchSequenceOneCommand owns it.
        addRequirements(drivetrain, launcher);
    }

    // =========================================================================
    // Command lifecycle
    // =========================================================================

    @Override
    public void initialize() {
        isFiring                = false;
        launchSequenceScheduled = false;
        dashboardCounter        = 0;
        flywheelStartTimestamp       = -1.0;
        lastCommandedHoodRotations   = Double.MAX_VALUE; // force update on first execute()
        // Lock the tower tag on command start — prevents oscillation when multiple
        // tower tags (e.g. 5 and 10) are simultaneously visible.
        lockedTagId       = towerTagIdSupplier.getAsInt();
        pendingTagId      = -1;
        pendingTagCounter = 0;

        // Publish locked tag immediately so Elastic shows it before the first throttled block.
        SmartDashboard.putNumber(DASH + "Locked Tag ID", lockedTagId);

        SmartDashboard.putString(DASH + "Status",  "Shooting on Arc");
        SmartDashboard.putBoolean(DASH + "Firing", false);
    }

    @Override
    public void execute() {
        // ── 0. Gather context — tag locking with stability ────────────────────
        int currentBestTag = towerTagIdSupplier.getAsInt();
        if (currentBestTag == lockedTagId) {
            pendingTagId      = -1;
            pendingTagCounter = 0;
        } else {
            if (currentBestTag == pendingTagId) {
                pendingTagCounter++;
                if (pendingTagCounter >= TAG_SWITCH_STABILITY_LOOPS) {
                    lockedTagId       = pendingTagId;
                    pendingTagId      = -1;
                    pendingTagCounter = 0;
                }
            } else {
                pendingTagId      = currentBestTag;
                pendingTagCounter = 1;
            }
        }
        int           towerTagId  = lockedTagId;
        var           robotPose   = drivetrain.getState().Pose;
        Translation3d towerCenter = ShootingArcManager.getTowerCenter(robotPose, towerTagId);

        // ── 1. Distance-based shooting parameters ─────────────────────────────
        double distance            = ShootingArcManager.calculateDistance(robotPose, towerCenter);
        double targetTurretAngle   = ShootingArcManager.calculateTurretAngle(robotPose, towerCenter);
        double targetLauncherRPS   = ShootingArcManager.calculateLauncherRPS(distance);
        double targetHoodRotations = ShootingArcManager.calculateHoodAngle(distance);

        // ── 2. Spin flywheel closed-loop (VelocityVoltage) ────────────────────
        // Closed-loop control lets the PID actively recover speed when a ball
        // loads the flywheel, rather than just holding a fixed duty cycle.
        // kV (0.12) handles steady-state; kP (0.3) handles transient speed drops.
        launcher.setCollectVelocity(targetLauncherRPS);

        // ── 2b. Set hood angle via Motion Magic (CAN 9) ───────────────────────
        // Only re-send the command when the target changes by more than 0.1 rot.
        // Calling setHoodPosition() every loop restarts the Motion Magic profile
        // every 20 ms, which causes the hood to oscillate even when stationary.
        if (Math.abs(targetHoodRotations - lastCommandedHoodRotations) > kHoodDeadbandRotations) {
            launcher.setHoodPosition(targetHoodRotations);
            lastCommandedHoodRotations = targetHoodRotations;
        }

        // Record the timestamp the first time the flywheel is commanded this cycle.
        if (flywheelStartTimestamp < 0) {
            flywheelStartTimestamp = Timer.getFPGATimestamp();
        }

        // ── 3. Aim turret (CAN ID 7) using Motion Magic position control ──────
        //
        // Convert target angle (degrees) to raw motor rotations.
        // SensorToMechanismRatio is not configured on this motor, so
        // getTurretPosition() / setTurretPosition() work in raw motor rotations.
        //
        double currentTurretDeg   = launcher.getTurretPositionDegrees();
        double turretError         = targetTurretAngle - currentTurretDeg;
        // Normalize error to [-180, 180]
        while (turretError >  180.0) turretError -= 360.0;
        while (turretError < -180.0) turretError += 360.0;

        // Convert target angle to raw motor rotations for Motion Magic
        double targetRawRotations = targetTurretAngle * TurretConstants.kRotationsPerDegree;

        // ── Limit enforcement (PHYSICAL HARD STOP: ±18 raw motor rotations) ───
        //
        // Three cases:
        //   1. PAST LIMIT  — turret has gone beyond ±18 rotations. Hold in place.
        //   2. OUT OF RANGE — target requires more than ±18 rotations. Drive to
        //                     nearest limit; robot heading control rotates the robot
        //                     to bring the target back into range.
        //   3. NORMAL       — target within ±18 rotations. Motion Magic aims normally.
        //
        boolean turretAtLimit    = !launcher.isTurretWithinLimits();
        boolean targetOutOfRange = Math.abs(targetRawRotations) > TurretConstants.kPhysicalLimitRotations;
        double  clampedRaw       = Math.max(-TurretConstants.kPhysicalLimitRotations,
                                   Math.min( TurretConstants.kPhysicalLimitRotations, targetRawRotations));

        // Deadband: only update the Motion Magic target if the error is large enough.
        // Constantly resetting the profile with tiny changes causes oscillation.
        // 0.5° deadband = 0.5 * kRotationsPerDegree ≈ 0.053 raw rotations.
        final double kDeadbandDeg = 0.5;

        // Capture turret limit status as a string for the throttled dashboard block below.
        // Previously these were written every loop (50 puts/sec) — now written at 10 Hz.
        final String turretLimitStatus;
        if (turretAtLimit) {
            // Case 1: PAST LIMIT — hold at current position to prevent further damage
            launcher.setTurretPosition(launcher.getTurretPosition());
            turretLimitStatus = "PAST LIMIT - STOPPED";

        } else if (targetOutOfRange) {
            // Case 2: OUT OF RANGE — drive to nearest limit and wait for robot to rotate
            launcher.setTurretPosition(clampedRaw);
            turretLimitStatus = String.format("Out of range (%.0f deg)", targetTurretAngle);

        } else if (Math.abs(turretError) > kDeadbandDeg) {
            // Case 3: NORMAL — error exceeds deadband, update Motion Magic target
            launcher.setTurretPosition(targetRawRotations);
            turretLimitStatus = "OK";

        } else {
            // Case 4: WITHIN DEADBAND — hold last commanded position (no profile reset)
            turretLimitStatus = "OK (holding)";
        }

        // ── 4. Evaluate fire conditions ───────────────────────────────────────
        // turretAimed is kept for telemetry only — it does not gate the feed motors.
        boolean turretAimed = !turretAtLimit && !targetOutOfRange
                && Math.abs(turretError) < Constants.ShootingArc.kTurretAimToleranceDeg;

        // Time-based spool-up check.
        // The launcher motor encoder does not reliably report velocity
        // (getCollectVelocity() returns ~0 even when the motor is physically spinning).
        // We therefore use a fixed spool-up delay (FLYWHEEL_SPOOL_TIME_SECONDS = 1.5 s)
        // as the "at speed" trigger instead of a velocity threshold.
        double actualRPS = Math.abs(launcher.getCollectVelocity());
        double targetRPS = Math.abs(targetLauncherRPS);
        double timeSpooling = (flywheelStartTimestamp >= 0)
                ? Timer.getFPGATimestamp() - flywheelStartTimestamp
                : 0.0;
        // Primary trigger: time elapsed since flywheel started.
        // Fallback: if the encoder IS reporting a valid velocity, also accept that.
        boolean launcherAtSpeed =
                (timeSpooling >= FLYWHEEL_SPOOL_TIME_SECONDS)
                || (actualRPS > 5.0 && Math.abs(actualRPS - targetRPS)
                        < Constants.ShootingArc.kLauncherVelocityTolerance);

        // ── 5. Schedule / cancel LaunchSequenceOneCommand ─────────────────────
        //
        // LaunchSequenceOneCommand is triggered as soon as the flywheel reaches its
        // desired RPS. It proportionally scales the spindexer/feeder duty cycles
        // based on the live flywheel velocity, which is more accurate than fixed
        // duty cycle values.
        //
        // The command is scheduled via CommandScheduler so it runs as a proper
        // WPILib command (with its own initialize/execute/end lifecycle) without
        // being a subsystem requirement of this command.
        if (launcherAtSpeed && !launchSequenceScheduled) {
            CommandScheduler.getInstance().schedule(launchSequenceOne);
            launchSequenceScheduled = true;
            isFiring = true;
        } else if (!launcherAtSpeed && launchSequenceScheduled) {
            launchSequenceOne.cancel();
            launchSequenceScheduled = false;
            isFiring = false;
        }

        // ── 6. Arc sliding — left/right joystick ─────────────────────────────
        double        lateralInput = lateralInputSupplier.getAsDouble();
        Translation2d arcVelocity  = ShootingArcManager.calculateArcVelocity(
                robotPose, towerCenter, lateralInput); // returns Translation2d (2D velocity vector)
        // ── 7. Rotation — lock robot heading to face tower ────────────────────
        double targetHeading  = ShootingArcManager.calculateTargetHeading(robotPose, towerCenter);
        double currentHeading = robotPose.getRotation().getRadians();
        double headingError   = targetHeading - currentHeading;
        // Normalize heading error to [-pi, pi]
        while (headingError >  Math.PI) headingError -= 2.0 * Math.PI;
        while (headingError < -Math.PI) headingError += 2.0 * Math.PI;

        double rotationOutput = Constants.ShootingArc.kArcRotationP * headingError;
        rotationOutput = Math.max(-Constants.ShootingArc.kMaxArcRotationRate,
                                   Math.min( Constants.ShootingArc.kMaxArcRotationRate, rotationOutput));

        // ── 8. Apply swerve drive request directly ────────────────────────────
        // setControl() is inherited from SwerveDrivetrain — safe to call in execute().
        drivetrain.setControl(arcDriveRequest
                .withVelocityX(arcVelocity.getX())
                .withVelocityY(arcVelocity.getY())
                .withRotationalRate(rotationOutput));

        // ── 9. Dashboard telemetry (throttled to every 5 loops ~100ms) ────────
        dashboardCounter++;
        if (dashboardCounter >= 5) {
            dashboardCounter = 0;

            // ── Tag identification outputs ─────────────────────────────────────
            // Determine alliance and whether this is the primary (center) tag.
            String  tagAlliance = ShootingArcManager.isRedTowerTag(towerTagId)  ? "Red"
                                : ShootingArcManager.isBlueTowerTag(towerTagId) ? "Blue"
                                : "Unknown";
            boolean isPrimary   = (towerTagId == Constants.ShootingArc.kRedPrimaryTagId)
                                || (towerTagId == Constants.ShootingArc.kBluePrimaryTagId);
            // e.g. "Tag 10 (Red Tower ★)" or "Tag 9 (Red Tower)"
            String  tagInfo     = String.format("Tag %d (%s Tower%s)",
                                    towerTagId, tagAlliance, isPrimary ? " \u2605" : "");
            boolean inZone      = ShootingArcManager.isInShootingZone(robotPose, towerTagId);

            SmartDashboard.putNumber( DASH + "Locked Tag ID",       lockedTagId);
            SmartDashboard.putNumber( DASH + "Tower Tag ID",        towerTagId);
            SmartDashboard.putString( DASH + "Tower Tag Info",      tagInfo);
            SmartDashboard.putString( DASH + "Tag Alliance",        tagAlliance);
            SmartDashboard.putBoolean(DASH + "In Shooting Zone",    inZone);
            SmartDashboard.putNumber( DASH + "Distance (m)",        distance);

            // ── Mechanism outputs ──────────────────────────────────────────────
            SmartDashboard.putNumber( DASH + "Turret Error (deg)",  turretError);
            SmartDashboard.putBoolean(DASH + "Turret Aimed",        turretAimed);
            SmartDashboard.putString( DASH + "Turret Limit",        turretLimitStatus);
            SmartDashboard.putNumber( DASH + "Hood Target (rot)",   targetHoodRotations);
            SmartDashboard.putBoolean(DASH + "Hood At Target",      launcher.isHoodAtTarget());
            SmartDashboard.putNumber( DASH + "Launcher RPS Target", targetLauncherRPS);
            SmartDashboard.putNumber( DASH + "Spool Time (s)",      timeSpooling);
            SmartDashboard.putBoolean(DASH + "Launcher At Speed",   launcherAtSpeed);
            SmartDashboard.putBoolean(DASH + "Firing",              isFiring);
            SmartDashboard.putString( DASH + "Status",
                    isFiring        ? "FIRING"
                    : launcherAtSpeed ? "At Speed - Launching"
                    : turretAimed   ? "Aimed - Spooling Up"
                                    : "Aiming...");
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Cancel LaunchSequenceOneCommand if it is still running.
        // Its own end() will call spindexer.stopFeedMotorsDirect().
        if (launchSequenceScheduled) {
            launchSequenceOne.cancel();
            launchSequenceScheduled = false;
        }

        // Stop all controlled mechanisms
        launcher.stopTurretDirect();
        launcher.stopLauncher();

        // Stop drivetrain
        drivetrain.setControl(new SwerveRequest.Idle());

        isFiring = false;
        SmartDashboard.putBoolean(DASH + "Firing", false);
        SmartDashboard.putString( DASH + "Status", interrupted ? "Interrupted" : "Complete");
    }

    @Override
    public boolean isFinished() {
        // Runs until interrupted (Y button released)
        return false;
    }

    // =========================================================================
    // Public accessors (for use in composed commands / triggers)
    // =========================================================================

    /** Returns {@code true} if the turret is currently within aim tolerance. */
    public boolean isTurretAimed() {
        int           towerTagId  = (lockedTagId > 0) ? lockedTagId : towerTagIdSupplier.getAsInt();
        Translation3d towerCenter = ShootingArcManager.getTowerCenter(drivetrain.getState().Pose, towerTagId);
        double        targetAngle = ShootingArcManager.calculateTurretAngle(
                drivetrain.getState().Pose, towerCenter);
        double        error       = Math.abs(targetAngle - launcher.getTurretPositionDegrees());
        return error < Constants.ShootingArc.kTurretAimToleranceDeg;
    }

    /** Returns {@code true} if the robot is currently firing. */
    public boolean isFiring() {
        return isFiring;
    }
}
