package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Spindexer;
import frc.robot.util.ShootingArcManager;

/**
 * Shoot-from-point command — stationary shooting from the robot's current field position.
 *
 * <p>Unlike {@link ShootOnMoveCommand}, this command does <em>not</em> move the drivetrain.
 * The robot stays exactly where it is. The command:
 * <ol>
 *   <li>Reads the robot's current field-relative pose from the drivetrain pose estimator.</li>
 *   <li>Calculates the distance to the active tower and derives the correct turret angle,
 *       hood position, and flywheel duty cycle from the lookup tables in
 *       {@link Constants.ShootingArc}.</li>
 *   <li>Continuously aims the turret (CAN 7) via Motion Magic position control.</li>
 *   <li>Sets the hood angle (CAN 9) via Motion Magic position control.</li>
 *   <li>Spins the flywheel (CAN 8) open-loop via DutyCycleOut.</li>
 *   <li>Schedules {@link LaunchSequenceOneCommand} (which proportionally scales the
 *       spindexer/feeder duty cycles) once <em>all three</em> conditions are met:
 *       <ul>
 *         <li>Turret within {@link Constants.ShootingArc#kTurretAimToleranceDeg}</li>
 *         <li>Hood within {@link Constants.LauncherConstants#kHoodPositionToleranceRotations}</li>
 *         <li>Flywheel spool-up time elapsed (1.5 s) or velocity within tolerance</li>
 *       </ul>
 *   </li>
 * </ol>
 *
 * <p>This command runs until interrupted (left trigger released below 75%).
 *
 * <h3>Subsystem requirements</h3>
 * {@code launcher} — turret (CAN 7), flywheel (CAN 8), hood (CAN 9).
 * {@code spindexer} is owned by {@link LaunchSequenceOneCommand} (scheduled separately).
 * {@code drivetrain} is read-only (pose only) — NOT a requirement, robot does not move.
 */
public class ShootFromPointCommand extends Command {

    // ── Subsystems ────────────────────────────────────────────────────────────
    /** Read-only — used for pose only. NOT in addRequirements(). */
    private final CommandSwerveDrivetrain drivetrain;
    /** Controls turret (CAN 7), flywheel (CAN 8), and hood (CAN 9). */
    private final Launcher                launcher;
    /** Passed to LaunchSequenceOneCommand — NOT in addRequirements() here. */
    //private final Spindexer               spindexer;
    /**
     * Intake subsystem — rollers are stopped in initialize() and re-enabled in end().
     * NOT added to addRequirements() so this command does not cancel intakeDeployCollect().
     * Uses stopRollersDirect() / enableRollers() which are safe to call without ownership.
     */
    private final Intake                  intake;

    // ── Suppliers ─────────────────────────────────────────────────────────────
    /** Supplies the active tower tag ID (alliance-aware, vision-preferred). */
    private final IntSupplier    towerTagIdSupplier;

    /**
     * Supplies a manual turret nudge rate each execute() loop.
     * Return value convention:
     *   +1.0 = nudge CCW (left)  — typically POV Left (270°)
     *   -1.0 = nudge CW  (right) — typically POV Right (90°)
     *    0.0 = no nudge
     * The command multiplies this by {@link #kPovNudgeDegreesPerLoop} and
     * accumulates it into {@link #manualTurretOffset}.
     */
    private final DoubleSupplier povNudgeSupplier;

    // ── LaunchSequenceOne (scheduled when all conditions are met) ─────────────
    private final LaunchSequenceOneCommand launchSequenceOne;
    /** Tracks whether launchSequenceOne is currently scheduled. */
    private boolean launchSequenceScheduled = false;

    // ── Flywheel spool-up timer ───────────────────────────────────────────────
    /**
     * FPGA timestamp (seconds) when the flywheel was first commanded this cycle.
     * -1 means the flywheel has not been commanded yet this cycle.
     */
    private double flywheelStartTimestamp = -1.0;

    /**
     * Time (seconds) to wait after the flywheel starts before considering it
     * "at speed". Matches the value used in {@link ShootOnMoveCommand}.
     */
    private static final double FLYWHEEL_SPOOL_TIME_SECONDS = 1.5;

    // ── POV manual turret override ────────────────────────────────────────────
    /**
     * Degrees added to the pose-calculated turret angle per execute() loop
     * when the POV nudge supplier returns ±1.0.
     * 0.5°/loop × 50 Hz = 25°/s — slow enough for fine adjustment.
     */
    private static final double kPovNudgeDegreesPerLoop = 0.5;

    /**
     * Maximum accumulated manual offset in either direction (degrees).
     * Prevents the driver from nudging the turret so far that it misses the tower entirely.
     */
    private static final double kPovNudgeMaxDegrees = 30.0;

    /**
     * Running manual turret offset (degrees) accumulated from POV nudge inputs.
     * Positive = CCW (left), negative = CW (right).
     * Reset to 0 in initialize() so each trigger press starts from the calculated angle.
     */
    private double manualTurretOffset = 0.0;

    // ── Dashboard throttle ────────────────────────────────────────────────────
    /** Counts execute() calls; telemetry is written every 5 calls (~100 ms). */
    private int dashboardCounter = 0;

    // ── Cached dashboard turret offset ───────────────────────────────────────
    /**
     * Live turret zero-offset (degrees) read from SmartDashboard.
     * Cached here and refreshed inside the throttled dashboard block (~10 Hz)
     * to avoid a NetworkTables read every execute() call (50 Hz).
     * Initialized in initialize() from the dashboard or the compiled constant.
     */
    private double liveOffset;

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

    // ── State ─────────────────────────────────────────────────────────────────
    private boolean isFiring = false;

    // ── Dashboard key prefix ──────────────────────────────────────────────────
    private static final String DASH = "ShootFromPoint/";

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * Creates a new ShootFromPointCommand.
     *
     * @param drivetrain         Swerve drivetrain (read-only — pose only, not required)
     * @param launcher           Launcher subsystem (turret + flywheel + hood)
     * @param spindexer          Spindexer subsystem (passed to LaunchSequenceOneCommand)
     * @param towerTagIdSupplier Supplies the active tower AprilTag ID at runtime
     * @param povNudgeSupplier   Supplies a manual turret nudge rate each loop.
     *                           Return +1.0 for CCW nudge (POV Left), -1.0 for CW nudge
     *                           (POV Right), 0.0 for no nudge.
     * @param intake             Intake subsystem — rollers are stopped on initialize and
     *                           re-enabled on end. Not added to requirements so this command
     *                           does not cancel intakeDeployCollect().
     */
    public ShootFromPointCommand(
            CommandSwerveDrivetrain drivetrain,
            Launcher                launcher,
            Spindexer               spindexer,
            IntSupplier             towerTagIdSupplier,
            DoubleSupplier          povNudgeSupplier,
            Intake                  intake) {

        this.drivetrain          = drivetrain;
        this.launcher            = launcher;
        //this.spindexer           = spindexer;
        this.towerTagIdSupplier  = towerTagIdSupplier;
        this.povNudgeSupplier    = povNudgeSupplier;
        this.intake              = intake;

        // LaunchSequenceOneCommand owns the spindexer requirement.
        // It is scheduled/cancelled based on readiness — not added to this
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
        flywheelStartTimestamp       = -1.0;
        lastCommandedHoodRotations   = Double.MAX_VALUE; // force update on first execute()
        manualTurretOffset           = 0.0; // reset nudge on each trigger press

        // Stop intake rollers for the duration of this command.
        // Sets rollersEnabled=false so intakeDeployCollect() will not restart them.
        // Safe to call without subsystem ownership — uses direct motor control.
        intake.stopRollersDirect();

        SmartDashboard.putString( DASH + "Status",  "Initializing");
        SmartDashboard.putBoolean(DASH + "Firing",  false);

        // Push the compiled offset as the initial dashboard value.
        // The driver/technician can change this number in Elastic/SmartDashboard
        // at runtime to fine-tune turret aim WITHOUT redeploying code.
        // Sign convention: positive = turret physical zero is CCW (left) of robot forward.
        SmartDashboard.putNumber(DASH + "Turret Zero Offset (deg)",
                frc.robot.Constants.TurretConstants.kTurretZeroOffsetDegrees);

        // Seed the cached offset from the dashboard (or compiled constant on first boot).
        liveOffset = SmartDashboard.getNumber(
                DASH + "Turret Zero Offset (deg)",
                frc.robot.Constants.TurretConstants.kTurretZeroOffsetDegrees);
    }

    @Override
    public void execute() {
        // ── 0. Gather context ─────────────────────────────────────────────────
        int           towerTagId  = towerTagIdSupplier.getAsInt();
        Translation2d towerCenter = ShootingArcManager.getTowerCenter(towerTagId);
        var           robotPose   = drivetrain.getState().Pose;

        // ── 1. Distance-based shooting parameters ─────────────────────────────
        double distance            = ShootingArcManager.calculateDistance(robotPose, towerCenter);
        double targetTurretAngle   = ShootingArcManager.calculateTurretAngle(robotPose, towerCenter);
        double targetLauncherRPS   = ShootingArcManager.calculateLauncherRPS(distance);
        double targetHoodRotations = ShootingArcManager.calculateHoodAngle(distance);

        // ── 2. Spin flywheel closed-loop (VelocityVoltage) ────────────────────
        // Closed-loop control lets the PID actively recover speed when a ball
        // loads the flywheel, rather than just holding a fixed duty cycle.
        // kV handles steady-state; kP (0.3) handles transient speed drops.
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

        // ── 3. Aim turret (CAN 7) via Motion Magic position control ──────────
        //
        // Use calculateTurretAngleRaw() (no compiled offset) and apply the
        // dashboard-adjustable offset instead. This lets the driver tune the
        // offset in Elastic without redeploying code.
        double rawTurretAngle = ShootingArcManager.calculateTurretAngleRaw(robotPose, towerCenter);

        // Accumulate POV manual nudge (±kPovNudgeDegreesPerLoop per loop, clamped to ±kPovNudgeMaxDegrees).
        // POV Left (270°) → supplier returns +1.0 → nudge CCW (positive offset).
        // POV Right (90°) → supplier returns -1.0 → nudge CW  (negative offset).
        manualTurretOffset = Math.max(-kPovNudgeMaxDegrees,
                Math.min(kPovNudgeMaxDegrees,
                        manualTurretOffset + povNudgeSupplier.getAsDouble() * kPovNudgeDegreesPerLoop));

        // Apply cached zero-offset + manual POV nudge
        double adjustedTurretAngle = rawTurretAngle - liveOffset + manualTurretOffset;
        while (adjustedTurretAngle >  180.0) adjustedTurretAngle -= 360.0;
        while (adjustedTurretAngle < -180.0) adjustedTurretAngle += 360.0;

        // Override the pose-based target with the adjusted version
        targetTurretAngle = adjustedTurretAngle;

        double currentTurretDeg = launcher.getTurretPositionDegrees();
        double turretError      = targetTurretAngle - currentTurretDeg;
        // Normalize error to [-180, 180]
        while (turretError >  180.0) turretError -= 360.0;
        while (turretError < -180.0) turretError += 360.0;

        double targetRawRotations = targetTurretAngle * TurretConstants.kRotationsPerDegree;

        // ── Limit enforcement ±18 raw motor rotations ───
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

        // ── 4. Evaluate all three fire conditions ─────────────────────────────
        boolean turretAimed = !turretAtLimit && !targetOutOfRange
                && Math.abs(turretError) < Constants.ShootingArc.kTurretAimToleranceDeg;

        boolean hoodAtTarget = launcher.isHoodAtTarget();

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

        // LaunchSequenceOneCommand is triggered as soon as the flywheel reaches speed.
        // Turret aim and hood position are still tracked for telemetry but do NOT gate firing.
        // This matches the behaviour of ShootOnMoveCommand.
        boolean readyToFire = launcherAtSpeed;

        // ── 5. Schedule / cancel LaunchSequenceOneCommand ─────────────────────
        if (readyToFire && !launchSequenceScheduled) {
            CommandScheduler.getInstance().schedule(launchSequenceOne);
            launchSequenceScheduled = true;
            isFiring = true;
        } else if (!readyToFire && launchSequenceScheduled) {
            launchSequenceOne.cancel();
            launchSequenceScheduled = false;
            isFiring = false;
        }

        // ── 6. Dashboard telemetry (throttled to every 5 loops ~100 ms) ───────
        dashboardCounter++;
        if (dashboardCounter >= 5) {
            dashboardCounter = 0;

            // Refresh the cached turret offset from the dashboard (~10 Hz).
            // Doing this inside the throttled block avoids a 50 Hz NT read.
            // Note: "Turret Zero Offset (deg)" is writable from the dashboard — do NOT overwrite it here.
            liveOffset = SmartDashboard.getNumber(
                    DASH + "Turret Zero Offset (deg)",
                    frc.robot.Constants.TurretConstants.kTurretZeroOffsetDegrees);

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

            // ── Robot pose diagnostics ─────────────────────────────────────────
            SmartDashboard.putNumber( DASH + "Robot X (m)",             robotPose.getX());
            SmartDashboard.putNumber( DASH + "Robot Y (m)",             robotPose.getY());
            SmartDashboard.putNumber( DASH + "Robot Heading (deg)",     robotPose.getRotation().getDegrees());

            // ── Turret angle diagnostics ───────────────────────────────────────
            SmartDashboard.putNumber( DASH + "Turret Raw Angle (deg)",    rawTurretAngle);
            SmartDashboard.putNumber( DASH + "Turret Manual Offset (deg)", manualTurretOffset);
            SmartDashboard.putNumber( DASH + "Turret Target (deg)",       targetTurretAngle);
            SmartDashboard.putNumber( DASH + "Turret Position (deg)",   currentTurretDeg);
            SmartDashboard.putNumber( DASH + "Turret Target (raw rot)", targetRawRotations);
            SmartDashboard.putNumber( DASH + "Turret Position (raw rot)", launcher.getTurretPosition());
            SmartDashboard.putNumber( DASH + "Turret Error (deg)",      turretError);
            SmartDashboard.putBoolean(DASH + "Turret Aimed",            turretAimed);

            // ── Tag identification outputs ─────────────────────────────────────
            SmartDashboard.putNumber( DASH + "Tower Tag ID",        towerTagId);
            SmartDashboard.putString( DASH + "Tower Tag Info",      tagInfo);
            SmartDashboard.putString( DASH + "Tag Alliance",        tagAlliance);
            SmartDashboard.putBoolean(DASH + "In Shooting Zone",    inZone);
            SmartDashboard.putNumber( DASH + "Distance (m)",        distance);

            // ── Mechanism outputs ──────────────────────────────────────────────
            SmartDashboard.putNumber( DASH + "Hood Target (rot)",   targetHoodRotations);
            SmartDashboard.putBoolean(DASH + "Hood At Target",      hoodAtTarget);
            SmartDashboard.putNumber( DASH + "Launcher RPS Target", targetLauncherRPS);
            SmartDashboard.putNumber( DASH + "Launcher Actual RPS", launcher.getCollectVelocity());
            SmartDashboard.putNumber( DASH + "Spool Time (s)",      timeSpooling);
            SmartDashboard.putBoolean(DASH + "Launcher At Speed",   launcherAtSpeed);
            SmartDashboard.putBoolean(DASH + "Ready To Fire",       readyToFire);
            SmartDashboard.putBoolean(DASH + "Firing",              isFiring);
            SmartDashboard.putString( DASH + "Status",
                    isFiring           ? "FIRING"
                    : !launcherAtSpeed ? String.format("Spooling (%.1f s)", timeSpooling)
                    : !turretAimed     ? "At Speed - Aiming Turret"
                    : !hoodAtTarget    ? "At Speed - Adjusting Hood"
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

        // Return turret to zero (home/forward) position via Motion Magic.
        // The TalonFX motor controller will continue executing this request
        // after the command ends, driving the turret back to 0 raw rotations
        // (robot forward direction) until another command takes over.
        launcher.setTurretPosition(0.0);

        // Stop flywheel. Hood stays at last position (safe to leave — retractHood()
        // default command will bring it home on the next scheduler cycle).
        launcher.stopLauncher();

        // Re-enable intake rollers now that shooting is complete.
        // If intakeDeployCollect() is still running, it will resume spinning
        // the rollers on its next loop. If it is not running, this is a no-op.
        intake.enableRollers();

        isFiring = false;
        SmartDashboard.putBoolean(DASH + "Firing",  false);
        SmartDashboard.putString( DASH + "Status",  interrupted ? "Interrupted - Turret Returning to Zero"
                                                                : "Complete - Turret Returning to Zero");
    }

    @Override
    public boolean isFinished() {
        // Runs until interrupted (left trigger released below 75%)
        return false;
    }
}
