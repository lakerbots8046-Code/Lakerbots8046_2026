 package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
    private final Spindexer               spindexer;

    // ── Suppliers ─────────────────────────────────────────────────────────────
    /** Supplies the active tower tag ID (alliance-aware, vision-preferred). */
    private final IntSupplier    towerTagIdSupplier;
    /** Supplies left/right joystick input in [-1, 1] for arc sliding. */
    private final DoubleSupplier lateralInputSupplier;

    // ── Swerve request for arc sliding ────────────────────────────────────────
    private final SwerveRequest.FieldCentric arcDriveRequest;

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
        this.spindexer            = spindexer;
        this.towerTagIdSupplier   = towerTagIdSupplier;
        this.lateralInputSupplier = lateralInputSupplier;

        // Field-centric swerve request (no deadband — we compute exact velocities)
        this.arcDriveRequest = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(drivetrain, launcher, spindexer);
    }

    // =========================================================================
    // Command lifecycle
    // =========================================================================

    @Override
    public void initialize() {
        isFiring = false;
        SmartDashboard.putString(DASH + "Status",  "Shooting on Arc");
        SmartDashboard.putBoolean(DASH + "Firing", false);
    }

    @Override
    public void execute() {
        // ── 0. Gather context ─────────────────────────────────────────────────
        int           towerTagId  = towerTagIdSupplier.getAsInt();
        Translation2d towerCenter = ShootingArcManager.getTowerCenter(towerTagId);
        var           robotPose   = drivetrain.getState().Pose;

        // ── 1. Distance-based shooting parameters ─────────────────────────────
        double distance          = ShootingArcManager.calculateDistance(robotPose, towerCenter);
        double targetTurretAngle = ShootingArcManager.calculateTurretAngle(robotPose, towerCenter);
        double targetLauncherRPS = ShootingArcManager.calculateLauncherRPS(distance);
        // NOTE: Hood (CAN 9) is NOT controlled here — it stays at its current position.
        // Set the hood manually before pressing Y, or add a separate hood command if needed.

        // ── 2. Set launcher flywheel velocity ─────────────────────────────────
        launcher.setCollectVelocity(targetLauncherRPS);

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

        if (turretAtLimit) {
            // Case 1: PAST LIMIT — hold at current position to prevent further damage
            launcher.setTurretPosition(launcher.getTurretPosition());
            SmartDashboard.putString(DASH + "Turret Limit Status", "PAST LIMIT - STOPPED");

        } else if (targetOutOfRange) {
            // Case 2: OUT OF RANGE — drive to nearest limit and wait for robot to rotate
            launcher.setTurretPosition(clampedRaw);
            SmartDashboard.putString(DASH + "Turret Limit Status",
                    String.format("Out of range (%.1f deg) - holding at %.1f deg",
                            targetTurretAngle,
                            clampedRaw / TurretConstants.kRotationsPerDegree));

        } else if (Math.abs(turretError) > kDeadbandDeg) {
            // Case 3: NORMAL — error exceeds deadband, update Motion Magic target
            launcher.setTurretPosition(targetRawRotations);
            SmartDashboard.putString(DASH + "Turret Limit Status", "OK");

        } else {
            // Case 4: WITHIN DEADBAND — hold last commanded position (no profile reset)
            SmartDashboard.putString(DASH + "Turret Limit Status", "OK (holding)");
        }

        // ── 4. Evaluate fire conditions ───────────────────────────────────────
        // Do NOT fire if turret is past a limit or target is out of range.
        boolean turretAimed = !turretAtLimit && !targetOutOfRange
                && Math.abs(turretError) < Constants.ShootingArc.kTurretAimToleranceDeg;
        boolean launcherAtSpeed = Math.abs(launcher.getCollectVelocity() - targetLauncherRPS)
                                  < Constants.ShootingArc.kLauncherVelocityTolerance;
        boolean shouldFire = turretAimed && launcherAtSpeed;

        // ── 5. Control feed motors ────────────────────────────────────────────
        if (shouldFire) {
            spindexer.runFeedMotorsDirect(
                    TurretConstants.spindexerDutyCycleOut,
                    TurretConstants.starFeederDutyCycleOut,
                    TurretConstants.feederDutyCycleOut);
            isFiring = true;
        } else {
            spindexer.stopFeedMotorsDirect();
            isFiring = false;
        }

        // ── 6. Arc sliding — left/right joystick ─────────────────────────────
        double        lateralInput = lateralInputSupplier.getAsDouble();
        Translation2d arcVelocity  = ShootingArcManager.calculateArcVelocity(
                robotPose, towerCenter, lateralInput);

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

        // ── 9. Dashboard telemetry ────────────────────────────────────────────
        SmartDashboard.putNumber( DASH + "Distance (m)",          distance);
        SmartDashboard.putNumber( DASH + "Turret Target (deg)",   targetTurretAngle);
        SmartDashboard.putNumber( DASH + "Turret Current (deg)",  currentTurretDeg);
        SmartDashboard.putNumber( DASH + "Turret Error (deg)",    turretError);
        SmartDashboard.putNumber( DASH + "Turret Raw Target",     targetRawRotations);
        SmartDashboard.putBoolean(DASH + "Turret Aimed",          turretAimed);
        SmartDashboard.putNumber( DASH + "Launcher Target RPS",   targetLauncherRPS);
        SmartDashboard.putNumber( DASH + "Launcher Actual RPS",   launcher.getCollectVelocity());
        SmartDashboard.putBoolean(DASH + "Launcher At Speed",     launcherAtSpeed);
        SmartDashboard.putBoolean(DASH + "Firing",                isFiring);
        SmartDashboard.putNumber( DASH + "Tower Tag ID",          towerTagId);
        SmartDashboard.putNumber( DASH + "Arc Lateral Input",     lateralInput);
        SmartDashboard.putNumber( DASH + "Heading Error (deg)",   Math.toDegrees(headingError));
        SmartDashboard.putString( DASH + "Status",
                isFiring ? "FIRING" : (turretAimed ? "Aimed - Waiting for Speed" : "Aiming..."));
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all controlled mechanisms
        launcher.stopTurretDirect();
        launcher.stopLauncher();
        spindexer.stopFeedMotorsDirect();

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
        int           towerTagId  = towerTagIdSupplier.getAsInt();
        Translation2d towerCenter = ShootingArcManager.getTowerCenter(towerTagId);
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
