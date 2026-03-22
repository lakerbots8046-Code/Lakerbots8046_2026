package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Spindexer;
import frc.robot.util.ShootingArcManager;

/**
 * Shoot-on-move command — velocity-compensated shooting while the robot is moving.
 *
 * <p>This command mirrors the mechanism readiness logic in {@link ShootFromPointCommand},
 * but computes turret aim using a velocity-compensated trajectory estimate so the shot
 * accounts for robot translation during flight time.
 */
public class ShootOnMoveCommand extends Command {

    // ── Subsystems ────────────────────────────────────────────────────────────
    /** Read-only pose/speeds source. Not required by this command. */
    private final CommandSwerveDrivetrain drivetrain;
    /** Controls turret (CAN 7), flywheel (CAN 8), and hood (CAN 9). */
    private final Launcher                launcher;

    // ── Suppliers ─────────────────────────────────────────────────────────────
    /** Supplies the active tower tag ID (alliance-aware, vision-preferred). */
    private final IntSupplier    towerTagIdSupplier;
    /**
     * Kept for constructor compatibility with RobotContainer bindings.
     * Arc behavior was removed per request; this value is intentionally unused.
     */
    @SuppressWarnings("unused")
    private final DoubleSupplier lateralInputSupplier;

    // ── LaunchSequenceOne (scheduled when all conditions are met) ────────────
    private final LaunchSequenceOneCommand launchSequenceOne;
    /** Tracks whether launchSequenceOne is currently scheduled. */
    private boolean launchSequenceScheduled = false;

    // ── Flywheel spool-up timer ───────────────────────────────────────────────
    private double flywheelStartTimestamp = -1.0;
    private static final double FLYWHEEL_SPOOL_TIME_SECONDS = 2.0;

    // ── Dashboard throttle ────────────────────────────────────────────────────
    private int dashboardCounter = 0;

    // ── Hood deadband ─────────────────────────────────────────────────────────
    private double lastCommandedHoodRotations = Double.MAX_VALUE;
    private static final double kHoodDeadbandRotations = 0.2;

    // ── Tag locking ───────────────────────────────────────────────────────────
    private int lockedTagId = -1;
    private int pendingTagId = -1;
    private int pendingTagCounter = 0;
    private static final int TAG_SWITCH_STABILITY_LOOPS = 25;

    // ── State ─────────────────────────────────────────────────────────────────
    private boolean isFiring = false;

    // ── Dashboard key prefix ──────────────────────────────────────────────────
    private static final String DASH = "ShootOnMove/";

    // ── Lead compensation tuning ──────────────────────────────────────────────
    private static final double kDefaultLeadCompGain = 2.6;
    private static final double kDefaultWheelCircumferenceMeters = 0.319; // ~4" wheel fallback
    private static final double kDefaultMuzzleSpeedScale = 0.75;

    /** Seeds all ShootOnMove dashboard keys so they are visible even when command is idle. */
    public static void seedDashboardKeys() {
        SmartDashboard.putNumber(DASH + "Lead Gain", SmartDashboard.getNumber(DASH + "Lead Gain", kDefaultLeadCompGain));
        SmartDashboard.putNumber(DASH + "Wheel Circumference (m)",
                SmartDashboard.getNumber(DASH + "Wheel Circumference (m)", kDefaultWheelCircumferenceMeters));
        SmartDashboard.putNumber(DASH + "Muzzle Speed Scale",
                SmartDashboard.getNumber(DASH + "Muzzle Speed Scale", kDefaultMuzzleSpeedScale));

        SmartDashboard.putNumber(DASH + "Locked Tag ID", SmartDashboard.getNumber(DASH + "Locked Tag ID", -1));
        SmartDashboard.putNumber(DASH + "Tower Tag ID", SmartDashboard.getNumber(DASH + "Tower Tag ID", -1));
        SmartDashboard.putString(DASH + "Tower Tag Info", SmartDashboard.getString(DASH + "Tower Tag Info", "Idle"));
        SmartDashboard.putString(DASH + "Tag Alliance", SmartDashboard.getString(DASH + "Tag Alliance", "Unknown"));
        SmartDashboard.putBoolean(DASH + "In Shooting Zone", SmartDashboard.getBoolean(DASH + "In Shooting Zone", false));
        SmartDashboard.putNumber(DASH + "Distance (m)", SmartDashboard.getNumber(DASH + "Distance (m)", 0.0));

        SmartDashboard.putNumber(DASH + "Robot Vx Robot (mps)", SmartDashboard.getNumber(DASH + "Robot Vx Robot (mps)", 0.0));
        SmartDashboard.putNumber(DASH + "Robot Vy Robot (mps)", SmartDashboard.getNumber(DASH + "Robot Vy Robot (mps)", 0.0));
        SmartDashboard.putNumber(DASH + "Robot Vx Field (mps)", SmartDashboard.getNumber(DASH + "Robot Vx Field (mps)", 0.0));
        SmartDashboard.putNumber(DASH + "Robot Vy Field (mps)", SmartDashboard.getNumber(DASH + "Robot Vy Field (mps)", 0.0));
        SmartDashboard.putNumber(DASH + "Shooter Speed Estimate (mps)", SmartDashboard.getNumber(DASH + "Shooter Speed Estimate (mps)", 0.0));
        SmartDashboard.putNumber(DASH + "Flight Time (s)", SmartDashboard.getNumber(DASH + "Flight Time (s)", 0.0));
        SmartDashboard.putNumber(DASH + "Comp Dx (m)", SmartDashboard.getNumber(DASH + "Comp Dx (m)", 0.0));
        SmartDashboard.putNumber(DASH + "Comp Dy (m)", SmartDashboard.getNumber(DASH + "Comp Dy (m)", 0.0));

        SmartDashboard.putNumber(DASH + "Turret Target Uncomp (deg)",
                SmartDashboard.getNumber(DASH + "Turret Target Uncomp (deg)", 0.0));
        SmartDashboard.putNumber(DASH + "Turret Target Comp (deg)",
                SmartDashboard.getNumber(DASH + "Turret Target Comp (deg)", 0.0));
        SmartDashboard.putNumber(DASH + "Turret Lead Delta (deg)",
                SmartDashboard.getNumber(DASH + "Turret Lead Delta (deg)", 0.0));
        SmartDashboard.putNumber(DASH + "Turret Target (deg)", SmartDashboard.getNumber(DASH + "Turret Target (deg)", 0.0));
        SmartDashboard.putNumber(DASH + "Turret Error (deg)", SmartDashboard.getNumber(DASH + "Turret Error (deg)", 0.0));
        SmartDashboard.putBoolean(DASH + "Turret Aimed", SmartDashboard.getBoolean(DASH + "Turret Aimed", false));
        SmartDashboard.putString(DASH + "Turret Limit", SmartDashboard.getString(DASH + "Turret Limit", "Idle"));

        SmartDashboard.putNumber(DASH + "Hood Target (rot)", SmartDashboard.getNumber(DASH + "Hood Target (rot)", 0.0));
        SmartDashboard.putBoolean(DASH + "Hood At Target", SmartDashboard.getBoolean(DASH + "Hood At Target", false));

        SmartDashboard.putNumber(DASH + "Launcher RPS Target", SmartDashboard.getNumber(DASH + "Launcher RPS Target", 0.0));
        SmartDashboard.putNumber(DASH + "Launcher Actual RPS", SmartDashboard.getNumber(DASH + "Launcher Actual RPS", 0.0));
        SmartDashboard.putNumber(DASH + "Spool Time (s)", SmartDashboard.getNumber(DASH + "Spool Time (s)", 0.0));
        SmartDashboard.putBoolean(DASH + "Launcher At Speed", SmartDashboard.getBoolean(DASH + "Launcher At Speed", false));

        SmartDashboard.putBoolean(DASH + "Ready To Fire", SmartDashboard.getBoolean(DASH + "Ready To Fire", false));
        SmartDashboard.putBoolean(DASH + "Firing", SmartDashboard.getBoolean(DASH + "Firing", false));
        SmartDashboard.putString(DASH + "Status", SmartDashboard.getString(DASH + "Status", "Idle"));
    }

    public ShootOnMoveCommand(
            CommandSwerveDrivetrain drivetrain,
            Launcher                launcher,
            Spindexer               spindexer,
            IntSupplier             towerTagIdSupplier,
            DoubleSupplier          lateralInputSupplier) {

        this.drivetrain           = drivetrain;
        this.launcher             = launcher;
        this.towerTagIdSupplier   = towerTagIdSupplier;
        this.lateralInputSupplier = lateralInputSupplier;

        this.launchSequenceOne = new LaunchSequenceOneCommand(
                spindexer,
                launcher::getCollectVelocity);

        // drivetrain is intentionally NOT listed (read-only pose/speeds).
        addRequirements(launcher);
    }

    @Override
    public void initialize() {
        isFiring = false;
        launchSequenceScheduled = false;
        dashboardCounter = 0;
        flywheelStartTimestamp = -1.0;
        lastCommandedHoodRotations = Double.MAX_VALUE;

        lockedTagId = towerTagIdSupplier.getAsInt();
        pendingTagId = -1;
        pendingTagCounter = 0;

        // Shooting no longer overrides LED mode.

        seedDashboardKeys();
        SmartDashboard.putNumber(DASH + "Locked Tag ID", lockedTagId);
        SmartDashboard.putString(DASH + "Status", "Shoot On Move");
        SmartDashboard.putBoolean(DASH + "Firing", false);
    }

    @Override
    public void execute() {
        // ── 0. Tag locking with stability ─────────────────────────────────────
        int currentBestTag = towerTagIdSupplier.getAsInt();
        if (currentBestTag == lockedTagId) {
            pendingTagId = -1;
            pendingTagCounter = 0;
        } else if (currentBestTag == pendingTagId) {
            pendingTagCounter++;
            if (pendingTagCounter >= TAG_SWITCH_STABILITY_LOOPS) {
                lockedTagId = pendingTagId;
                pendingTagId = -1;
                pendingTagCounter = 0;
            }
        } else {
            pendingTagId = currentBestTag;
            pendingTagCounter = 1;
        }

        int towerTagId = lockedTagId;
        var robotPose = drivetrain.getState().Pose;
        Translation3d towerCenter = ShootingArcManager.getTowerCenter(robotPose, towerTagId);

        // ── 1. Velocity-compensated shot geometry ─────────────────────────────
        Translation2d robotRelativeVelocity = new Translation2d(
                drivetrain.getState().Speeds.vxMetersPerSecond,
                drivetrain.getState().Speeds.vyMetersPerSecond);
        Translation2d robotFieldVelocity = robotRelativeVelocity.rotateBy(robotPose.getRotation());

        Translation2d turretPos2d = ShootingArcManager.getTurretFieldPosition(robotPose).toTranslation2d();
        Translation2d towerPos2d  = towerCenter.toTranslation2d();
        Translation2d toTower     = towerPos2d.minus(turretPos2d);

        double horizontalDistance = toTower.getNorm();
        double verticalDelta      = towerCenter.getZ() - Constants.TurretConstants.kTurretOffsetZ;
        double directDistance     = Math.hypot(horizontalDistance, verticalDelta);

        // First pass launcher settings from direct line-of-sight distance
        double targetLauncherRPS = ShootingArcManager.calculateLauncherRPS(directDistance);
        double targetHoodRotations = ShootingArcManager.calculateHoodAngle(directDistance);

        // Tunables for compensation magnitude
        double leadGain = SmartDashboard.getNumber(DASH + "Lead Gain", kDefaultLeadCompGain);
        double wheelCircumferenceMeters =
                SmartDashboard.getNumber(DASH + "Wheel Circumference (m)", kDefaultWheelCircumferenceMeters);
        double muzzleSpeedScale = SmartDashboard.getNumber(DASH + "Muzzle Speed Scale", kDefaultMuzzleSpeedScale);

        // Derive approximate projectile speed from launcher RPS and circumference, then scale.
        double shooterMps = Math.max(
                0.1,
                Math.abs(targetLauncherRPS) * Math.max(0.01, wheelCircumferenceMeters) * Math.max(0.05, muzzleSpeedScale));

        double flightTimeSec = directDistance / shooterMps;

        // Lead compensation: subtract scaled robot travel during shot flight from target vector
        Translation2d compensatedToTower = toTower.minus(robotFieldVelocity.times(flightTimeSec * leadGain));
        double compensatedHorizontalDistance = compensatedToTower.getNorm();
        double distance = Math.hypot(compensatedHorizontalDistance, verticalDelta);

        // Optional second pass using compensated distance
        targetLauncherRPS = ShootingArcManager.calculateLauncherRPS(distance);
        targetHoodRotations = ShootingArcManager.calculateHoodAngle(distance);

        double rawUncompensatedFieldAngle = Math.atan2(toTower.getY(), toTower.getX());
        double rawCompensatedFieldAngle = Math.atan2(
                compensatedToTower.getY(),
                compensatedToTower.getX());
        double robotHeadingRad = robotPose.getRotation().getRadians();
        double targetTurretAngleUncomp = Math.toDegrees(rawUncompensatedFieldAngle - robotHeadingRad);
        double targetTurretAngle = Math.toDegrees(rawCompensatedFieldAngle - robotHeadingRad);

        while (targetTurretAngleUncomp > 180.0) targetTurretAngleUncomp -= 360.0;
        while (targetTurretAngleUncomp < -180.0) targetTurretAngleUncomp += 360.0;
        while (targetTurretAngle > 180.0) targetTurretAngle -= 360.0;
        while (targetTurretAngle < -180.0) targetTurretAngle += 360.0;

        // ── 2. Launcher + hood ────────────────────────────────────────────────
        launcher.setCollectVelocity(targetLauncherRPS);

        if (Math.abs(targetHoodRotations - lastCommandedHoodRotations) > kHoodDeadbandRotations) {
            launcher.setHoodPosition(targetHoodRotations);
            lastCommandedHoodRotations = targetHoodRotations;
        }

        if (flywheelStartTimestamp < 0) {
            flywheelStartTimestamp = Timer.getFPGATimestamp();
        }

        // ── 3. Turret aiming and limits ───────────────────────────────────────
        double currentTurretDeg = launcher.getTurretPositionDegrees();
        double turretError = targetTurretAngle - currentTurretDeg;
        while (turretError > 180.0) turretError -= 360.0;
        while (turretError < -180.0) turretError += 360.0;

        double targetRawRotations = targetTurretAngle * TurretConstants.kRotationsPerDegree;

        boolean turretAtLimit = !launcher.isTurretWithinLimits();
        boolean targetOutOfRange = Math.abs(targetRawRotations) > TurretConstants.kPhysicalLimitRotations;
        double clampedRaw = Math.max(-TurretConstants.kPhysicalLimitRotations,
                Math.min(TurretConstants.kPhysicalLimitRotations, targetRawRotations));

        final double kDeadbandDeg = 0.5;
        final String turretLimitStatus;

        if (turretAtLimit) {
            launcher.setTurretPosition(launcher.getTurretPosition());
            turretLimitStatus = "PAST LIMIT - STOPPED";
        } else if (targetOutOfRange) {
            launcher.setTurretPosition(clampedRaw);
            turretLimitStatus = String.format("Out of range (%.0f deg)", targetTurretAngle);
        } else if (Math.abs(turretError) > kDeadbandDeg) {
            launcher.setTurretPosition(targetRawRotations);
            turretLimitStatus = "OK";
        } else {
            turretLimitStatus = "OK (holding)";
        }

        // ── 4. Readiness gates (match ShootFromPoint logic) ───────────────────
        boolean turretAimed = !turretAtLimit && !targetOutOfRange
                && Math.abs(turretError) < Constants.ShootingArc.kTurretAimToleranceDeg;
        boolean hoodAtTarget = launcher.isHoodAtTarget();

        double actualRPS = Math.abs(launcher.getCollectVelocity());
        double targetRPS = Math.abs(targetLauncherRPS);
        double timeSpooling = (flywheelStartTimestamp >= 0)
                ? Timer.getFPGATimestamp() - flywheelStartTimestamp
                : 0.0;

        boolean launcherAtSpeed =
                (timeSpooling >= FLYWHEEL_SPOOL_TIME_SECONDS)
                || (actualRPS > 5.0 && Math.abs(actualRPS - targetRPS)
                        < Constants.ShootingArc.kLauncherVelocityTolerance);

        boolean readyToFire = launcherAtSpeed && turretAimed && hoodAtTarget;

        // ── 5. Schedule / cancel launch sequence ──────────────────────────────
        if (readyToFire && !launchSequenceScheduled) {
            CommandScheduler.getInstance().schedule(launchSequenceOne);
            launchSequenceScheduled = true;
            isFiring = true;
        } else if (!readyToFire && launchSequenceScheduled) {
            launchSequenceOne.cancel();
            launchSequenceScheduled = false;
            isFiring = false;
        }

        // ── 6. Telemetry (throttled) ───────────────────────────────────────────
        dashboardCounter++;
        if (dashboardCounter >= 5) {
            dashboardCounter = 0;

            String tagAlliance = ShootingArcManager.isRedTowerTag(towerTagId) ? "Red"
                    : ShootingArcManager.isBlueTowerTag(towerTagId) ? "Blue" : "Unknown";
            boolean isPrimary = (towerTagId == Constants.ShootingArc.kRedPrimaryTagId)
                    || (towerTagId == Constants.ShootingArc.kBluePrimaryTagId);
            String tagInfo = String.format("Tag %d (%s Tower%s)",
                    towerTagId, tagAlliance, isPrimary ? " ★" : "");
            boolean inZone = ShootingArcManager.isInShootingZone(robotPose, towerTagId);

            SmartDashboard.putNumber(DASH + "Locked Tag ID", lockedTagId);
            SmartDashboard.putNumber(DASH + "Tower Tag ID", towerTagId);
            SmartDashboard.putString(DASH + "Tower Tag Info", tagInfo);
            SmartDashboard.putString(DASH + "Tag Alliance", tagAlliance);
            SmartDashboard.putBoolean(DASH + "In Shooting Zone", inZone);
            SmartDashboard.putNumber(DASH + "Distance (m)", distance);

            SmartDashboard.putNumber(DASH + "Robot Vx Robot (mps)", robotRelativeVelocity.getX());
            SmartDashboard.putNumber(DASH + "Robot Vy Robot (mps)", robotRelativeVelocity.getY());
            SmartDashboard.putNumber(DASH + "Robot Vx Field (mps)", robotFieldVelocity.getX());
            SmartDashboard.putNumber(DASH + "Robot Vy Field (mps)", robotFieldVelocity.getY());
            SmartDashboard.putNumber(DASH + "Lead Gain", leadGain);
            SmartDashboard.putNumber(DASH + "Wheel Circumference (m)", wheelCircumferenceMeters);
            SmartDashboard.putNumber(DASH + "Muzzle Speed Scale", muzzleSpeedScale);
            SmartDashboard.putNumber(DASH + "Shooter Speed Estimate (mps)", shooterMps);
            SmartDashboard.putNumber(DASH + "Flight Time (s)", flightTimeSec);
            SmartDashboard.putNumber(DASH + "Comp Dx (m)", compensatedToTower.getX());
            SmartDashboard.putNumber(DASH + "Comp Dy (m)", compensatedToTower.getY());

            double leadDeltaDeg = targetTurretAngle - targetTurretAngleUncomp;
            while (leadDeltaDeg > 180.0) leadDeltaDeg -= 360.0;
            while (leadDeltaDeg < -180.0) leadDeltaDeg += 360.0;

            SmartDashboard.putNumber(DASH + "Turret Target Uncomp (deg)", targetTurretAngleUncomp);
            SmartDashboard.putNumber(DASH + "Turret Target Comp (deg)", targetTurretAngle);
            SmartDashboard.putNumber(DASH + "Turret Lead Delta (deg)", leadDeltaDeg);
            SmartDashboard.putNumber(DASH + "Turret Target (deg)", targetTurretAngle);
            SmartDashboard.putNumber(DASH + "Turret Error (deg)", turretError);
            SmartDashboard.putBoolean(DASH + "Turret Aimed", turretAimed);
            SmartDashboard.putString(DASH + "Turret Limit", turretLimitStatus);

            SmartDashboard.putNumber(DASH + "Hood Target (rot)", targetHoodRotations);
            SmartDashboard.putBoolean(DASH + "Hood At Target", hoodAtTarget);

            SmartDashboard.putNumber(DASH + "Launcher RPS Target", targetLauncherRPS);
            SmartDashboard.putNumber(DASH + "Launcher Actual RPS", launcher.getCollectVelocity());
            SmartDashboard.putNumber(DASH + "Spool Time (s)", timeSpooling);
            SmartDashboard.putBoolean(DASH + "Launcher At Speed", launcherAtSpeed);

            SmartDashboard.putBoolean(DASH + "Ready To Fire", readyToFire);
            SmartDashboard.putBoolean(DASH + "Firing", isFiring);
            SmartDashboard.putString(DASH + "Status",
                    isFiring ? "FIRING"
                            : !launcherAtSpeed ? String.format("Spooling (%.1f s)", timeSpooling)
                            : !turretAimed ? "At Speed - Aiming Turret"
                            : !hoodAtTarget ? "At Speed - Adjusting Hood"
                            : "At Speed - Firing");
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (launchSequenceScheduled) {
            launchSequenceOne.cancel();
            launchSequenceScheduled = false;
        }

        launcher.stopTurretDirect();
        launcher.stopLauncher();

        // Shooting no longer overrides LED mode.

        isFiring = false;
        SmartDashboard.putBoolean(DASH + "Firing", false);
        SmartDashboard.putString(DASH + "Status", interrupted ? "Interrupted" : "Complete");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /** Returns {@code true} if the turret is currently within aim tolerance. */
    public boolean isTurretAimed() {
        int towerTagId = (lockedTagId > 0) ? lockedTagId : towerTagIdSupplier.getAsInt();
        Translation3d towerCenter = ShootingArcManager.getTowerCenter(drivetrain.getState().Pose, towerTagId);

        Translation2d turretPos2d = ShootingArcManager.getTurretFieldPosition(drivetrain.getState().Pose).toTranslation2d();
        Translation2d toTower = towerCenter.toTranslation2d().minus(turretPos2d);
        double fieldAngle = Math.atan2(toTower.getY(), toTower.getX());
        double targetAngle = Math.toDegrees(fieldAngle - drivetrain.getState().Pose.getRotation().getRadians());

        while (targetAngle > 180.0) targetAngle -= 360.0;
        while (targetAngle < -180.0) targetAngle += 360.0;

        double error = Math.abs(targetAngle - launcher.getTurretPositionDegrees());
        return error < Constants.ShootingArc.kTurretAimToleranceDeg;
    }

    /** Returns {@code true} if the robot is currently firing. */
    public boolean isFiring() {
        return isFiring;
    }
}
