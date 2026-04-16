package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
 * Velocity-compensated shooting while the robot is moving.
 *
 * <p>Uses ShootFromPoint-style readiness gates, but applies translational lead
 * to turret yaw based on estimated projectile flight time.
 */
public class ShootOnMoveCommand extends Command {

    // ── Subsystems ────────────────────────────────────────────────────────────
    /** Read-only pose/speeds source. Not required by this command. */
    private final CommandSwerveDrivetrain drivetrain;
    /** Controls turret (CAN 7), flywheel (CAN 8), and hood (CAN 9). */
    private final Launcher                launcher;
    /** Controls intake rollers/pivot subsystem. */
    private final Intake                  intake;

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

    // ── Turret stabilization state ────────────────────────────────────────────
    private double lastSlewLimitedTurretTargetDeg = 0.0;
    private int turretSettledCounter = 0;
    private int turretMissCounter = 0;
    private double lastExecuteTimestamp = -1.0;

    // ── Hood deadband ─────────────────────────────────────────────────────────
    private double lastCommandedHoodRotations = Double.MAX_VALUE;
    private static final double kHoodDeadbandRotations = 0.2;


    // ── State ─────────────────────────────────────────────────────────────────
    private boolean isFiring = false;
    private static volatile boolean shootOnMoveActive = false;

    // ── Dashboard key prefix ──────────────────────────────────────────────────
    private static final String DASH = "ShootOnMove/";

    // ── Lead compensation state ───────────────────────────────────────────────
    private Translation2d filteredRobotFieldVelocity = new Translation2d();

    /**
     * Selects the best turret target in raw rotations by considering equivalent
     * wrapped angles (base, +360°, -360°) and choosing the in-range option with
     * the smallest travel from current position.
     *
     * @param baseTargetRaw base target in raw motor rotations
     * @param currentRaw current turret position in raw motor rotations
     * @return best in-range target raw rotations, or nearest clamped base target if none in range
     */
    private double selectBestWrappedTurretTargetRaw(double baseTargetRaw, double currentRaw) {
        double limit = TurretConstants.kPhysicalLimitRotations;

        double[] candidates = new double[] {
            baseTargetRaw,
            baseTargetRaw + 360.0 * TurretConstants.kRotationsPerDegree,
            baseTargetRaw - 360.0 * TurretConstants.kRotationsPerDegree
        };

        double best = Double.NaN;
        double bestDelta = Double.POSITIVE_INFINITY;

        for (double candidate : candidates) {
            if (Math.abs(candidate) <= limit) {
                double delta = Math.abs(candidate - currentRaw);
                if (delta < bestDelta) {
                    bestDelta = delta;
                    best = candidate;
                }
            }
        }

        if (!Double.isNaN(best)) {
            return best;
        }

        // Fallback if no equivalent candidate is physically reachable.
        return Math.max(-limit, Math.min(limit, baseTargetRaw));
    }

    /** Seeds all ShootOnMove dashboard keys so they are visible even when command is idle. */
    public static void seedDashboardKeys() {
        SmartDashboard.putNumber(DASH + "Lead Gain", Constants.ShootingArc.kShootOnMoveLeadCompGain);
        SmartDashboard.putNumber(DASH + "Wheel Circumference (m)",
                Constants.ShootingArc.kShootOnMoveWheelCircumferenceMeters);
        SmartDashboard.putNumber(DASH + "Muzzle Speed Scale",
                Constants.ShootingArc.kShootOnMoveMuzzleSpeedScale);
        SmartDashboard.putNumber(DASH + "Latency Sec", Constants.ShootingArc.kShootOnMoveExtraLatencySec);
        SmartDashboard.putNumber(DASH + "Lead Iterations", Constants.ShootingArc.kShootOnMoveLeadSolveIterations);
        SmartDashboard.putNumber(DASH + "Velocity LPF Alpha", Constants.ShootingArc.kShootOnMoveVelocityLpfAlpha);
        SmartDashboard.putNumber(DASH + "Lead Gain X", Constants.ShootingArc.kShootOnMoveLeadCompGainX);
        SmartDashboard.putNumber(DASH + "Lead Gain Y", Constants.ShootingArc.kShootOnMoveLeadCompGainY);
        SmartDashboard.putNumber(DASH + "Velocity Lookahead Sec", Constants.ShootingArc.kShootOnMoveVelocityLookaheadSec);
        SmartDashboard.putNumber(DASH + "Max Lead Meters", Constants.ShootingArc.kShootOnMoveMaxLeadMeters);
        SmartDashboard.putBoolean(DASH + "Invert Lead", SmartDashboard.getBoolean(DASH + "Invert Lead", false));

        SmartDashboard.putNumber(DASH + "Tower Tag ID", SmartDashboard.getNumber(DASH + "Tower Tag ID", -1));
        SmartDashboard.putString(DASH + "Tower Tag Info", SmartDashboard.getString(DASH + "Tower Tag Info", "Idle"));
        SmartDashboard.putString(DASH + "Tag Alliance", SmartDashboard.getString(DASH + "Tag Alliance", "Unknown"));
        SmartDashboard.putBoolean(DASH + "In Shooting Zone", SmartDashboard.getBoolean(DASH + "In Shooting Zone", false));
        SmartDashboard.putNumber(DASH + "Distance (m)", SmartDashboard.getNumber(DASH + "Distance (m)", 0.0));

        SmartDashboard.putNumber(DASH + "Robot Vx Robot (mps)", SmartDashboard.getNumber(DASH + "Robot Vx Robot (mps)", 0.0));
        SmartDashboard.putNumber(DASH + "Robot Vy Robot (mps)", SmartDashboard.getNumber(DASH + "Robot Vy Robot (mps)", 0.0));
        SmartDashboard.putNumber(DASH + "Robot Vx Field (mps)", SmartDashboard.getNumber(DASH + "Robot Vx Field (mps)", 0.0));
        SmartDashboard.putNumber(DASH + "Robot Vy Field (mps)", SmartDashboard.getNumber(DASH + "Robot Vy Field (mps)", 0.0));
        SmartDashboard.putNumber(DASH + "Radial Velocity (mps)", SmartDashboard.getNumber(DASH + "Radial Velocity (mps)", 0.0));
        SmartDashboard.putNumber(DASH + "Tangential Velocity (mps)", SmartDashboard.getNumber(DASH + "Tangential Velocity (mps)", 0.0));
        SmartDashboard.putNumber(DASH + "Shooter Speed Estimate (mps)", SmartDashboard.getNumber(DASH + "Shooter Speed Estimate (mps)", 0.0));
        SmartDashboard.putNumber(DASH + "Flight Time (s)", SmartDashboard.getNumber(DASH + "Flight Time (s)", 0.0));
        SmartDashboard.putNumber(DASH + "Flight Time Formula (s)", SmartDashboard.getNumber(DASH + "Flight Time Formula (s)", 0.0));
        SmartDashboard.putNumber(DASH + "Flight Time Table (s)", SmartDashboard.getNumber(DASH + "Flight Time Table (s)", 0.0));
        SmartDashboard.putNumber(DASH + "Predicted Pose X (m)", SmartDashboard.getNumber(DASH + "Predicted Pose X (m)", 0.0));
        SmartDashboard.putNumber(DASH + "Predicted Pose Y (m)", SmartDashboard.getNumber(DASH + "Predicted Pose Y (m)", 0.0));
        SmartDashboard.putNumber(DASH + "Predicted Heading (deg)", SmartDashboard.getNumber(DASH + "Predicted Heading (deg)", 0.0));
        SmartDashboard.putNumber(DASH + "Predicted Distance (m)", SmartDashboard.getNumber(DASH + "Predicted Distance (m)", 0.0));
        SmartDashboard.putNumber(DASH + "Lead Time Used (s)", SmartDashboard.getNumber(DASH + "Lead Time Used (s)", 0.0));
        SmartDashboard.putNumber(DASH + "Lead Time Clamped (s)", SmartDashboard.getNumber(DASH + "Lead Time Clamped (s)", 0.0));

        SmartDashboard.putNumber(DASH + "Turret Target Uncomp (deg)",
                SmartDashboard.getNumber(DASH + "Turret Target Uncomp (deg)", 0.0));
        SmartDashboard.putNumber(DASH + "Turret Target Comp (deg)",
                SmartDashboard.getNumber(DASH + "Turret Target Comp (deg)", 0.0));
        SmartDashboard.putNumber(DASH + "Turret Lead Delta (deg)",
                SmartDashboard.getNumber(DASH + "Turret Lead Delta (deg)", 0.0));
        SmartDashboard.putNumber(DASH + "Turret Target (deg)", SmartDashboard.getNumber(DASH + "Turret Target (deg)", 0.0));
        SmartDashboard.putNumber(DASH + "Turret Target Slew Limited (deg)",
                SmartDashboard.getNumber(DASH + "Turret Target Slew Limited (deg)", 0.0));
        SmartDashboard.putNumber(DASH + "Turret Error (deg)", SmartDashboard.getNumber(DASH + "Turret Error (deg)", 0.0));
        SmartDashboard.putBoolean(DASH + "Turret Aimed", SmartDashboard.getBoolean(DASH + "Turret Aimed", false));
        SmartDashboard.putNumber(DASH + "Turret Settled Counter",
                SmartDashboard.getNumber(DASH + "Turret Settled Counter", 0.0));
        SmartDashboard.putNumber(DASH + "Turret Settle Loops Required",
                SmartDashboard.getNumber(DASH + "Turret Settle Loops Required", 0.0));
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

    public static boolean isShootOnMoveActive() {
        return shootOnMoveActive;
    }

    public ShootOnMoveCommand(
            CommandSwerveDrivetrain drivetrain,
            Launcher                launcher,
            Spindexer               spindexer,
            Intake                  intake,
            IntSupplier             towerTagIdSupplier,
            DoubleSupplier          lateralInputSupplier) {

        this.drivetrain           = drivetrain;
        this.launcher             = launcher;
        this.intake               = intake;
        this.towerTagIdSupplier   = towerTagIdSupplier;
        this.lateralInputSupplier = lateralInputSupplier;

        this.launchSequenceOne = new LaunchSequenceOneCommand(spindexer);

        // drivetrain and intake are intentionally NOT listed (read-only / shared roller override behavior).
        addRequirements(launcher);
    }

    @Override
    public void initialize() {
        shootOnMoveActive = true;
        isFiring = false;
        launchSequenceScheduled = false;
        dashboardCounter = 0;
        flywheelStartTimestamp = -1.0;
        lastCommandedHoodRotations = Double.MAX_VALUE;
        filteredRobotFieldVelocity = new Translation2d();

        lastSlewLimitedTurretTargetDeg = 0.0;
        turretSettledCounter = 0;
        turretMissCounter = 0;
        lastExecuteTimestamp = Timer.getFPGATimestamp();

        // Shooting no longer overrides LED mode.

        intake.setRollersVelocity(Constants.IntakeConstants.kRollersIntakeVelocity);

        seedDashboardKeys();
        SmartDashboard.putString(DASH + "Status", "Shoot On Move");
        SmartDashboard.putBoolean(DASH + "Firing", false);
    }

    @Override
    public void execute() {
        // Keep intake collecting during ShootOnMove. If operator outtake is held,
        // that command can override while active and this resumes on release.
        intake.setRollersVelocity(Constants.IntakeConstants.kRollersIntakeVelocity);

        // ── 0. Target selection (no tag locking) ──────────────────────────────
        int towerTagId = towerTagIdSupplier.getAsInt();
        var robotPose = drivetrain.getState().Pose;
        Translation3d towerCenter = ShootingArcManager.getTowerCenter(robotPose, towerTagId);

        // ── 1. Velocity-compensated shot geometry ─────────────────────────────
        Translation2d robotRelativeVelocity = new Translation2d(
                drivetrain.getState().Speeds.vxMetersPerSecond,
                drivetrain.getState().Speeds.vyMetersPerSecond);
        Translation2d measuredRobotFieldVelocity = robotRelativeVelocity.rotateBy(robotPose.getRotation());

        // Low-pass filter field velocity for lead stability
        double alpha = Constants.ShootingArc.kShootOnMoveVelocityLpfAlpha;
        alpha = Math.max(0.0, Math.min(1.0, alpha));
        filteredRobotFieldVelocity = filteredRobotFieldVelocity.times(1.0 - alpha)
                .plus(measuredRobotFieldVelocity.times(alpha));
        Translation2d robotFieldVelocity = filteredRobotFieldVelocity;

        Translation2d turretPos2d = ShootingArcManager.getTurretFieldPosition(robotPose).toTranslation2d();
        Translation2d towerPos2d  = towerCenter.toTranslation2d();
        Translation2d toTower     = towerPos2d.minus(turretPos2d);

        Translation2d radialUnit = (toTower.getNorm() > 1e-6)
                ? toTower.div(toTower.getNorm())
                : new Translation2d(1.0, 0.0);
        Translation2d tangentialUnit = new Translation2d(-radialUnit.getY(), radialUnit.getX());

        double radialVelocityMps = robotFieldVelocity.getX() * radialUnit.getX()
                + robotFieldVelocity.getY() * radialUnit.getY();
        double tangentialVelocityMps = robotFieldVelocity.getX() * tangentialUnit.getX()
                + robotFieldVelocity.getY() * tangentialUnit.getY();

        double horizontalDistance = toTower.getNorm();
        double verticalDelta      = towerCenter.getZ() - Constants.TurretConstants.kTurretOffsetZ;
        double directDistance     = Math.hypot(horizontalDistance, verticalDelta);

        // Constants-only compensation parameters
        double leadGain = Constants.ShootingArc.kShootOnMoveLeadCompGain;
        double leadGainX = Constants.ShootingArc.kShootOnMoveLeadCompGainX;
        double leadGainY = Constants.ShootingArc.kShootOnMoveLeadCompGainY;
        double wheelCircumferenceMeters = Constants.ShootingArc.kShootOnMoveWheelCircumferenceMeters;
        double muzzleSpeedScale = Constants.ShootingArc.kShootOnMoveMuzzleSpeedScale;
        double extraLatencySec = Math.max(0.0, Constants.ShootingArc.kShootOnMoveExtraLatencySec);
        double velocityLookaheadSec = Math.max(0.0, Constants.ShootingArc.kShootOnMoveVelocityLookaheadSec);
        double maxLeadMeters = Math.max(0.0, Constants.ShootingArc.kShootOnMoveMaxLeadMeters);
        int leadIterations = Math.max(1, Constants.ShootingArc.kShootOnMoveLeadSolveIterations);

        // Iterative full future-pose solve (x, y, heading)
        double predictedDistance = directDistance;
        double targetLauncherRPS = ShootingArcManager.calculateLauncherRPS(predictedDistance);
        double targetHoodRotations = ShootingArcManager.calculateHoodAngle(predictedDistance);
        double shooterMps = 0.1;
        double flightTimeSec = 0.0;
        double leadTimeUsedSec = 0.0;
        Pose2d predictedRobotPose = robotPose;
        Translation2d predictedToTower = toTower;
        boolean invertLead = SmartDashboard.getBoolean(DASH + "Invert Lead", false);

        for (int i = 0; i < leadIterations; i++) {
            targetLauncherRPS = ShootingArcManager.calculateLauncherRPS(predictedDistance);

            shooterMps = Math.max(
                    0.1,
                    Math.abs(targetLauncherRPS)
                            * Math.max(0.01, wheelCircumferenceMeters)
                            * Math.max(0.05, muzzleSpeedScale));

            double formulaFlightTimeSec = (predictedDistance / shooterMps) + extraLatencySec;
            double tableFlightTimeSec = interpolate(Constants.ShootingArc.kShootOnMoveTofLookup, predictedDistance) + extraLatencySec;
            flightTimeSec = Constants.ShootingArc.kShootOnMoveUseTofTable ? tableFlightTimeSec : formulaFlightTimeSec;
            leadTimeUsedSec = flightTimeSec + velocityLookaheadSec;
            leadTimeUsedSec = Math.min(leadTimeUsedSec, Math.max(0.0, Constants.ShootingArc.kShootOnMoveMaxPredictionSec));

            double baseRadialScale = Constants.ShootingArc.kShootOnMoveRadialCompScale;
            double radialScaleAway = Constants.ShootingArc.kShootOnMoveRadialCompScaleAway;
            double radialScaleToward = Constants.ShootingArc.kShootOnMoveRadialCompScaleToward;
            double radialScale = radialVelocityMps > 0.0 ? radialScaleAway
                    : radialVelocityMps < 0.0 ? radialScaleToward
                    : baseRadialScale;
            double tangentialScale = Constants.ShootingArc.kShootOnMoveTangentialCompScale;

            // Apply gain per field axis first, then project into radial/tangential basis.
            // This makes Lead Gain X/Y tuning predictable and avoids compounded scalar distortion.
            double effectiveLeadGainX = leadGain * leadGainX;
            double effectiveLeadGainY = leadGain * leadGainY;

            Translation2d gainedFieldVelocity = new Translation2d(
                    robotFieldVelocity.getX() * effectiveLeadGainX,
                    robotFieldVelocity.getY() * effectiveLeadGainY);

            double gainedRadialVelocityMps = gainedFieldVelocity.getX() * radialUnit.getX()
                    + gainedFieldVelocity.getY() * radialUnit.getY();
            double gainedTangentialVelocityMps = gainedFieldVelocity.getX() * tangentialUnit.getX()
                    + gainedFieldVelocity.getY() * tangentialUnit.getY();

            double radialComp = gainedRadialVelocityMps * leadTimeUsedSec * radialScale;
            double tangentialComp = gainedTangentialVelocityMps * leadTimeUsedSec * tangentialScale;

            Translation2d leadVector = radialUnit.times(radialComp).plus(tangentialUnit.times(tangentialComp));

            if (maxLeadMeters > 0.0 && leadVector.getNorm() > maxLeadMeters) {
                leadVector = leadVector.times(maxLeadMeters / Math.max(1e-6, leadVector.getNorm()));
            }

            // Canonical sign convention: compensate opposite robot motion by default.
            // Invert Lead dashboard toggle intentionally flips this behavior for quick A/B tuning.
            Translation2d appliedLead = invertLead ? leadVector : leadVector.unaryMinus();

            double predictedX = robotPose.getX() + appliedLead.getX();
            double predictedY = robotPose.getY() + appliedLead.getY();
            double omegaRadPerSec = drivetrain.getState().Speeds.omegaRadiansPerSecond;
            double predictedHeading = robotPose.getRotation().getRadians() + (omegaRadPerSec * leadTimeUsedSec);

            predictedRobotPose = new Pose2d(predictedX, predictedY, new Rotation2d(predictedHeading));

            Translation2d predictedTurretPos2d = ShootingArcManager
                    .getTurretFieldPosition(predictedRobotPose)
                    .toTranslation2d();
            predictedToTower = towerPos2d.minus(predictedTurretPos2d);

            double predictedHorizontalDistance = predictedToTower.getNorm();
            predictedDistance = Math.hypot(predictedHorizontalDistance, verticalDelta);
        }

        // Launcher stays based on predicted distance (motion-compensated).
        targetLauncherRPS = ShootingArcManager.calculateLauncherRPS(predictedDistance);

        // Hood can use direct distance at close range to avoid jitter while driving toward goal.
        boolean useDirectHoodAtClose = Constants.ShootingArc.kShootOnMoveUseDirectDistanceForHoodAtCloseRange
                && directDistance <= Constants.ShootingArc.kShootOnMoveHoodDirectDistanceThresholdMeters;
        double hoodDistanceForLookup = useDirectHoodAtClose ? directDistance : predictedDistance;
        targetHoodRotations = ShootingArcManager.calculateHoodAngle(hoodDistanceForLookup);

        double rawUncompensatedFieldAngle = Math.atan2(toTower.getY(), toTower.getX());
        double rawCompensatedFieldAngle = Math.atan2(
                predictedToTower.getY(),
                predictedToTower.getX());
        double robotHeadingRad = robotPose.getRotation().getRadians();
        double targetTurretAngleUncomp = Math.toDegrees(rawUncompensatedFieldAngle - robotHeadingRad);
        double targetTurretAngleRaw = Math.toDegrees(rawCompensatedFieldAngle - robotHeadingRad);

        // Slew-limit turret command so prediction does not outrun turret dynamics.
        double nowSec = Timer.getFPGATimestamp();
        double dtSec = (lastExecuteTimestamp > 0.0) ? Math.max(1e-3, nowSec - lastExecuteTimestamp) : 0.02;
        lastExecuteTimestamp = nowSec;

        double maxStepDeg = Math.max(0.0, Constants.ShootingArc.kShootOnMoveTurretSlewRateDegPerSec) * dtSec;
        double deltaDeg = targetTurretAngleRaw - lastSlewLimitedTurretTargetDeg;
        double limitedDeltaDeg = Math.max(-maxStepDeg, Math.min(maxStepDeg, deltaDeg));
        double targetTurretAngle = lastSlewLimitedTurretTargetDeg + limitedDeltaDeg;
        lastSlewLimitedTurretTargetDeg = targetTurretAngle;

        // Keep raw field-relative solution (do not normalize to [-180, 180])
        // so the turret can use the extended mechanical range.

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
        double baseTargetRaw = targetTurretAngle * TurretConstants.kRotationsPerDegree;
        double currentTurretRaw = launcher.getTurretPosition();
        double targetRawRotations = selectBestWrappedTurretTargetRaw(baseTargetRaw, currentTurretRaw);

        // Compute error in raw-rotation domain so aiming respects extended range.
        double turretError = (targetRawRotations - currentTurretRaw)
                / TurretConstants.kRotationsPerDegree;

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
        boolean turretAimedInstant = !turretAtLimit && !targetOutOfRange
                && Math.abs(turretError) < Constants.ShootingArc.kTurretAimToleranceDeg;

        if (turretAimedInstant) {
            turretSettledCounter++;
            turretMissCounter = 0;
        } else {
            turretMissCounter++;
            if (turretMissCounter >= 2) {
                turretSettledCounter = 0;
            }
        }

        int settleLoopsBase = Math.max(1, Constants.ShootingArc.kShootOnMoveTurretSettleLoops);
        int settleLoopsMin = Math.max(1, Constants.ShootingArc.kShootOnMoveTurretSettleLoopsMin);
        double lateralRelaxStart = Math.max(0.0, Constants.ShootingArc.kShootOnMoveLateralRelaxStartMps);
        int settleLoopsRequired = settleLoopsBase;
        if (Math.abs(tangentialVelocityMps) > lateralRelaxStart) {
            settleLoopsRequired = settleLoopsMin;
        }

        double absTurretErrorDeg = Math.abs(turretError);
        double aimToleranceDeg = Constants.ShootingArc.kTurretAimToleranceDeg;
        if (absTurretErrorDeg < (0.6 * aimToleranceDeg)) {
            settleLoopsRequired = Math.min(settleLoopsRequired, 1);
        }

        boolean turretAimed = turretSettledCounter >= settleLoopsRequired;
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

            SmartDashboard.putNumber(DASH + "Tower Tag ID", towerTagId);
            SmartDashboard.putString(DASH + "Tower Tag Info", tagInfo);
            SmartDashboard.putString(DASH + "Tag Alliance", tagAlliance);
            SmartDashboard.putBoolean(DASH + "In Shooting Zone", inZone);
            SmartDashboard.putNumber(DASH + "Distance (m)", predictedDistance);
            SmartDashboard.putNumber(DASH + "Distance Uncomp (m)", directDistance);
            SmartDashboard.putNumber(DASH + "Hood Distance Used (m)", hoodDistanceForLookup);
            SmartDashboard.putString(DASH + "Hood Distance Source", useDirectHoodAtClose ? "Direct" : "Predicted");

            SmartDashboard.putNumber(DASH + "Robot Vx Robot (mps)", robotRelativeVelocity.getX());
            SmartDashboard.putNumber(DASH + "Robot Vy Robot (mps)", robotRelativeVelocity.getY());
            SmartDashboard.putNumber(DASH + "Robot Vx Field (mps)", robotFieldVelocity.getX());
            SmartDashboard.putNumber(DASH + "Robot Vy Field (mps)", robotFieldVelocity.getY());
            SmartDashboard.putNumber(DASH + "Radial Velocity (mps)", radialVelocityMps);
            SmartDashboard.putNumber(DASH + "Tangential Velocity (mps)", tangentialVelocityMps);
            SmartDashboard.putNumber(DASH + "Radial Scale Active", radialVelocityMps > 0.0
                    ? Constants.ShootingArc.kShootOnMoveRadialCompScaleAway
                    : radialVelocityMps < 0.0
                            ? Constants.ShootingArc.kShootOnMoveRadialCompScaleToward
                            : Constants.ShootingArc.kShootOnMoveRadialCompScale);
            SmartDashboard.putString(DASH + "Radial Direction", radialVelocityMps > 0.0 ? "Away"
                    : radialVelocityMps < 0.0 ? "Toward" : "Neutral");
            SmartDashboard.putNumber(DASH + "Lead Gain", leadGain);
            SmartDashboard.putNumber(DASH + "Lead Gain X", leadGainX);
            SmartDashboard.putNumber(DASH + "Lead Gain Y", leadGainY);
            SmartDashboard.putNumber(DASH + "Wheel Circumference (m)", wheelCircumferenceMeters);
            SmartDashboard.putNumber(DASH + "Muzzle Speed Scale", muzzleSpeedScale);
            SmartDashboard.putNumber(DASH + "Latency Sec", extraLatencySec);
            SmartDashboard.putNumber(DASH + "Velocity Lookahead Sec", velocityLookaheadSec);
            SmartDashboard.putNumber(DASH + "Lead Iterations", leadIterations);
            SmartDashboard.putNumber(DASH + "Velocity LPF Alpha", alpha);
            SmartDashboard.putNumber(DASH + "Max Lead Meters", maxLeadMeters);
            SmartDashboard.putNumber(DASH + "Shooter Speed Estimate (mps)", shooterMps);
            SmartDashboard.putNumber(DASH + "Flight Time (s)", flightTimeSec);
            SmartDashboard.putNumber(DASH + "Flight Time Formula (s)", (predictedDistance / Math.max(0.1, shooterMps)) + extraLatencySec);
            SmartDashboard.putNumber(DASH + "Flight Time Table (s)", interpolate(Constants.ShootingArc.kShootOnMoveTofLookup, predictedDistance) + extraLatencySec);
            SmartDashboard.putNumber(DASH + "Lead Time Used (s)", leadTimeUsedSec);
            SmartDashboard.putNumber(DASH + "Lead Time Clamped (s)", Math.min(leadTimeUsedSec,
                    Math.max(0.0, Constants.ShootingArc.kShootOnMoveMaxPredictionSec)));
            SmartDashboard.putBoolean(DASH + "Invert Lead", invertLead);
            SmartDashboard.putNumber(DASH + "Lead Applied X (m)", predictedRobotPose.getX() - robotPose.getX());
            SmartDashboard.putNumber(DASH + "Lead Applied Y (m)", predictedRobotPose.getY() - robotPose.getY());
            SmartDashboard.putNumber(DASH + "Lead Applied Mag (m)",
                    Math.hypot(predictedRobotPose.getX() - robotPose.getX(),
                               predictedRobotPose.getY() - robotPose.getY()));
            SmartDashboard.putNumber(DASH + "Predicted Pose X (m)", predictedRobotPose.getX());
            SmartDashboard.putNumber(DASH + "Predicted Pose Y (m)", predictedRobotPose.getY());
            SmartDashboard.putNumber(DASH + "Predicted Heading (deg)", predictedRobotPose.getRotation().getDegrees());
            SmartDashboard.putNumber(DASH + "Predicted Distance (m)", predictedDistance);

            double leadDeltaDeg = targetTurretAngle - targetTurretAngleUncomp;

            SmartDashboard.putNumber(DASH + "Turret Target Uncomp (deg)", targetTurretAngleUncomp);
            SmartDashboard.putNumber(DASH + "Turret Target Comp (deg)", targetTurretAngle);
            SmartDashboard.putNumber(DASH + "Turret Lead Delta (deg)", leadDeltaDeg);
            SmartDashboard.putNumber(DASH + "Turret Target (deg)", targetTurretAngleRaw);
            SmartDashboard.putNumber(DASH + "Turret Target Slew Limited (deg)", targetTurretAngle);
            SmartDashboard.putNumber(DASH + "Turret Error (deg)", turretError);
            SmartDashboard.putBoolean(DASH + "Turret Aimed", turretAimed);
            SmartDashboard.putBoolean(DASH + "Turret Aimed Instant", turretAimedInstant);
            SmartDashboard.putNumber(DASH + "Turret Error Abs (deg)", absTurretErrorDeg);
            SmartDashboard.putNumber(DASH + "Turret Aim Tolerance (deg)", aimToleranceDeg);
            SmartDashboard.putNumber(DASH + "Turret Settled Counter", turretSettledCounter);
            SmartDashboard.putNumber(DASH + "Turret Miss Counter", turretMissCounter);
            SmartDashboard.putNumber(DASH + "Turret Settle Loops Required", settleLoopsRequired);
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
        shootOnMoveActive = false;
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
        int towerTagId = towerTagIdSupplier.getAsInt();
        Translation3d towerCenter = ShootingArcManager.getTowerCenter(drivetrain.getState().Pose, towerTagId);

        Translation2d turretPos2d = ShootingArcManager.getTurretFieldPosition(drivetrain.getState().Pose).toTranslation2d();
        Translation2d toTower = towerCenter.toTranslation2d().minus(turretPos2d);
        double fieldAngle = Math.atan2(toTower.getY(), toTower.getX());
        double targetAngle = Math.toDegrees(fieldAngle - drivetrain.getState().Pose.getRotation().getRadians());

        double targetRawRotations = targetAngle * TurretConstants.kRotationsPerDegree;
        double errorDeg = Math.abs((targetRawRotations - launcher.getTurretPosition())
                / TurretConstants.kRotationsPerDegree);
        return errorDeg < Constants.ShootingArc.kTurretAimToleranceDeg;
    }

    /** Returns {@code true} if the robot is currently firing. */
    public boolean isFiring() {
        return isFiring;
    }

    /** Linear interpolation helper for lookup tables of shape {{x,y}, ...}. */
    private static double interpolate(double[][] table, double x) {
        if (table == null || table.length == 0) {
            return 0.0;
        }
        if (x <= table[0][0]) {
            return table[0][1];
        }
        for (int i = 1; i < table.length; i++) {
            double x0 = table[i - 1][0];
            double y0 = table[i - 1][1];
            double x1 = table[i][0];
            double y1 = table[i][1];
            if (x <= x1) {
                double t = (x - x0) / Math.max(1e-9, (x1 - x0));
                return y0 + t * (y1 - y0);
            }
        }
        return table[table.length - 1][1];
    }
}
