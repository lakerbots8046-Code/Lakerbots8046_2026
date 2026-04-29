package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;

public class Constants {
    
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static boolean isLazerConnected;

    public static final double wheelBase = 22.75;
    public static final double trackWidth = 20.75;
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));


    public class TagConstants{
        //x and area of tag
        public static final Double[] tagTranslation = {-7.0,0.0};
        public static final Double[] tagPoseSecondLeg  = {-9.0, 0.0};
        public static final Double[] tagePoseAlgea = {-16.0, 0.0};
    }

    public static class SpindexerConstants {
        // Motor CAN IDs
        public static final int kSpindexerMotorID = 4;
        public static final int kFlappyWheelFeederMotorID = 5;
        public static final int kFeederMotorID = 6;
        public static double kSensorToMechanismRatio;
        public static double kSpindexerVelocity;

        // ── Spindexer velocity setpoints (RPS) ───────────────────────────────
        // Negative = intake direction for spindexer.
        // Tune kSpindexerIntakeVelocity first; adjust kSpindexerOuttakeVelocity as needed.
        public static final double kSpindexerIntakeVelocity  = -90.0; // RPS — intake direction
        public static final double kSpindexerOuttakeVelocity =  30.0; // RPS — outtake direction
        public static final double kSpindexerHoldVelocity    = -10.0; // RPS — slow hold

        // ── FlappyWheel (Stars) velocity setpoints (RPS) ─────────────────────
        // Negative = intake direction for flappy wheel.
        public static final double kFlappyWheelIntakeVelocity  = -10.0; // RPS — intake direction
        public static final double kFlappyWheelOuttakeVelocity =  10.0; // RPS — outtake direction

        // ── Feeder velocity setpoints (RPS) ──────────────────────────────────
        // Positive = intake direction for feeder.
        public static final double kFeederIntakeVelocity  =  90.0; // RPS — intake direction
        public static final double kFeederOuttakeVelocity = -30.0; // RPS — outtake direction
        public static final double kFeederHoldVelocity    =  10.0; // RPS — slow hold

        // SmartDashboard keys
        public static String kSmartDashboardPrefix;
        public static String kSpindexerVelocityKey;
        public static String kSpindexerCurrentKey;
        public static String kFeederVelocityKey;
        public static String kFeederCurrentKey;
        public static String kFeederTempKey;
        public static String kSpindexerTempKey;
        public static String kStatusKey;
        public static String kFlappyWheelVelocityKey;
        public static String kFlappyWheelCurrentKey;
        public static String kFlappyWheelTempKey;
    }


    public static class LauncherConstants {
        // Motor CAN IDs
        public static final int kLauncherMotorID = 8;
        public static final int kHoodMotorID = 9;
        public static final int kTurretMotorID = 7;

        // ── Hood physical constants ───────────────────────────────────────────
        /** Total gear reduction from motor to hood mechanism. */
        public static final double kHoodGearRatio = 93.25;
        /** Minimum motor position (rotations). */
        public static final double kHoodMinRotations = 0.0;
        /** Maximum motor position (rotations). */
        public static final double kHoodMaxRotations = 11.5;

        // Motor direction is inverted:
        // 0 rotations = 68° from horizontal (steepest), 11.5 rotations = 28° (flattest).
        /** Launch angle (degrees from horizontal) when motor is at kHoodMinRotations (0 rot). */
        public static final double kHoodMinAngleDeg = 68.0;   // 22° from vertical = steepest
        /** Launch angle (degrees from horizontal) when motor is at kHoodMaxRotations (11.5 rot). */
        public static final double kHoodMaxAngleDeg = 28.0;   // 62° from vertical = flattest

        /**
         * Converts a desired launch angle (degrees from horizontal) to motor rotations.
         * Motor is inverted: 0 rot = 68° (steep), 11.5 rot = 28° (flat).
         * Formula: rotations = (kHoodMinAngleDeg - angleDeg) / (kHoodMinAngleDeg - kHoodMaxAngleDeg) * kHoodMaxRotations
         */
        public static double hoodAngleToRotations(double angleDeg) {
            return (kHoodMinAngleDeg - angleDeg)
                    / (kHoodMinAngleDeg - kHoodMaxAngleDeg)
                    * kHoodMaxRotations;
        }
        /** Position tolerance for hood "at target" check (motor rotations). */
        public static final double kHoodPositionToleranceRotations = 0.6; // 0.2, 0.4

        /** Flywheel idle speed between shots (negative = shooting direction). */
        public static final double kFlywheelIdleRPS = -20.0;
        /** Enables flywheel idle behavior between shots. */
        public static final boolean kFlywheelIdleEnabled = true;

        public static double kSensorToMechanismRatio;
        public static double kHoodStowedPosition;
        public static double kHoodCollectPosition;
        public static double kHoodScoreHighPosition;
        public static double kHoodScoreLowPosition;
        public static double kLauncherIntakeVelocity;
        public static double kCollectOuttakeVelocity;
        public static double kCollectHoldVelocity;
        public static double kHoodPositionTolerance;
        public static String kSmartDashboardPrefix;
        public static String kCollectVelocityKey;
        public static String kCollectCurrentKey;
        public static String kCollectTempKey;
        public static String kStatusKey;
        public static double kTurretPositionTolerance;
        public static String kTurretPositionKey;
        public static String kTurretTargetKey;
        public static String kTurretErrorKey;
        public static String kTurretAtTargetKey;
        public static String kHoodPositionKey;
        public static String kHoodTargetKey;
        public static String kHoodErrorKey;
        public static String kHoodAtTargetKey;
        public static String kPivotCurrentKey;
        public static String kTurretTempKey;
        public static String kHoodTempKey;
    }


    public static class ClimberConstants {
        // Motor CAN ID
        public static final int kClimberMotorID = 10;

        // Gear ratio: motor rotations per climber mechanism rotation
        public static final double kSensorToMechanismRatio = 12.0;

        // Climber setpoints (mechanism rotations) // All positions x 5 for new 60:1 gear ratio.
        public static final double kClimberReverseSoftLimit = -13.25; // 2.65, -13.65
        public static final double kClimberReachSetpoint = -12.65; // 2.65
        public static final double kClimberPullDownSetpoint = 0.0; // 0.0
        public static final double kClimberYButtonSetpoint = -5; // 1.5, -7.5

        ;

        // Optional dashboard/config placeholders used elsewhere
        public static double kClimbVelocity;
        public static double kClimbHoldVelocity;
        public static String kSmartDashboardPrefix;
        public static String kClimbVelocityKey;
        public static String kClimbCurrentKey;
        public static String kClimbTempKey;
        public static String kStatusKey;
    }


// Note: Turret code is currently integrated into Launcher subsystem for better coordination between hood and turret aiming.
    public static class TurretConstants {
        // Motor CAN ID
        public static final int kTurretMotorID = 6;
        public static final String kCANBusName = "rio";
        
        // ── Gear Ratio ────────────────────────────────────────────────────────
        // Measured: 38.333... (115/3) motor rotations per one full mechanism rotation.
        // Source: physical gearbox measurement.
        public static final double kGearRatio = 115.0 / 3.0; // 38.333... motor rot / mechanism rot
        public static final double kRotationsPerDegree = kGearRatio / 360.0; // ≈ 0.10648 rot/deg

        // Speed Control
        public static final double kDefaultSpinSpeed = 0.3; // 30% power for testing
        public static final double kMaxSpinSpeed = 1.0; // Maximum allowed speed
        public static final double kMinSpinSpeed = -1.0; // Minimum allowed speed (reverse)
        
        // Position Control
        public static final double kPositionToleranceDegrees = 2.0; // Tolerance for angle checking, 2.0

        // ── Physical Rotation Limits (HARDWARE HARD STOPS — DO NOT EXCEED) ───
        //
        // ±200° mechanical range (±21.3 raw motor rotations).
        // EXCEEDING THESE LIMITS WILL DAMAGE THE TURRET.
        // Turret WRAPS at limits: 200° → -160° (360°-200°), -200° → +160°.
        public static final double kPhysicalLimitRotations = 22.0; // raw motor rotations (±200° exactly)
        public static final double kMinRotationDegrees = -200.0;
        public static final double kMaxRotationDegrees =  200.0;

        // Soft-stop buffer: turret is considered "near a limit" when within this many
        // degrees of the hard stop. Prevents the turret from slamming into the physical stop.
        public static final double kNearLimitBuffer = 2.0; // degrees of buffer before hard stop

        // Turret zero calibration offset in degrees; tune on robot if needed.
        public static final double kTurretZeroOffsetDegrees = 0.0;

        // ── Turret pivot offset from robot center ─────────────────────────────
        //
        // The turret rotation axis is NOT at the robot center.
        // Measured physically: 5 inches behind the robot center, centered left-right.
        //
        // This offset is used by ShootingArcManager to compute the turret's actual
        // field position for accurate angle and distance calculations, especially
        // at side angles where the offset shifts the turret laterally.
        //
        // Sign convention (robot-relative, WPILib):
        //   +X = robot forward,  -X = robot backward
        //   +Y = robot left,     -Y = robot right
        /** Turret pivot offset along robot X axis (meters). Negative = behind robot center. */
        public static final double kTurretOffsetX = -Units.inchesToMeters(5.0); // 5 in behind center
        /** Turret pivot offset along robot Y axis (meters). Zero = centered left-right. */
        public static final double kTurretOffsetY = 0.0;
        /** Turret pivot height above the floor (meters). Measured: 14 in from floor to pivot. */
        public static final double kTurretOffsetZ = Units.inchesToMeters(14.0); // 14 in = 0.3556 m

        // Wrapping threshold: trigger wrap if |target| exceeds this (degrees).
        // Matches physical limits: turret wraps +180° ↔ -180°.
        public static final double kWrapAroundThreshold = 185.0;
        
        // Wrapping tolerance: trigger wrap if shortest path > this many degrees
        public static final double kWrapToleranceDegrees = 5.0;
        
        // AprilTag Tracking
        public static final double kTrackingP = 20.0; // Proportional gain for tracking
        public static final double kTrackingI = 0.0;  // Integral gain
        public static final double kTrackingD = 0.001; // Derivative gain
        public static final double kTrackingToleranceDegrees = 3.0; // Tolerance for "locked in" state
        public static final double kMaxTrackingSpeed = 0.5; // Maximum speed during tracking (50%)
        public static final double kMinTrackingOutput = 0.05; // Minimum output to overcome friction
        
        // Wrap-around behavior
        public static final double kWrapAroundSpeed = 0.3; // Speed during wrap-around rotation
        public static final double kWrapAroundTargetOffset = 170.0; // Target angle after wrap (opposite side)
        
        // Launch from Tower Command duty cycle outputs (range: -1.0 to +1.0)
        // Temp test values — update to final values after on-robot testing
        public static final double spindexerDutyCycleOut = -0.6;  // Spindexer (CAN 4): temp -0.2 | final: -1.0
        public static final double starFeederDutyCycleOut = -0.1;  // StarFeeder (CAN 5): temp -0.2 | final: -1.0
        public static final double feederDutyCycleOut = 0.6;     // Feeder (CAN 6):     temp -0.2 | final: -0.5
        public static final double flywheelDutyCycleOut = -0.65;    //-0.75  // Launcher (CAN 8):   temp -0.2 | final: -1.0
        /**
         * Flywheel velocity (RPS) for launchFromTower shots — used by velocity closed-loop control.
         * Equivalent to flywheelDutyCycleOut (-0.65) × Kraken X60 free speed (100 RPS) = -65 RPS.
         * Negative = shooting direction. Tune on robot: increase magnitude if shots land short,
         * decrease if shots overshoot.
         */
        public static final double kFlywheelTowerRPS = -45.0; // 55 , 52 , 45
        // Hood position (motor rotations) during Launch from Tower.
        // Range: 0.0 rot = 68° from horizontal (steepest) → 11.5 rot = 28° from horizontal (flattest).
        // TUNE ON ROBOT: increase toward 11.5 to flatten the shot; decrease toward 0.0 to steepen it.
        public static final double hoodTowerPosition = 0.0; // TESTED on robot 0.5

        public static final String kSmartDashboardPrefix = "Turret/";
        public static final String kStartSpinKey = "Start Spin";
        public static final String kStopSpinKey = "Stop Spin";
        public static final String kSpinSpeedKey = "Spin Speed";
        public static final String kIsSpinningKey = "Is Spinning";
        public static final String kCurrentSpeedKey = "Current Speed";
        public static final String kMotorCurrentKey = "Motor Current (A)";
        public static final String kMotorTempKey = "Motor Temp (C)";
        public static final String kStatusKey = "Status";
        public static final String kTargetAngleKey = "Target Angle";
        public static final String kCurrentAngleKey = "Current Angle";
        public static final String kAtLimitKey = "At Limit";
        public static final String kTrackingLockedKey = "Tracking Locked";
        public static final String kTrackingErrorKey = "Tracking Error";
        public static double kSensorToMechanismRatio;
       
    }

    public static class IntakeConstants {
        // Motor CAN IDs
        public static final int kIntakeRollersMotorID = 3;
        public static final int kIntakePivotMotorID = 2;
        public static final String kCANBusName = "rio";
        
        // Intake Pivot Position Setpoints (in rotations)
        public static final double kPivotStowedPosition = 0.0;           // Fully retracted/stowed
        public static final double kPivotCollectPosition = 0.25;         // Extended for collecting game pieces
       // public static final double kPivotScoreHighPosition = 0.15;       // Position for high scoring
       // public static final double kPivotScoreLowPosition = 0.05;        // Position for low scoring
        public static final double kPivotDeployCollectPosition = -1.33   ; // Deploy position for intake_deploy_collect -1.36 to -1.25 to 1.304 to -1.36, 1.355, *** -1.365 ***
        public static final double kPivotHardDownLimit = -1.4;          // Physical hard stop limit (DO NOT EXCEED)
        public static final double kPivotHomePosition = 0.0;            // Retract/home position after collecting
        public static final double kPivotDumpPosition = -0.75;          // 
        // Intake Collection Velocities (in rotations per second
        public static final double kCollectIntakeVelocity = 30.0;   // Speed when intaking game pieces
        public static final double kCollectOuttakeVelocity = -20.0; // Speed when ejecting game pieces
        public static final double kCollectHoldVelocity = 5.0;      // Low speed to hold game piece
        public static final double kCollectStopVelocity = 0.0;      // Stop the collection motor
        
        // Position Control Tolerance
        public static final double kPivotPositionTolerance = 0.02;  // Tolerance in rotations (±0.02)
        
        // Gear Ratio
        public static final double kSensorToMechanismRatio = 12.8;  // 12.8 rotor rotations per mechanism rotation
        
        // SmartDashboard Keys
        public static final String kSmartDashboardPrefix = "Intake/";
        public static final String kPivotPositionKey = "Pivot Position";
        public static final String kPivotTargetKey = "Pivot Target";
        public static final String kPivotErrorKey = "Pivot Error";
        public static final String kPivotAtTargetKey = "Pivot At Target";
        public static final String kCollectVelocityKey = "Collect Velocity";
        public static final String kCollectCurrentKey = "Collect Current (A)";
        public static final String kPivotCurrentKey = "Pivot Current (A)";
        public static final String kPivotTempKey = "Pivot Temp (C)";
        public static final String kCollectTempKey = "Collect Temp (C)";
        public static final String kStatusKey = "Status";
        public static final double kIntakeVoltage = -9.0;
        // ── Roller velocity setpoints (RPS) ──────────────────────────────────
        public static final double kRollersIntakeVelocity = -100.0; // RPS — intake direction
        public static final double kRollersOuttakeVelocity = -kRollersIntakeVelocity; // RPS — outtake direction (equal/opposite of intake)
        public static final double kRollersHoldVelocity = 5.0;      // RPS — slow hold

        // ── Roller current limits ─────────────────────────────────────────────
        public static final double kRollersStatorCurrentLimit = 40.0; // Amps — stall protection
        public static final double kRollersSupplyCurrentLimit = 40.0; // Amps — breaker protection
        public static String kRollersVelocityKey;
        public static String kRollersCurrentKey;
        public static String kRollersTempKey;

        // Manual pivot control speed (duty cycle, 0.0–1.0)
        // X button lowers at -kPivotManualSpeed, Y button raises at +kPivotManualSpeed
        public static final double kPivotManualSpeed = 0.1; // 10% — reduced to prevent slamming into limits

        // POV bump factor for adjusting intake pivot position (in rotations)
        // POV Up (0) increases position, POV Down (180) decreases position
        public static final double kPivotBumpFactor = 0.01;
    }
    
    /** Constants for FeedFromCenter command behavior. */
    public static class FeedFromCenter {

        // ── Hood position by distance ─────────────────────────────────────────
        /**
         * Distance threshold (meters) from hub/feed target center used to switch
         * feed-from-center hood setpoints.
         * <p>
         * If distance is below this threshold, use {@link #kHoodPositionNear}.
         * If distance is greater than or equal to this threshold, use
         * {@link #kHoodPositionFar}.
         */
        public static final double kHoodDistanceThresholdMeters = 7.0;

        /**
         * Hood position (motor rotations) for feed-from-center shots when
         * distance is under {@link #kHoodDistanceThresholdMeters}.
         * 0.0 rot = 68° from horizontal (steepest), 11.5 rot = 28° (flattest).
         */
        public static final double kHoodFeedPosition = 5.5;

        /**
         * Hood position (motor rotations) for feed-from-center shots when
         * distance is greater than or equal to
         * {@link #kHoodDistanceThresholdMeters}.
         * 0.0 rot = 68° from horizontal (steepest), 11.5 rot = 28° (flattest).
         */
        public static final double kHoodPositionFar = 7.0;
        /** Extra hood flattening (rotations) applied only for long feed-from-center shots. */
        public static final double kFarHoodExtraRotations = 0.0;
        /** Distance threshold (m) to apply far-only hood extra rotations. */
        public static final double kFarHoodDistanceMeters = 3.5;

        /** Fixed offset (rotations) added to ALL FeedFromCenter hood positions. */
        public static final double kFeedHoodOffsetRotations = 2.0;
        /** Fixed offset (RPS) added to ALL FeedFromCenter launcher velocity targets. */
        public static final double kFeedVelocityOffsetRps = 0.0; // 20.0

        // ── Target depth offset ───────────────────────────────────────────────
        /**
         * Distance (meters) to offset the aiming target BEHIND the tag-pair midpoint,
         * in the direction from the robot toward the midpoint.
         * "A few feet behind" the midpoint = 0.6096 m (2 ft). TUNE ON ROBOT.
         */
        public static final double kFeedTargetDepthMeters = 2.0; // 1.5, 2.0

        /**
         * Turret aim tolerance (degrees) for FeedFromCenter firing readiness.
         * Set to 2x ShootOnMove/ShootingArc tolerance so FeedFromCenter can fire
         * with a wider acceptable turret error band.
         */
        public static final double kTurretAimToleranceDeg = 50.0;

        // ── Velocity compensation ─────────────────────────────────────────────
        /**
         * Ball exit speed per flywheel RPS (meters per second per RPS).
         *
         * <p>Used to estimate ball flight time for velocity lead compensation:
         * {@code exitSpeed = kBallExitSpeedPerRPS × |targetRPS|}
         * {@code flightTime = distance / exitSpeed}
         *
         * <p>Derived from Kraken X60 launcher wheel geometry (same value used in
         * ShootingArcManager physics comments). TUNE ON ROBOT if shots consistently
         * lead or lag the target while the robot is moving.
         */
        public static final double kBallExitSpeedPerRPS = 0.196; // m/s per RPS

        /**
         * Feed-from-center-specific motion lead gain.
         *
         * <p>Scales the velocity-compensation lead offset used by
         * FeedFromCenterCommand:
         * {@code virtualTarget = target - (robotVelocity * flightTime * gain)}.
         *
         * <p>Increase above 1.0 if shots still trail robot motion.
         * Decrease below 1.0 if shots over-lead.
         */
        public static final double kFeedFromCenterMotionCompensationGain = 2.0; // 1.2, 2, 3, 5

        /**
         * Enables translational motion compensation while aiming fixed FeedFromCenter targets.
         * When true, turret aims at a velocity-lead virtual target.
         * When false, turret aims at the exact fixed ground coordinate.
         */
        public static final boolean kUseMotionCompensation = true;

        /**
         * Feed-from-center flywheel RPS scale factor.
         *
         * <p>Multiplies the calculated launcher RPS only for FeedFromCenter shots.
         * Use values above 1.0 to increase shot energy when balls land short.
         */
        public static final double kFeedFromCenterRpsScale = 1.2; // 1.2, 1.6
        /** Launcher at-speed tolerance (RPS) specifically for FeedFromCenter readiness gating. */
        public static final double kLauncherVelocityToleranceRps = 10.0;

        // ── Feed sequencing / readiness timing ────────────────────────────────
        /** Time flywheel must remain in tolerance before feed starts (seconds). */
        public static final double kReadyStableTimeSec = 0.0;
        /** Max time allowed waiting for full ready conditions (seconds). */
        public static final double kReadyTimeoutSec = 4.0;
        /** Max time allowed while feed motors are running (seconds). */
        public static final double kFeedingTimeoutSec = 2.0;

        // ── Feed motor velocity targets for LaunchSequenceOne (RPS) ───────────
        /** Spindexer feed velocity (CAN 4), negative = intake/feed direction. */
        public static final double kFeedSpindexerRPS = -90.0;
        /** Flappy wheel feed velocity (CAN 5), positive = intake/feed direction. */
        public static final double kFeedFlappyWheelRPS = 90.0;
        /** Feeder feed velocity (CAN 6), positive = intake/feed direction. */
        public static final double kFeedFeederRPS = 90.0;

        // ── Optional recovery pulse on timeout ────────────────────────────────
        /** Enable one reverse pulse retry if feed phase times out. */
        public static final boolean kEnableRecoveryPulse = true;
        /** Reverse pulse duration before retrying feed (seconds). */
        public static final double kRecoveryPulseDurationSec = 0.10;
        /** Reverse pulse speed multiplier applied to feed speeds (0..1). */
        public static final double kRecoveryPulseScale = 0.35;

        // ── Red alliance feed-station tag pairs ───────────────────────────────
        /** Red alliance feed station — one side (tags 1 and 3). */
        public static final int[] kRedPairA = {1, 3};
        /** Red alliance feed station — other side (tags 4 and 6). */
        public static final int[] kRedPairB = {4, 6};

        // ── Blue alliance feed-station tag pairs ──────────────────────────────
        /** Blue alliance feed station — one side (tags 17 and 19). */
        public static final int[] kBluePairA = {17, 19};
        /** Blue alliance feed station — other side (tags 20 and 22). */
        public static final int[] kBluePairB = {20, 22};
    }

    /** Constants for shooting-arc and shoot-on-move behavior. */
    public static class ShootingArc {

        // ── Primary tower tag IDs (center of tower opening) ──────────────────
        public static final int kRedPrimaryTagId  = 10;  // Red  tower: x=12.505, y=4.021
        public static final int kBluePrimaryTagId = 20;  // Blue tower: x=5.215,  y=4.021

        // ── All tower tag ID arrays ───────────────────────────────────────────
        public static final int[] kRedTowerTagIds  = {2, 3, 4, 5, 8, 9, 10, 11};
        public static final int[] kBlueTowerTagIds = {18, 19, 20, 21, 24, 25, 26, 27};

        // ── Shooting arc geometry ─────────────────────────────────────────────
        /** Preferred radius of the shooting arc from the tower center (meters).
         *  Reduced from 3.0 to 2.0 — one meter closer to the tower. */
        public static final double kPreferredShootingDistance = 2.0;
        /** Minimum valid shooting distance (meters). */
        public static final double kMinShootingDistance = 0.0;
        /** Maximum valid shooting distance (meters). */
        public static final double kMaxShootingDistance = 6.0;

        // ── Tolerances ────────────────────────────────────────────────────────
        /** Position tolerance for "arrived at arc" check (meters). */
        public static final double kPositionToleranceMeters = 0.20;
        /** Heading tolerance for "facing tower" check (degrees). */
        public static final double kRotationToleranceDeg = 5.0;
        /** Turret aim tolerance before firing is allowed (degrees). */
        public static final double kTurretAimToleranceDeg = 300.0; // 4.0, 18.0
        /** Launcher velocity tolerance before firing is allowed (RPS). */
        public static final double kLauncherVelocityTolerance = 5.0;

        // ── Shoot-on-move lead compensation constants ─────────────────────────
        /** Global multiplier magnitude for translational lead compensation (dimensionless). Direction is handled in ShootOnMoveCommand. */
        public static final double kShootOnMoveLeadCompGain = 1.0; // 1.0
        /** Shooter wheel circumference used to estimate projectile speed from RPS. */
        public static final double kShootOnMoveWheelCircumferenceMeters = 0.319; // ~4" wheel
        /** Empirical scale factor from wheel surface speed to effective muzzle speed. */
        public static final double kShootOnMoveMuzzleSpeedScale = 0.75;
        /** Additional control/system latency added to projectile flight time (seconds). */
        public static final double kShootOnMoveExtraLatencySec = 0.20;
        /** Number of fixed-point iterations for lead/intercept solve (>=1). */
        public static final int kShootOnMoveLeadSolveIterations = 3;
        /** LPF alpha for field-relative velocity used in lead solve (0..1). */
        public static final double kShootOnMoveVelocityLpfAlpha = 0.25;

        /** Axis-specific X gain magnitude for translational lead compensation (field X). Direction is handled in ShootOnMoveCommand. */
        public static final double kShootOnMoveLeadCompGainX = 1.0; // 1.0
        /** Axis-specific Y gain magnitude for translational lead compensation (field Y). Direction is handled in ShootOnMoveCommand. */
        public static final double kShootOnMoveLeadCompGainY = 1.0; // 1.0
        /** Extra lookahead time used for velocity lead application (seconds). */
        public static final double kShootOnMoveVelocityLookaheadSec = 0.24;
        /** Maximum magnitude of applied lead vector (meters). */
        public static final double kShootOnMoveMaxLeadMeters = 2.5;

        /** Max allowed future prediction horizon used by shoot-on-move (seconds). */
        public static final double kShootOnMoveMaxPredictionSec = 0.60;
        /** Max commanded turret target change rate while tracking (deg/sec). */
        public static final double kShootOnMoveTurretSlewRateDegPerSec = 180.0;
        /** Consecutive execute loops turret must remain within tolerance before firing. */
        public static final int kShootOnMoveTurretSettleLoops = 4;

        /** Base scale on radial (toward/away target) velocity contribution to prediction horizon. */
        public static final double kShootOnMoveRadialCompScale = 0.45;
        /** Radial scale used when robot is moving away from target (radial velocity > 0). */
        public static final double kShootOnMoveRadialCompScaleAway = 0.65;
        /** Radial scale used when robot is moving toward target (radial velocity < 0). */
        public static final double kShootOnMoveRadialCompScaleToward = 0.25;
        /** Scale on tangential (sideways) velocity contribution to prediction horizon. */
        public static final double kShootOnMoveTangentialCompScale = 1.30;

        /** Distance threshold (m) below which hood uses direct (uncompensated) distance for stability. */
        public static final double kShootOnMoveHoodDirectDistanceThresholdMeters = 2.5;
        /** If true, use direct distance for hood below threshold to avoid close-range hood jitter. */
        public static final boolean kShootOnMoveUseDirectDistanceForHoodAtCloseRange = true;

        /** Lateral speed (m/s) above which settle-loop requirement can be relaxed. */
        public static final double kShootOnMoveLateralRelaxStartMps = 1.0;
        /** Minimum settle loops allowed when moving laterally fast. */
        public static final int kShootOnMoveTurretSettleLoopsMin = 2;

        /** Enables distance->time-of-flight lookup for shoot-on-move lead solve. */
        public static final boolean kShootOnMoveUseTofTable = true;
        /**
         * Distance (m) -> time-of-flight (s) table for translational lead.
         * Tune on-robot from observed moving-shot error.
         */
        public static final double[][] kShootOnMoveTofLookup = {
            {1.0, 0.313}, // 0.16
            {1.5, 0.369}, // 0.20
            {2.0, 0.426}, // 0.24
            {2.5, 0.449}, // 0.28
            {3.0, 0.473}, // 0.33
            {3.5, 0.502}, // 0.38
            {4.0, 0.538}, // 0.44
            {4.5, 0.562}, // 0.50
            {5.0, 0.548}  // 0.57
        };
        // ── Shoot-on-move drivetrain gating ───────────────────────────────────
        /** Max drivetrain translation speed while ShootOnMove is active (m/s). */
        public static final double kShootOnMoveDriveMaxSpeedMps = 2.0; // 1.0
        /** Joystick deadband for direction gating while ShootOnMove is active (unitless 0..1). */
        public static final double kShootOnMoveDriveDirectionDeadband = 0.10;

        // ── Arc sliding ───────────────────────────────────────────────────────
        /** Speed at which the robot slides along the arc (m/s).
         *  TESTING VALUE: 0.5 m/s — increase to 1.5 m/s once behavior is confirmed. */
        public static final double kArcSlideSpeed = 0.5;

        // ── Rotation control (robot faces tower) ──────────────────────────────
        /** Max rotation rate while facing the tower (rad/s).
         *  TESTING VALUE: 0.75 rad/s — increase to 2.5 rad/s once behavior is confirmed. */
        public static final double kMaxArcRotationRate = 0.75;
        /** Proportional gain for tower-facing rotation PID. */
        public static final double kArcRotationP = 3.5;

        // ── PathPlanner constraints for Phase 1 (drive to arc) ────────────────
        // Separate from Vision.kPathPlannerMaxVelocity so arc approach can be
        // tuned independently. TESTING VALUES — increase once behavior is confirmed.
        /** Max linear velocity during PathPlanner drive-to-arc (m/s). */
        public static final double kApproachMaxVelocity = 1.0;
        /** Max linear acceleration during PathPlanner drive-to-arc (m/s²). */
        public static final double kApproachMaxAcceleration = 0.75;
        /** Max angular velocity during PathPlanner drive-to-arc (rad/s). */
        public static final double kApproachMaxAngularVelocity = Math.PI / 2.0;
        /** Max angular acceleration during PathPlanner drive-to-arc (rad/s²). */
        public static final double kApproachMaxAngularAcceleration = Math.PI / 2.0;

        // Launcher velocity lookup: {distance_m, velocity_RPS}
        public static final double[][] kLauncherRPSLookup = {
            {1.0,  -37.0},    // -36.0, 38.0      // was -50.0, 43.0, 42.0
            {1.5,  -37.0},    // -36.0, 38.0      // was -50.5, 43.5, 42.0
            {2.0,  -37.0},    // -36.0, 38.0      // was -53.0, 46.0, 42.0, 40
            {2.5,  -42.0},    // -38.0, 42.0      // was -55.5, 48.5, 47.5, 43, 41.5, 38 , 41.5
            {3.0,  -46.0},    // -42.5, 46.5      // was -58.5, 51.5, 50.0, 45, 43
           // {3.175, -51.0},               // added intermediate point at 3.175 m (Tower Shot) 49.0, 51.0
            {3.5,  -48.5},    // -45.0, 49.0      // was -61.5, 54.5, 48.5, 46.5, 46.0
            {4.0,  -49.5},    // -46.0, 50.0      // was -74.5, 67.5, 55.5, 53.5, 48
            {4.5,  -52.0},    // -49.0, 53.0      // was -80.5, 73.5, 56.0, 54.0
            {5.0,  -60.0}     // -58.0, 60.0      // was -58, 60
        };

        // Hood angle lookup: {distance_m, hood_motor_rotations}
        public static final double[][] kHoodAngleLookup = {
            {1.0,  0.0},    // 0.0    // extrapolated — steepest (68° from horizontal)
            {1.5,  0.0},    // 0.0    // extrapolated
            {2.0,  0.0},    // 0.0    // TESTED: hood position 0
            {2.5,  0.0},    // 0.0    // TESTED: hood position 0
            {3.0,  0.5},    // 0.5    // TESTED: hood position 0 0  
            {3.5,  1.0},    // 1.0    // TESTED: hood position 1 1, 2              0.75   , 2.25
            {4.0,  2.5},    // 2.5    // extrapolated (+1 rot per 0.5m)1           2.0
            {4.5,  3.0},    // 3.0    // extrapolated (+1 rot per 0.5m) 2  3.5
            {5.0,  3.875}   // 3.875  // was 4.0
        };

        // Elastic/SmartDashboard tunable table keys (flattened: [x0,y0,x1,y1,...])
        public static final String kLauncherRpsLookupDashboardKey = "ShootingArc/LauncherRPSLookupFlat";
        public static final String kHoodAngleLookupDashboardKey = "ShootingArc/HoodAngleLookupFlat";
        // Optional string fallbacks for dashboards that only edit text fields.
        public static final String kLauncherRpsLookupDashboardStringKey = "ShootingArc/LauncherRPSLookupCSV";
        public static final String kHoodAngleLookupDashboardStringKey = "ShootingArc/HoodAngleLookupCSV";
    }
    public static class Vision {
        // Camera names — must match exactly what is configured in PhotonVision
        public static final String kCameraNameBF = "CAM_BF";      // Back Facing camera
        public static final String kCameraNameFF = "CAM_FF";      // Front Facing camera
        public static final String kCameraNameLeft = "CAM_Left";  // Left-facing camera (2nd Orange Pi)
        public static final String kCameraNameRight = "CAM_Right";// Right-facing camera (2nd Orange Pi)

        // Camera stream URLs for dashboard viewing (Elastic, etc.)
        // Using PhotonVision coprocessor IP address (10.80.46.11)
        // Ports are assigned by PhotonVision — check http://photonvision.local:5800 for exact ports
        // CAM_BF stream port and CAM_FF stream port (update if streams don't load in Elastic)
        public static final String kCameraStreamBF = "http://10.80.46.11:1184/stream.mjpg";
        public static final String kCameraStreamFF = "http://10.80.46.11:1182/stream.mjpg";
        public static final String kCameraStreamLeft = "http://10.80.46.12:1182/stream.mjpg";
        public static final String kCameraStreamRight = "http://10.80.46.12:1184/stream.mjpg";

        // Transform from robot center to Back Facing camera
        // Measured position: X=-12.955" (back), Y=-6.998" (right side), Z=10.711" (from floor)
        // Rotation: Roll=0°, Pitch=+10° (10° from vertical = slightly toward ceiling),
        //   Yaw=187.502° (180° facing backward + 7.502° inward toward left/center)
        public static final Transform3d kRobotToCamBF =
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(-40.0), // Backward (negative X = behind robot center) // -12.941
                                Units.inchesToMeters(-7.0625),  // Right side (negative Y = right in WPILib)
                                Units.inchesToMeters(10.612)   // Up (positive Z, measured from floor)
                        ),
                        new Rotation3d(
                                Units.degreesToRadians(0),       // Roll
                                Units.degreesToRadians(25),      // Pitch: +10° = slightly toward ceiling (10° from vertical/forward)
                                Units.degreesToRadians(180.0))); // Yaw: 180° facing back + 7.502° toed inward toward left

        // Transform from robot center to Front Facing camera
        // Measured position: X=-7.069" (back), Y=+11.75" (left side), Z=20.4" (from ground)
        // Rotation: Roll=0°, Pitch=+22° (camera leans back 22° from vertical, lens tilts upward)
        //
        // ── Yaw sign convention (viewed from above) ───────────────────────────
        // This camera is on the LEFT side of the robot (+Y).
        //   Yaw =   0 deg          -> faces straight forward
        //   Yaw = +2 to +10 deg    -> faces slightly LEFT  (away from center) -- use if toed outward
        //   Yaw = -2 to -10 deg    -> faces slightly RIGHT (toward center)    -- use if toed inward
        //   (negative yaw can also be written as 350-358 deg)
        //
        // TUNE: place a tag directly in front of the robot. If PhotonVision reports
        // a non-zero yaw when the tag is centered in the image, adjust this value
        // until the reported yaw reads ~0 deg for a perfectly centered tag.
        public static final Transform3d kRobotToCamFF =
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(30),   // Backward (negative X = behind robot center) // -7.069, 15, 30
                                Units.inchesToMeters(11.75),    // Left side (positive Y = left in WPILib)
                                Units.inchesToMeters(20.4)      // Up (positive Z, measured from ground)
                        ),
                        new Rotation3d(
                                Units.degreesToRadians(0),      // Roll
                                Units.degreesToRadians(22),     // Pitch: +22° = lens tilts up (camera leans back 22° from forward)
                                Units.degreesToRadians(0)));    // Yaw: +2 deg = slightly away from center (LEFT of forward)
                                                                // Camera is on the left side and toed outward -- positive yaw is correct

        // Transform from robot center to Left-facing camera (second Orange Pi)
        // Measured position: X=-1.938" (back), Y=+13.746" (left), Z=9.92" (from ground)
        // Rotation: Roll=0°, Pitch=+20° (up from horizontal), Yaw=+90° (left of forward)
        public static final Transform3d kRobotToCamLeft =
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(-1.938),   // Backward (negative X)
                                Units.inchesToMeters(40.0),   // Left (positive Y) 13.746, 40
                                Units.inchesToMeters(9.92)      // Up (positive Z)
                        ),
                        new Rotation3d(
                                Units.degreesToRadians(0),      // Roll
                                Units.degreesToRadians(20),     // Pitch
                                Units.degreesToRadians(90)));   // Yaw: left-facing

        // Transform from robot center to Right-facing camera (second Orange Pi)
        // Measured position: X=-3.0625" (back), Y=-13.746" (right), Z=9.571" (from ground)
        // Rotation: Roll=0°, Pitch=+20° (up from horizontal), Yaw=-90° (right of forward)
        public static final Transform3d kRobotToCamRight =
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(-3.0625),  // Backward (negative X)
                                Units.inchesToMeters(-40.0),  // Right (negative Y)//-13.746?, -50
                                Units.inchesToMeters(9.571)     // Up (positive Z)
                        ),
                        new Rotation3d(
                                Units.degreesToRadians(0),      // Roll
                                Units.degreesToRadians(20),     // Pitch
                                Units.degreesToRadians(-90)));  // Yaw: right-facing

        // The layout of the AprilTags on the field
        // Using custom 2026 Rebuilt field layout loaded from JSON file
        // File location: src/main/deploy/2026-rebuilt.json
        public static final AprilTagFieldLayout kTagLayout = loadCustomLayout();
        
        /**
         * Loads the custom 2026 Rebuilt AprilTag field layout from JSON file.
         * Falls back to kDefaultField if loading fails.
         */
        private static AprilTagFieldLayout loadCustomLayout() {
            try {
                return new AprilTagFieldLayout(
                    Filesystem.getDeployDirectory().toPath()
                        .resolve("2026-rebuilt.json")
                );
            } catch (IOException e) {
                System.err.println("Failed to load 2026 Rebuilt AprilTag layout! Using default.");
                System.err.println("Error: " + e.getMessage());
                e.printStackTrace();
                return AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            }
        }

        // The standard deviations of our vision estimated poses, which affect correction rate
        // Lower values = trust vision more, higher values = trust odometry more
        // Single tag is less reliable, so higher standard deviation
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        // Multiple tags are more reliable, so lower standard deviation
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        /**
         * Explicit std dev multiplier applied to the BF (hub-facing) camera when it
         * sees a tower tag during ShootFromPointCommand.
         *
         * <p>A value of 0.5 means the BF camera's std devs are halved (50% tighter)
         * compared to the distance-scaled base, giving it higher weight in the
         * drivetrain Kalman filter and therefore more influence on the pose estimate
         * used for turret aiming.
         *
         * <p>Tune toward 1.0 to reduce the priority boost; toward 0.0 for maximum
         * trust in the hub-facing camera. 0.5 is a safe starting value.
         */
        public static final double kHubCameraStdDevBonus = 0.5;

        /**
         * Distance-based scaling factor applied to vision standard deviations.
         *
         * <p>Standard deviations are multiplied by {@code (1 + avgDist² × kDistanceScaleFactor)},
         * so a camera seeing tags at 1 m gets tighter (more trusted) std devs than one
         * seeing tags at 3 m. The camera with the closer tag therefore has higher weight
         * in the drivetrain Kalman filter automatically.
         *
         * <p>At the default value of 0.1:
         * <ul>
         *   <li>1 m → ×1.1  (10% looser than base)</li>
         *   <li>2 m → ×1.4  (40% looser)</li>
         *   <li>3 m → ×1.9  (90% looser)</li>
         *   <li>4 m → ×2.6  (160% looser)</li>
         * </ul>
         * Increase to trust close tags even more aggressively; decrease to flatten the curve.
         */
        public static final double kDistanceScaleFactor = 0.1;

        /**
         * AprilTag IDs to ignore for vision pose fusion.
         *
         * <p>If any of these IDs are used in a camera's pose estimate, that full
         * measurement is rejected before being added to the drivetrain pose estimator.
         * Useful for temporarily excluding problematic tags during bring-up/tuning.
         */
        public static final int[] kIgnoredPoseTagIds = {};

        /**
         * Maximum acceptable pose ambiguity for a single-tag detection (range 0.0–1.0).
         *
         * <p>PhotonVision's {@code getPoseAmbiguity()} returns a ratio of how similar the
         * two possible 3-D poses are for a single tag. A value close to 0 means one pose
         * is clearly better (trustworthy); a value close to 1 means both poses are nearly
         * identical (untrustworthy — the estimator cannot tell which is correct).
         *
         * <p>Any vision measurement where a target reports ambiguity strictly above this
         * threshold is discarded before being fed into the drivetrain Kalman filter.
         * Multi-tag PnP results return {@code -1} and are always accepted.
         *
         * <p>0.3 is the widely-recommended FRC threshold (PhotonVision docs).
         */
        public static final double kMaxAmbiguity = 0.2;

        /**
         * Maximum allowed age (seconds) of a vision measurement when fusing into odometry.
         * Older/stale frames are rejected to avoid delayed pose snaps.
         */
        public static final double kMaxVisionMeasurementAgeSec = 0.25;

        /**
         * Maximum allowed XY distance (meters) between current odometry pose and incoming
         * vision pose before rejecting as an outlier.
         */
        public static final double kMaxVisionXYJumpMeters = 1.25;

        /**
         * Maximum allowed heading delta (degrees) between current odometry pose and incoming
         * vision pose before rejecting as an outlier.
         */
        public static final double kMaxVisionHeadingJumpDeg = 35.0;
        
        // AprilTag driving parameters
        public static final double kTargetDistanceMeters = 1.524; // Target distance from tag (5 feet = 1.524 meters)
        public static final double kPositionToleranceMeters = 0.05; // 5cm tolerance
        public static final double kRotationToleranceDegrees = 2.0; // 2 degree tolerance
        
        // PID constants for driving to AprilTag
        public static final double kDriveP = 2.0; // Proportional gain for forward/backward
        public static final double kDriveI = 0.0; // Integral gain
        public static final double kDriveD = 0.1; // Derivative gain
        
        public static final double kStrafeP = 2.0; // Proportional gain for left/right
        public static final double kStrafeI = 0.0;
        public static final double kStrafeD = 0.1;
        
        public static final double kRotationP = 3.0; // Proportional gain for rotation
        public static final double kRotationI = 0.0;
        public static final double kRotationD = 0.1;
        
        // Maximum speeds when driving to tag (as fraction of max speed)
        public static final double kMaxDriveSpeed = 0.5;         // 50% of max speed (PathPlanner / DriveToAprilTag)
        public static final double kMaxStrafeSpeed = 0.5;
        public static final double kMaxRotationSpeed = 0.3;      // 30% of max rotation

        // Slower speeds specifically for vision centering (CenterOnAprilTagCommand)
        // Reduced for safety during initial testing — increase once behavior is confirmed correct
        public static final double kMaxCenteringDriveSpeed = 0.2;  // 20% of max speed
        public static final double kMaxCenteringStrafe = 0.2;      // 20% of max speed
        
        // PathPlanner configuration for driving to AprilTags
        public static final double kPathPlannerMaxVelocity = 3.0; // meters per second
        public static final double kPathPlannerMaxAcceleration = 2.0; // meters per second squared
        public static final double kPathPlannerMaxAngularVelocity = Math.PI; // radians per second
        public static final double kPathPlannerMaxAngularAcceleration = Math.PI; // radians per second squared
    }
    
}
