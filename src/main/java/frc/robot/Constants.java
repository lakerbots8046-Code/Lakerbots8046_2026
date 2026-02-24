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
        public static double kSpindexerOuttakeVelocity;
        public static double kSpindexerIntakeVelocity;
        public static double kSpindexerHoldVelocity;
        public static String kSmartDashboardPrefix;
        public static String kSpindexerVelocityKey;
        public static String kSpindexerCurrentKey;
        public static String kFeederVelocityKey;
        public static String kFeederCurrentKey;
        public static String kFeederTempKey;
        public static String kSpindexerTempKey;
        public static String kStatusKey;
        public static double kFeederIntakeVelocity;
        public static double kFeederOuttakeVelocity;
        public static double kFeederHoldVelocity;
        public static String kFlappyWheelVelocityKey;
        public static String kFlappyWheelCurrentKey;
        public static String kFlappyWheelTempKey;
    }


    public static class LauncherConstants {
        // Motor CAN IDs
        public static final int kLauncherMotorID = 8;
        public static final int kHoodMotorID = 9;
        public static final int kTurretMotorID = 7;
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
        // Motor CAN IDs
        public static final int kClimberMotorID = 10;
        public static double kSensorToMechanismRatio;
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
        public static final double kPositionToleranceDegrees = 2.0; // Tolerance for angle checking

        // ── Physical Rotation Limits (HARDWARE HARD STOPS — DO NOT EXCEED) ───
        //
        // Tuner X reports ±18 raw motor rotations at the physical hard stops.
        // Converted to mechanism degrees:
        //   18 motor rotations / kRotationsPerDegree
        //   = 18 * 360 / (115/3)
        //   = 18 * 1080 / 115
        //   = 19440 / 115
        //   ≈ ±169.04°
        //
        // EXCEEDING THESE LIMITS WILL BREAK THE TURRET.
        public static final double kPhysicalLimitRotations = 18.0; // raw motor rotations (from Tuner X)
        public static final double kMinRotationDegrees = -(kPhysicalLimitRotations * 360.0 / kGearRatio); // ≈ -169.04°
        public static final double kMaxRotationDegrees =  (kPhysicalLimitRotations * 360.0 / kGearRatio); // ≈ +169.04°

        // Soft-stop buffer: turret is considered "near a limit" when within this many
        // degrees of the hard stop. Prevents the turret from slamming into the physical stop.
        public static final double kNearLimitBuffer = 5.0; // degrees of buffer before hard stop

        // kWrapAroundThreshold kept for API compatibility — wrap-around is DISABLED.
        // With ±169° range the turret does not need to wrap around.
        public static final double kWrapAroundThreshold = 177.0; // unused
        
        // AprilTag Tracking
        public static final double kTrackingP = 0.02; // Proportional gain for tracking
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
        public static final double flywheelDutyCycleOut = -0.75;    // Launcher (CAN 8):   temp -0.2 | final: -1.0

        // SmartDashboard Keys
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
        public static final double kPivotScoreHighPosition = 0.15;       // Position for high scoring
        public static final double kPivotScoreLowPosition = 0.05;        // Position for low scoring
        public static final double kPivotDeployCollectPosition = -1.375; // Deploy position for intake_deploy_collect
        public static final double kPivotHomePosition = 0.0;            // Retract/home position after collecting
        
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
        public static final double kRollersIntakeVelocity = -75.0;  // Speed when intaking game pieces (rps) — negative = correct intake direction
        public static final double kRollersOuttakeVelocity = -20.0; // Speed when ejecting game pieces (rps)
        public static final double kRollersHoldVelocity = 5.0;      // Low speed to hold game piece (rps)
        public static String kRollersVelocityKey;
        public static String kRollersCurrentKey;
        public static String kRollersTempKey;

        // Manual pivot control speed (duty cycle, 0.0–1.0)
        // X button lowers at -kPivotManualSpeed, Y button raises at +kPivotManualSpeed
        public static final double kPivotManualSpeed = 0.1; // 10% — reduced to prevent slamming into limits
    }
    
    /**
     * Constants for the shoot-on-arc feature.
     *
     * The "shooting arc" is a circular arc of valid shooting positions around a tower.
     * At any point on this arc the robot is at a known field position, the turret can
     * aim at the tower, and the launcher can shoot the ball in.
     *
     * All distance-based lookup tables use linear interpolation.
     * Values marked PLACEHOLDER must be tuned on the actual robot.
     */
    public static class ShootingArc {

        // ── Primary tower tag IDs (center of tower opening) ──────────────────
        public static final int kRedPrimaryTagId  = 10;  // Red  tower: x=12.505, y=4.021
        public static final int kBluePrimaryTagId = 20;  // Blue tower: x=5.215,  y=4.021

        // ── All tower tag ID arrays ───────────────────────────────────────────
        public static final int[] kRedTowerTagIds  = {9, 10, 11};
        public static final int[] kBlueTowerTagIds = {19, 20, 21};

        // ── Shooting arc geometry ─────────────────────────────────────────────
        /** Preferred radius of the shooting arc from the tower center (meters).
         *  Reduced from 3.0 to 2.0 — one meter closer to the tower. */
        public static final double kPreferredShootingDistance = 2.0;
        /** Minimum valid shooting distance (meters). */
        public static final double kMinShootingDistance = 1.0;
        /** Maximum valid shooting distance (meters). */
        public static final double kMaxShootingDistance = 4.5;

        // ── Tolerances ────────────────────────────────────────────────────────
        /** Position tolerance for "arrived at arc" check (meters). */
        public static final double kPositionToleranceMeters = 0.20;
        /** Heading tolerance for "facing tower" check (degrees). */
        public static final double kRotationToleranceDeg = 5.0;
        /** Turret aim tolerance before firing is allowed (degrees). */
        public static final double kTurretAimToleranceDeg = 4.0;
        /** Launcher velocity tolerance before firing is allowed (RPS). */
        public static final double kLauncherVelocityTolerance = 5.0;

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

        // ── Launcher velocity lookup table ────────────────────────────────────
        // Format: { {distance_m, velocity_RPS}, ... }  — sorted ascending by distance
        // NEGATIVE values = correct flywheel spin direction (matches launchFromTowerLauncher()
        // which uses flywheelDutyCycleOut = -0.75).
        // Kraken X60 free speed ≈ 100 RPS at 12V → -0.75 duty ≈ -75 RPS.
        // PLACEHOLDER VALUES — tune on robot
        public static final double[][] kLauncherRPSLookup = {
            {1.0, -55.0},
            {1.5, -60.0},
            {2.0, -65.0},
            {3.0, -72.0},
            {4.0, -80.0},
            {4.5, -85.0}
        };

        // ── Hood angle lookup table ───────────────────────────────────────────
        // Format: { {distance_m, hood_rotations}, ... }  — sorted ascending by distance
        // PLACEHOLDER VALUES — tune on robot
        public static final double[][] kHoodAngleLookup = {
            {1.5, 0.08},
            {2.0, 0.10},
            {3.0, 0.15},
            {4.0, 0.20},
            {5.0, 0.25},
            {5.5, 0.28}
        };
    }

    public static class Vision {
        // Camera names — must match exactly what is configured in PhotonVision
        public static final String kCameraNameBF = "CAM_BF";   // Back Facing camera
        public static final String kCameraNameFF = "CAM_FF";   // Front Facing camera

        // Camera stream URLs for dashboard viewing (Elastic, etc.)
        // Using PhotonVision coprocessor IP address (10.80.46.11)
        // Ports are assigned by PhotonVision — check http://photonvision.local:5800 for exact ports
        // CAM_BF stream port and CAM_FF stream port (update if streams don't load in Elastic)
        public static final String kCameraStreamBF = "http://10.80.46.11:1184/stream.mjpg";
        public static final String kCameraStreamFF = "http://10.80.46.11:1182/stream.mjpg";

        // Transform from robot center to Back Facing camera
        // Measured position: X=-12.955" (back), Y=-6.998" (right side), Z=10.711" (from floor)
        // Rotation: Roll=0°, Pitch=+10° (10° from vertical = slightly toward ceiling),
        //   Yaw=187.502° (180° facing backward + 7.502° inward toward left/center)
        public static final Transform3d kRobotToCamBF =
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(-12.955), // Backward (negative X = behind robot center)
                                Units.inchesToMeters(-6.998),  // Right side (negative Y = right in WPILib)
                                Units.inchesToMeters(10.711)   // Up (positive Z, measured from floor)
                        ),
                        new Rotation3d(
                                Units.degreesToRadians(0),       // Roll
                                Units.degreesToRadians(10),      // Pitch: +10° = slightly toward ceiling (10° from vertical/forward)
                                Units.degreesToRadians(187.502))); // Yaw: 180° facing back + 7.502° toed inward toward left

        // Transform from robot center to Front Facing camera
        // Measured position: X=-5.90" (back), Y=+12.938" (left side), Z=17.655" (from ground)
        // Rotation: Roll=0°, Yaw=0° (facing forward), Pitch=+22° (camera leans back 22° from vertical/forward,
        //   lens tilts upward 22° above horizontal — positive pitch in WPILib = nose up)
        public static final Transform3d kRobotToCamFF =
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(-5.90),   // Backward (negative X = behind robot center)
                                Units.inchesToMeters(12.938),  // Left side (positive Y = left in WPILib)
                                Units.inchesToMeters(17.655)   // Up (positive Z, measured from ground)
                        ),
                        new Rotation3d(
                                Units.degreesToRadians(0),     // Roll
                                Units.degreesToRadians(22),    // Pitch: +22° = lens tilts up (camera leans back 22° from forward)
                                Units.degreesToRadians(0)));   // Yaw: facing forward (0°)

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
