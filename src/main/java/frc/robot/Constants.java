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

        // ── Hood physical constants ───────────────────────────────────────────
        /** Total gear reduction from motor to hood mechanism. */
        public static final double kHoodGearRatio = 93.25;
        /** Minimum motor position (rotations). */
        public static final double kHoodMinRotations = 0.0;
        /** Maximum motor position (rotations). */
        public static final double kHoodMaxRotations = 11.5;

        // ── IMPORTANT: Motor direction is INVERTED ────────────────────────────
        // Confirmed on robot (measured physically):
        //   0 rotations   = 22° from straight up = 68° from horizontal (steepest)
        //   11.5 rotations = 62° from straight up = 28° from horizontal (flattest)
        // Higher motor position = FLATTER shot (lower arc).
        // Lower motor position  = STEEPER shot (higher arc).
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
        public static final double kHoodPositionToleranceRotations = 0.2;

        /**
         * Flywheel idle speed (RPS) while the robot is enabled but not actively shooting.
         * Keeping the flywheel spinning at low speed reduces spool-up time when a shot
         * is requested. Negative = shooting direction (same sign as kLauncherRPSLookup).
         *
         * <p>At -15 RPS the flywheel draws minimal current (~5-10% of stall) but already
         * has meaningful rotational inertia. Increase toward -25 RPS for faster spool-up;
         * decrease toward -5 RPS to reduce heat/noise during long idle periods.
         *
         * <p>Set to 0.0 to disable idle spinning entirely.
         */
        public static final double kFlywheelIdleRPS = -15.0;

        /**
         * Toggle to enable or disable flywheel idling between shots.
         *
         * <p>When {@code true}  — the flywheel spins at {@link #kFlywheelIdleRPS} while
         *    the robot is enabled but not actively shooting (reduces spool-up time).
         * <p>When {@code false} — the flywheel is fully stopped between shots
         *    (useful for testing or when idle noise/heat is undesirable).
         *
         * <p>Flip this value to quickly enable/disable idle spinning without touching
         * any other code.
         */
        public static final boolean kFlywheelIdleEnabled = true; // set true to re-enable idling

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
        public static final double kPhysicalLimitRotations = 20.0; // raw motor rotations (from Tuner X) //18.0
        public static final double kMinRotationDegrees = -(kPhysicalLimitRotations * 360.0 / kGearRatio); // ≈ -169.04°
        public static final double kMaxRotationDegrees =  (kPhysicalLimitRotations * 360.0 / kGearRatio); // ≈ +169.04°

        // Soft-stop buffer: turret is considered "near a limit" when within this many
        // degrees of the hard stop. Prevents the turret from slamming into the physical stop.
        public static final double kNearLimitBuffer = 5.0; // degrees of buffer before hard stop

        // ── Turret zero calibration offset ────────────────────────────────────
        //
        // The TalonFX encoder resets to 0 wherever the turret physically is at
        // power-on. If the turret was NOT pointing along the robot's forward
        // direction (+X) when powered on, every angle command will be off by
        // that amount. This offset compensates for that physical misalignment.
        //
        // HOW TO TUNE:
        //   1. Place the robot directly in front of the tower (robot facing tower).
        //   2. Run ShootFromPointCommand (left trigger hard press).
        //   3. Watch "ShootFromPoint/Turret Error (deg)" on the dashboard.
        //   4. If the turret overshoots CCW (left), increase this value.
        //      If the turret overshoots CW (right), decrease this value.
        //   5. Repeat until front shots are centred, then verify side shots.
        //
        // Sign convention:
        //   Positive = turret physical zero is CCW (left) of robot forward.
        //   Negative = turret physical zero is CW  (right) of robot forward.
        // ── Tuning history ────────────────────────────────────────────────────
        // v0: 0.0  — shots consistently miss to the INSIDE EDGE of the hub from
        //            both left and right sides.
        // v1: 5.0  — still 3-5° CCW per driver feedback (inside edge, less severe).
        // v2: 9.0  — (+4° from v1) — appeared to fix the issue.
        //
        // ROOT CAUSE FOUND: The "inside edge" error was NOT a physical turret
        // misalignment. It was caused by ShootingArcManager.getTowerCenter() using
        // the TAG FACE position (x=12.505) as the goal center instead of the actual
        // goal center, which is 2 ft (0.6096 m) BEHIND the tag (x=11.895).
        // This caused a ~9–10° CCW aiming error from every robot position.
        // The previous non-zero offsets (5.0, 9.0) were compensating for this bug.
        //
        // FIXED IN: ShootingArcManager.java — GOAL_CENTER_DEPTH_METERS = 0.6096 m
        //           now subtracted from the tag X coordinate in getTowerCenter().
        //
        // RESET TO: 0.0 — with the correct goal center, no offset should be needed.
        //           If shots are still slightly off after deploying the hub-center fix,
        //           tune this value using "ShootFromPoint/Turret Zero Offset (deg)"
        //           on the dashboard (no redeploy needed), then update this constant.
        //
        // TUNING PROCEDURE (if still needed after hub-center fix):
        //   1. Place robot directly in front of the tower (robot facing tower, ~2m away).
        //   2. Hold left trigger hard (ShootFromPointCommand).
        //   3. Watch "ShootFromPoint/Turret Error (deg)" on dashboard — should read ~0.
        //   4. If shots miss to the INSIDE EDGE  → increase this value (+2° at a time).
        //      If shots miss to the OUTSIDE EDGE → decrease this value (-2° at a time).
        //   5. Verify from both left and right sides of the alliance zone.
        // RESET TO 0.0 — the previous 6.0 was compensating for the wrong goal center
        // calculation in getTowerCenter() (goal center was drifting with robot position).
        // That bug is now fixed: getTowerCenter() uses a fixed -X offset from the primary
        // tag. Re-tune this value on the robot using "ShootFromPoint/Turret Zero Offset (deg)"
        // on the Elastic dashboard (no redeploy needed), then update this constant.
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

        // kWrapAroundThreshold kept for API compatibility — wrap-around is DISABLED.
        // With ±169° range the turret does not need to wrap around.
        public static final double kWrapAroundThreshold = 177.0; // unused
        
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
        public static final double kRollersIntakeVelocity = -100.0; //-85.0 // Speed when intaking game pieces (rps) — negative = correct intake direction, (100 rps max?)
        public static final double kRollersOuttakeVelocity = -20.0; // Speed when ejecting game pieces (rps)
        public static final double kRollersHoldVelocity = 5.0;      // Low speed to hold game piece (rps)
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
    
    /**
     * Constants for the FeedFromCenter command.
     *
     * <p>The robot aims the turret at the midpoint between two feed-station AprilTags
     * (offset a fixed depth behind the midpoint) and spins the flywheel to feed
     * game pieces into the feed station.
     *
     * <p>Tag pairs (one pair per side of each alliance's feed station):
     * <ul>
     *   <li>Red  pair A: tags 1 &amp; 3</li>
     *   <li>Red  pair B: tags 4 &amp; 6</li>
     *   <li>Blue pair A: tags 17 &amp; 19</li>
     *   <li>Blue pair B: tags 20 &amp; 22</li>
     * </ul>
     * The command automatically selects whichever pair is closer to the robot.
     */
    public static class FeedFromCenter {

        // ── Hood position ─────────────────────────────────────────────────────
        /**
         * Hood position (motor rotations) for feed-from-center shots.
         * 0.0 rot = 68° from horizontal (steepest), 11.5 rot = 28° (flattest).
         * TUNE ON ROBOT.
         */
        public static final double kHoodPosition = 3.0;

        // ── Target depth offset ───────────────────────────────────────────────
        /**
         * Distance (meters) to offset the aiming target BEHIND the tag-pair midpoint,
         * in the direction from the robot toward the midpoint.
         * "A few feet behind" the midpoint = 0.6096 m (2 ft). TUNE ON ROBOT.
         */
        public static final double kFeedTargetDepthMeters = 1.5; // 2 ft

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
        public static final double kBallExitSpeedPerRPS = 0.18617; // m/s per RPS

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
        public static final int[] kRedTowerTagIds  = {2, 3, 4, 5, 8, 9, 10, 11};
        public static final int[] kBlueTowerTagIds = {18, 19, 20, 21, 24, 25, 26, 27};

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
        // NEGATIVE values = correct flywheel spin direction.
        // Kraken X60 free speed ≈ 100 RPS at 12V.
        //
        // RPS values are set ~10% ABOVE the ideal single-shot value to compensate for
        // flywheel speed drop during rapid-fire (multiple balls in quick succession
        // temporarily load the motor and reduce actual RPS by ~5-10%).
        //
        // Tuning history:
        //   v1: -55 to -85 RPS — balls landing short.
        //   v2: -65 to -95 RPS — +10 RPS across the board.
        //   v3: 2.0m reduced to -70 RPS — was overshooting.
        //   v4: +10% rapid-fire compensation; hood angles recalculated.
        //   v5: -5 RPS across all distances — power reduced slightly per driver feedback.
        //   v6: -10 RPS across all distances — too much velocity per driver feedback.
        //   v7: PHYSICALLY TESTED on robot. Duty cycle values converted to RPS (DC × 100).
        //       2.0m: DC -0.500 → -50.0 RPS  |  2.5m: DC -0.525 → -52.5 RPS
        //       3.0m: DC -0.575 → -57.5 RPS  |  3.5m: DC -0.575 → -57.5 RPS
        //       1.0m, 1.5m, 4.0m, 4.5m: extrapolated from tested trend.
        //   v8: +5 RPS across all distances — shots landing a bit short.

        // v10: -5 RPS again — still overshooting after v9 reduction.
        public static final double[][] kLauncherRPSLookup = {
            {1.0,  -42.0},  // was -50.0, 43.0, 42.0
            {1.5,  -42.0},  // was -50.5, 43.5, 42.0
            {2.0,  -40.0},  // was -53.0, 46.0, 42.0, 40
            {2.5,  -43.0},  // was -55.5, 48.5, 47.5, 43
            {3.0,  -45.0},  // was -58.5, 51.5, 50.0, 45
           // {3.175, -51.0},  // added intermediate point at 3.175 m (Tower Shot) 49.0, 51.0
            {3.5,  -48.5},  // was -61.5, 54.5, 48.5
            {4.0,  -53.5},  // was -74.5, 67.5, 55.5, 53.5
            {4.5,  -54.0},   // was -80.5, 73.5, 56.0, 54.0
            {5.0,  -58.0}   // was -58, 60
        };

        // ── Hood angle lookup table ───────────────────────────────────────────
        // Format: { {distance_m, hood_motor_rotations}, ... }  — sorted ascending by distance
        //
        // MOTOR DIRECTION: 0 rot = ~85° (steep/vertical), 11.5 rot = ~5° (flat/horizontal)
        // → LOW rotation values = steep angle = high arc trajectory
        // → HIGH rotation values = flat angle  = low arc trajectory
        //
        // Physics: v_exit = 0.18617 * RPS (m/s)
        //          target h = 1.372 m above launch point
        //          Angle formula: θ = lower root of  g·x²·tan²θ - 2v²·x·tanθ + (2v²·h + g·x²) = 0
        //
        // TUNE ON ROBOT: decrease a value (steeper) if ball arcs too low / lands short;
        //                increase a value (flatter) if ball arcs too high / overshoots.
        //
        // ── Physical angle limits (confirmed on robot) ────────────────────────
        //   0.0 rot  = 68° from horizontal (22° from vertical) — steepest possible
        //   11.5 rot = 28° from horizontal (62° from vertical) — flattest possible
        //
        // ── Physics (recalculated with correct angle range) ───────────────────
        //   v_exit = 0.18617 × |RPS|  (m/s, Kraken X60)
        //   target height above launch = 1.372 m  (6 ft goal − 1.5 ft launch height)
        //   g = 9.81 m/s²
        //
        //   Quadratic in tan(θ):  a·t² − x·t + (h + a) = 0
        //   where a = g·x²/(2·v²)
        //   Lower root = low-arc (achievable); upper root always exceeds 68° limit.
        //
        //   Low-arc angles and rotation values (0 rot=68°, 11.5 rot=28°):
        //     rotations = (68° − angle°) / 40° × 11.5
        //
        //   1.0 m  v=12.47 m/s  → 55.8°  → 3.51 rot
        //   1.5 m  v=13.40 m/s  → 44.9°  → 6.64 rot
        //   2.0 m  v=13.40 m/s  → 37.7°  → 8.71 rot
        //   3.0 m  v=15.82 m/s  → 28.1°  → 11.47 rot  (near flat limit)
        //   4.0 m  v=16.76 m/s  → 23.0°  → clamped to 28° → 11.5 rot
        //   4.5 m  v=17.69 m/s  → 21.1°  → clamped to 28° → 11.5 rot
        //
        // Tuning history:
        //   v1–v6: used wrong angle constants (85°/5°) — all angle comments were wrong.
        //   v7–v9: iterative tuning — still too low.
        //   v10: PHYSICALLY TESTED on robot. Hood positions confirmed working.
        //        2.0m: hood=0  |  2.5m: hood=0  |  3.0m: hood=0  |  3.5m: hood=1
        //        1.0m, 1.5m: extrapolated at 0 (steepest, same as close-range tested).
        //        4.0m, 4.5m: extrapolated (+1 rot per 0.5m beyond 3.5m).
        public static final double[][] kHoodAngleLookup = {
            {1.0,  0.0},   // extrapolated — steepest (68° from horizontal)
            {1.5,  0.0},   // extrapolated
            {2.0,  0.0},   // TESTED: hood position 0
            {2.5,  0.0},   // TESTED: hood position 0
            {3.0,  0.0},   // TESTED: hood position 0 0         2   1
            {3.5,  0.75},   // TESTED: hood position 1 1         3
            {4.0,  2.0},   // extrapolated (+1 rot per 0.5m)1   3
            {4.5,  3.0},    // extrapolated (+1 rot per 0.5m) 2  3.5
            {5.0,  3.875}    // was 4.0
        };
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
                                Units.inchesToMeters(-12.941), // Backward (negative X = behind robot center)
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
                                Units.inchesToMeters(-7.069),   // Backward (negative X = behind robot center)
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
                                Units.inchesToMeters(13.746),   // Left (positive Y)
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
                                Units.inchesToMeters(-13.746),  // Right (negative Y)
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
        public static final double kMaxAmbiguity = 1.0;
        
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
