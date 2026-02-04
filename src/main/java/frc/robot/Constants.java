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
    
    

    public class FlapHookConstants{
        public static final double hookflapOpen = 0; // all x 9 for new gear ratio
        public static final double hookPrepare = -70; //-6.5
        public static final double hookLatch = -143; //-143     -16
        public static final double flapCollect = -45; //-5
    }


    public class TagConstants{
        //x and area of tag
        public static final Double[] tagTranslation = {-7.0,0.0};
        public static final Double[] tagPoseSecondLeg  = {-9.0, 0.0};
        public static final Double[] tagePoseAlgea = {-16.0, 0.0};
    }

    public static class IntakeConstants {
        // Motor CAN IDs
        public static final int kIntakeCollectMotorID = 3;
        public static final int kIntakePivotMotorID = 2;
        public static final String kCANBusName = "rio";
        
        // Intake Pivot Position Setpoints (in rotations)
        public static final double kPivotStowedPosition = 0.0;      // Fully retracted/stowed
        public static final double kPivotCollectPosition = 0.25;    // Extended for collecting game pieces
        public static final double kPivotScoreHighPosition = 0.15;  // Position for high scoring
        public static final double kPivotScoreLowPosition = 0.05;   // Position for low scoring
        
        // Intake Collection Velocities (in rotations per second)
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
    }
    
    public static class Vision {
        // Camera names - Updated for back-mounted cameras
        public static final String kCameraNameBL = "CAM_BL"; // Back Left camera
        public static final String kCameraNameBR = "CAM_BR"; // Back Right camera
        
        // Camera stream URLs for dashboard viewing (Elastic, etc.)
        // Using PhotonVision coprocessor IP address (10.80.46.11)
        public static final String kCameraStreamBL = "http://10.80.46.11:1184/stream.mjpg";
        public static final String kCameraStreamBR = "http://10.80.46.11:1182/stream.mjpg";
        
        // Transform from robot center to Back Left camera
        // Measured position: X=-6.0" (adjusted for calibration), Y=+10.8977515" (left), Z=8.264031"
        // Rotation: Roll=0°, Pitch=25° (tilted up), Yaw=190° (facing back-right, angled inward)
        public static final Transform3d kRobotToCamBL =
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(-6.0), // Backward (negative X) - ADJUSTED for calibration
                                Units.inchesToMeters(10.8977515), // Left (positive Y)
                                Units.inchesToMeters(8.264031) // Up (positive Z)
                        ),
                        new Rotation3d(
                                Units.degreesToRadians(0), // Roll
                                Units.degreesToRadians(25), // Pitch (tilted up) - CORRECTED from 65°
                                Units.degreesToRadians(190))); // Yaw (angled inward toward right)

        // Transform from robot center to Back Right camera
        // Measured position: X=-6.0" (adjusted for calibration), Y=-10.8977517" (right), Z=8.264031"
        // Rotation: Roll=0°, Pitch=25° (tilted up), Yaw=170° (facing back-left, angled inward)
        public static final Transform3d kRobotToCamBR =
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(-6.0), // Backward (negative X) - ADJUSTED for calibration
                                Units.inchesToMeters(-10.8977517), // Right (negative Y)
                                Units.inchesToMeters(8.264031) // Up (positive Z)
                        ),
                        new Rotation3d(
                                Units.degreesToRadians(0), // Roll
                                Units.degreesToRadians(25), // Pitch (tilted up) - CORRECTED from 65°
                                Units.degreesToRadians(170))); // Yaw (angled inward toward left)

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
        public static final double kTargetDistanceMeters = 1.0; // Target distance from tag (1 meter)
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
        public static final double kMaxDriveSpeed = 0.5; // 50% of max speed
        public static final double kMaxStrafeSpeed = 0.5;
        public static final double kMaxRotationSpeed = 0.3; // 30% of max rotation
        
        // PathPlanner configuration for driving to AprilTags
        public static final double kPathPlannerMaxVelocity = 3.0; // meters per second
        public static final double kPathPlannerMaxAcceleration = 2.0; // meters per second squared
        public static final double kPathPlannerMaxAngularVelocity = Math.PI; // radians per second
        public static final double kPathPlannerMaxAngularAcceleration = Math.PI; // radians per second squared
    }
    
}
