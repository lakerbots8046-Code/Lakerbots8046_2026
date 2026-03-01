package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * Command to drive the robot to a specific AprilTag using vision data.
 * The robot will position itself at a target distance from the tag and face it.
 */
public class DriveToAprilTag extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    private final int targetTagId;
    private final double targetDistance;
    
    // PID Controllers for X, Y, and rotation
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    
    // Swerve request for field-centric driving
    private final SwerveRequest.FieldCentric driveRequest;
    
    private Pose2d targetPose;
    private boolean hasValidTarget = false;

    /** Throttle counter — SmartDashboard writes every 5 execute() calls (~100 ms). */
    private int dashboardCounter = 0;

    /**
     * Rounds a double to the specified number of decimal places for cleaner dashboard display.
     * Position values use 3 decimals (mm precision); angles use 2 decimals.
     */
    private static double round(double value, int decimals) {
        double scale = Math.pow(10, decimals);
        return Math.round(value * scale) / scale;
    }
    
    /**
     * Creates a new DriveToAprilTag command.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @param visionSubsystem The vision subsystem
     * @param targetTagId The AprilTag ID to drive to
     * @param targetDistance The desired distance from the tag in meters
     */
    public DriveToAprilTag(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem visionSubsystem,
            int targetTagId,
            double targetDistance) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.targetTagId = targetTagId;
        this.targetDistance = targetDistance;
        
        // Initialize PID controllers
        xController = new PIDController(
            Constants.Vision.kDriveP,
            Constants.Vision.kDriveI,
            Constants.Vision.kDriveD
        );
        
        yController = new PIDController(
            Constants.Vision.kStrafeP,
            Constants.Vision.kStrafeI,
            Constants.Vision.kStrafeD
        );
        
        rotationController = new PIDController(
            Constants.Vision.kRotationP,
            Constants.Vision.kRotationI,
            Constants.Vision.kRotationD
        );
        
        // Configure rotation controller for continuous input (angles wrap around)
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        // Set tolerances
        xController.setTolerance(Constants.Vision.kPositionToleranceMeters);
        yController.setTolerance(Constants.Vision.kPositionToleranceMeters);
        rotationController.setTolerance(Math.toRadians(Constants.Vision.kRotationToleranceDegrees));
        
        // Initialize drive request
        driveRequest = new SwerveRequest.FieldCentric();
        
        addRequirements(drivetrain);
    }
    
    /**
     * Convenience constructor using default target distance from Constants.
     */
    public DriveToAprilTag(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem visionSubsystem,
            int targetTagId) {
        this(drivetrain, visionSubsystem, targetTagId, Constants.Vision.kTargetDistanceMeters);
    }
    
    @Override
    public void initialize() {
        SmartDashboard.putString("DriveToTag/Status", "Initializing");
        SmartDashboard.putNumber("DriveToTag/Target Tag", targetTagId);

        xController.reset();
        yController.reset();
        rotationController.reset();

        hasValidTarget = false;
        dashboardCounter = 0;
    }
    
    @Override
    public void execute() {
        // Get the AprilTag pose from the field layout
        Optional<Pose2d> tagPoseOpt = getAprilTagPose(targetTagId);
        
        if (tagPoseOpt.isEmpty()) {
            SmartDashboard.putString("DriveToTag/Status", "Tag not in field layout");
            hasValidTarget = false;
            return;
        }
        
        Pose2d tagPose = tagPoseOpt.get();
        
        // Calculate the target pose (offset from the tag by targetDistance)
        // We want to be in front of the tag, facing it
        // The tag's rotation tells us which way it's facing
        Rotation2d tagRotation = tagPose.getRotation();
        
        // Calculate offset: move back from tag by targetDistance in the direction opposite to tag's facing
        Translation2d offset = new Translation2d(
            -targetDistance * tagRotation.getCos(),
            -targetDistance * tagRotation.getSin()
        );
        
        targetPose = new Pose2d(
            tagPose.getX() + offset.getX(),
            tagPose.getY() + offset.getY(),
            tagRotation // Face the same direction as the tag
        );
        
        hasValidTarget = true;
        
        // Get current robot pose
        Pose2d currentPose = drivetrain.getState().Pose;
        
        // Calculate errors
        double xError = targetPose.getX() - currentPose.getX();
        double yError = targetPose.getY() - currentPose.getY();
        double rotationError = targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians();
        
        // Normalize rotation error to [-pi, pi]
        while (rotationError > Math.PI) rotationError -= 2 * Math.PI;
        while (rotationError < -Math.PI) rotationError += 2 * Math.PI;
        
        // Calculate control outputs
        double xSpeed = xController.calculate(0, -xError); // Negative because we want to reduce error
        double ySpeed = yController.calculate(0, -yError);
        double rotSpeed = rotationController.calculate(0, -rotationError);
        
        // Clamp speeds to maximum values
        // Get max speeds from TunerConstants (same as used in RobotContainer)
        double maxSpeed = frc.robot.generated.TunerConstants.kSpeedAt12Volts.in(edu.wpi.first.units.Units.MetersPerSecond) 
            * Constants.Vision.kMaxDriveSpeed;
        double maxRotSpeed = edu.wpi.first.units.Units.RotationsPerSecond.of(0.75).in(edu.wpi.first.units.Units.RadiansPerSecond) 
            * Constants.Vision.kMaxRotationSpeed;
        
        xSpeed = Math.max(-maxSpeed, Math.min(maxSpeed, xSpeed));
        ySpeed = Math.max(-maxSpeed, Math.min(maxSpeed, ySpeed));
        rotSpeed = Math.max(-maxRotSpeed, Math.min(maxRotSpeed, rotSpeed));
        
        // Apply the drive request (unthrottled — motor control every loop)
        drivetrain.setControl(
            driveRequest
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(rotSpeed)
        );

        // Throttled dashboard writes (~10 Hz)
        dashboardCounter++;
        if (dashboardCounter >= 5) {
            dashboardCounter = 0;
            SmartDashboard.putString( "DriveToTag/Status",               "Driving");
            SmartDashboard.putNumber( "DriveToTag/X Error",              round(xError, 3));
            SmartDashboard.putNumber( "DriveToTag/Y Error",              round(yError, 3));
            SmartDashboard.putNumber( "DriveToTag/Rotation Error (deg)", round(Math.toDegrees(rotationError), 2));
            SmartDashboard.putNumber( "DriveToTag/Distance to Target",   round(Math.sqrt(xError * xError + yError * yError), 3));
            SmartDashboard.putBoolean("DriveToTag/At Target",            isFinished());
            SmartDashboard.putNumber( "DriveToTag/Target X",             round(targetPose.getX(), 3));
            SmartDashboard.putNumber( "DriveToTag/Target Y",             round(targetPose.getY(), 3));
            SmartDashboard.putNumber( "DriveToTag/Target Rotation",      round(targetPose.getRotation().getDegrees(), 2));
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        drivetrain.setControl(new SwerveRequest.Idle());
        
        SmartDashboard.putString("DriveToTag/Status", interrupted ? "Interrupted" : "Complete");
    }
    
    @Override
    public boolean isFinished() {
        if (!hasValidTarget) {
            return false; // Don't finish if we don't have a valid target
        }
        
        // Check if all controllers are at their setpoints
        return xController.atSetpoint() && 
               yController.atSetpoint() && 
               rotationController.atSetpoint();
    }
    
    /**
     * Gets the 2D pose of an AprilTag from the field layout.
     * 
     * @param tagId The AprilTag ID
     * @return Optional containing the tag's Pose2d if found
     */
    private Optional<Pose2d> getAprilTagPose(int tagId) {
        try {
            var tagPose3d = Constants.Vision.kTagLayout.getTagPose(tagId);
            if (tagPose3d.isPresent()) {
                var pose3d = tagPose3d.get();
                return Optional.of(new Pose2d(
                    pose3d.getX(),
                    pose3d.getY(),
                    pose3d.getRotation().toRotation2d()
                ));
            }
        } catch (Exception e) {
            System.err.println("Error getting AprilTag pose: " + e.getMessage());
        }
        return Optional.empty();
    }
}
