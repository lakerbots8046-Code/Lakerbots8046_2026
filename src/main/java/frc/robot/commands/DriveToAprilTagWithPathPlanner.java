package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
//import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

//import java.util.List;
import java.util.Optional;

/**
 * Advanced command to drive the robot to a specific AprilTag using PathPlanner.
 * This command generates an on-the-fly path from the current robot position to
 * a target position near the AprilTag, then follows it using PathPlanner's path following.
 */
public class DriveToAprilTagWithPathPlanner extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    private final int targetTagId;
    private final double targetDistance;
    
    private Command pathFollowingCommand;
    private boolean hasValidTarget = false;
    private Pose2d targetPose;

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
     * Creates a new DriveToAprilTagWithPathPlanner command.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @param visionSubsystem The vision subsystem
     * @param targetTagId The AprilTag ID to drive to
     * @param targetDistance The desired distance from the tag in meters
     */
    public DriveToAprilTagWithPathPlanner(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem visionSubsystem,
            int targetTagId,
            double targetDistance) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.targetTagId = targetTagId;
        this.targetDistance = targetDistance;
        
        addRequirements(drivetrain);
    }
    
    /**
     * Convenience constructor using default target distance from Constants.
     */
    public DriveToAprilTagWithPathPlanner(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem visionSubsystem,
            int targetTagId) {
        this(drivetrain, visionSubsystem, targetTagId, Constants.Vision.kTargetDistanceMeters);
    }
    
    @Override
    public void initialize() {
        SmartDashboard.putString("PathPlanner/Status", "Initializing");
        SmartDashboard.putNumber("PathPlanner/Target Tag", targetTagId);

        hasValidTarget = false;
        pathFollowingCommand = null;
        dashboardCounter = 0;
        
        // Get the AprilTag pose from the field layout
        Optional<Pose2d> tagPoseOpt = getAprilTagPose(targetTagId);
        
        if (tagPoseOpt.isEmpty()) {
            SmartDashboard.putString("PathPlanner/Status", "Tag not in field layout");
            System.err.println("DriveToAprilTagWithPathPlanner: Tag " + targetTagId + " not found in field layout");
            return;
        }
        
        Pose2d tagPose = tagPoseOpt.get();
        
        // Calculate the target pose (offset from the tag by targetDistance)
        // We want to be in front of the tag, facing it
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
        
        // Create path constraints
        PathConstraints constraints = new PathConstraints(
            Constants.Vision.kPathPlannerMaxVelocity,
            Constants.Vision.kPathPlannerMaxAcceleration,
            Constants.Vision.kPathPlannerMaxAngularVelocity,
            Constants.Vision.kPathPlannerMaxAngularAcceleration
        );
        
        try {
            // Generate path on-the-fly from current position to target
            // PathPlanner will handle obstacle avoidance and smooth trajectory generation
            pathFollowingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0 // Goal end velocity (0 = stop at target)
            );
            
            // Schedule the path following command
            CommandScheduler.getInstance().schedule(pathFollowingCommand);
            
            SmartDashboard.putString("PathPlanner/Status", "Path Generated - Following");
            SmartDashboard.putNumber("PathPlanner/Target X",        round(targetPose.getX(), 3));
            SmartDashboard.putNumber("PathPlanner/Target Y",        round(targetPose.getY(), 3));
            SmartDashboard.putNumber("PathPlanner/Target Rotation", round(targetPose.getRotation().getDegrees(), 2));
            
        } catch (Exception e) {
            System.err.println("DriveToAprilTagWithPathPlanner: Failed to generate path: " + e.getMessage());
            SmartDashboard.putString("PathPlanner/Status", "Path Generation Failed");
            hasValidTarget = false;
        }
    }
    
    @Override
    public void execute() {
        if (!hasValidTarget || pathFollowingCommand == null) {
            return;
        }

        // Throttled dashboard writes (~10 Hz) — no motor control here (PathPlanner owns it)
        dashboardCounter++;
        if (dashboardCounter >= 5) {
            dashboardCounter = 0;
            Pose2d currentPose = drivetrain.getState().Pose;
            double distanceToTarget = currentPose.getTranslation().getDistance(targetPose.getTranslation());
            double rotationError = Math.abs(
                currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians()
            );
            SmartDashboard.putNumber( "PathPlanner/Distance to Target",   round(distanceToTarget, 3));
            SmartDashboard.putNumber( "PathPlanner/Rotation Error (deg)", round(Math.toDegrees(rotationError), 2));
            SmartDashboard.putBoolean("PathPlanner/At Target",            isFinished());
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Cancel the path following command if it's still running
        if (pathFollowingCommand != null && pathFollowingCommand.isScheduled()) {
            pathFollowingCommand.cancel();
        }
        
        SmartDashboard.putString("PathPlanner/Status", interrupted ? "Interrupted" : "Complete");
    }
    
    @Override
    public boolean isFinished() {
        if (!hasValidTarget || pathFollowingCommand == null) {
            return true; // Finish immediately if we don't have a valid target
        }
        
        // Check if the path following command has finished
        return !pathFollowingCommand.isScheduled();
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
