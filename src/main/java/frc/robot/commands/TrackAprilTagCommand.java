package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Command to track an AprilTag with the turret.
 * Continuously adjusts turret angle to keep the AprilTag centered.
 * Handles rotation limits and automatically wraps around when needed.
 */
public class TrackAprilTagCommand extends Command {
    private final TurretSubsystem turretSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final int targetTagId;
    
    private final PIDController pidController;
    private boolean isLockedOn = false;
    private boolean isWrappingAround = false;
    private double wrapAroundTarget = 0.0;
    
    /**
     * Creates a new TrackAprilTagCommand.
     * 
     * @param turretSubsystem The turret subsystem
     * @param visionSubsystem The vision subsystem
     * @param targetTagId The AprilTag ID to track (use -1 for currently selected tag)
     */
    public TrackAprilTagCommand(TurretSubsystem turretSubsystem, VisionSubsystem visionSubsystem, int targetTagId) {
        this.turretSubsystem = turretSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.targetTagId = targetTagId;
        
        // Initialize PID controller for tracking
        this.pidController = new PIDController(
            TurretConstants.kTrackingP,
            TurretConstants.kTrackingI,
            TurretConstants.kTrackingD
        );
        
        // Set PID controller tolerance
        pidController.setTolerance(TurretConstants.kTrackingToleranceDegrees);
        
        addRequirements(turretSubsystem);
    }
    
    /**
     * Creates a new TrackAprilTagCommand that tracks the currently selected tag.
     * 
     * @param turretSubsystem The turret subsystem
     * @param visionSubsystem The vision subsystem
     */
    public TrackAprilTagCommand(TurretSubsystem turretSubsystem, VisionSubsystem visionSubsystem) {
        this(turretSubsystem, visionSubsystem, -1);
    }
    
    @Override
    public void initialize() {
        System.out.println("TrackAprilTagCommand: Starting AprilTag tracking");
        pidController.reset();
        isLockedOn = false;
        isWrappingAround = false;
        
        // Publish initial status
        SmartDashboard.putBoolean(TurretConstants.kSmartDashboardPrefix + TurretConstants.kTrackingLockedKey, false);
    }
    
    @Override
    public void execute() {
        // Determine which tag to track
        int tagToTrack = targetTagId == -1 ? visionSubsystem.getSelectedTagId() : targetTagId;
        
        // Check if we're currently wrapping around
        if (isWrappingAround) {
            handleWrapAround();
            return;
        }
        
        // Get vision data from both cameras
        boolean targetVisibleBL = visionSubsystem.isTargetVisibleBL() && 
                                 visionSubsystem.getDetectedTagIdBL() == tagToTrack;
        boolean targetVisibleBR = visionSubsystem.isTargetVisibleBR() && 
                                 visionSubsystem.getDetectedTagIdBR() == tagToTrack;
        
        // If target is not visible, stop tracking
        if (!targetVisibleBL && !targetVisibleBR) {
            turretSubsystem.setSpeed(0.0);
            isLockedOn = false;
            SmartDashboard.putBoolean(TurretConstants.kSmartDashboardPrefix + TurretConstants.kTrackingLockedKey, false);
            SmartDashboard.putString(TurretConstants.kSmartDashboardPrefix + TurretConstants.kStatusKey, 
                                    "Tracking: No Target");
            return;
        }
        
        // Get target yaw (prefer BL camera, use BR as backup)
        double targetYaw = targetVisibleBL ? visionSubsystem.getTargetYawBL() : visionSubsystem.getTargetYawBR();
        
        // Calculate desired turret angle
        // The turret needs to rotate to align with the target
        // Positive yaw means target is to the right, so turret should rotate right (positive)
        double currentAngle = turretSubsystem.getAngle();
        double desiredAngle = currentAngle + targetYaw;
        
        // Check if we need to wrap around
        if (turretSubsystem.needsWrapAround(desiredAngle)) {
            initiateWrapAround(desiredAngle);
            return;
        }
        
        // Calculate PID output
        double pidOutput = pidController.calculate(0, -targetYaw); // Negative because we want to minimize error
        
        // Clamp output to max tracking speed
        pidOutput = Math.max(-TurretConstants.kMaxTrackingSpeed, 
                           Math.min(TurretConstants.kMaxTrackingSpeed, pidOutput));
        
        // Apply minimum output to overcome friction (only if not at setpoint)
        if (!pidController.atSetpoint() && Math.abs(pidOutput) < TurretConstants.kMinTrackingOutput) {
            pidOutput = Math.copySign(TurretConstants.kMinTrackingOutput, pidOutput);
        }
        
        // Apply speed to turret
        turretSubsystem.setSpeed(pidOutput);
        
        // Update locked-on status
        isLockedOn = pidController.atSetpoint();
        
        // Update dashboard
        SmartDashboard.putBoolean(TurretConstants.kSmartDashboardPrefix + TurretConstants.kTrackingLockedKey, isLockedOn);
        SmartDashboard.putNumber(TurretConstants.kSmartDashboardPrefix + TurretConstants.kTrackingErrorKey, targetYaw);
        SmartDashboard.putNumber(TurretConstants.kSmartDashboardPrefix + TurretConstants.kCurrentAngleKey, currentAngle);
        
        String status = isLockedOn ? "Tracking: LOCKED" : String.format("Tracking: Error %.1f°", targetYaw);
        SmartDashboard.putString(TurretConstants.kSmartDashboardPrefix + TurretConstants.kStatusKey, status);
    }
    
    /**
     * Initiates a wrap-around rotation to the opposite side
     * 
     * @param desiredAngle The angle we were trying to reach
     */
    private void initiateWrapAround(double desiredAngle) {
        isWrappingAround = true;
        
        // Calculate wrap-around target (opposite side)
        if (turretSubsystem.isNearMaxLimit()) {
            wrapAroundTarget = -TurretConstants.kWrapAroundTargetOffset;
            System.out.println("TrackAprilTagCommand: Wrapping around to negative side (target: " + wrapAroundTarget + "°)");
        } else {
            wrapAroundTarget = TurretConstants.kWrapAroundTargetOffset;
            System.out.println("TrackAprilTagCommand: Wrapping around to positive side (target: " + wrapAroundTarget + "°)");
        }
        
        SmartDashboard.putString(TurretConstants.kSmartDashboardPrefix + TurretConstants.kStatusKey, 
                                "Tracking: Wrapping Around");
    }
    
    /**
     * Handles the wrap-around rotation
     */
    private void handleWrapAround() {
        double currentAngle = turretSubsystem.getAngle();
        
        // Check if we've reached the wrap-around target
        if (turretSubsystem.atAngle(wrapAroundTarget, TurretConstants.kPositionToleranceDegrees)) {
            isWrappingAround = false;
            pidController.reset();
            System.out.println("TrackAprilTagCommand: Wrap-around complete at " + currentAngle + "°");
            return;
        }
        
        // Continue rotating toward wrap-around target
        double error = wrapAroundTarget - currentAngle;
        double speed = Math.copySign(TurretConstants.kWrapAroundSpeed, error);
        turretSubsystem.setSpeed(speed);
        
        // Update dashboard
        SmartDashboard.putString(TurretConstants.kSmartDashboardPrefix + TurretConstants.kStatusKey, 
                                String.format("Wrapping: %.1f° -> %.1f°", currentAngle, wrapAroundTarget));
    }
    
    @Override
    public void end(boolean interrupted) {
        turretSubsystem.setSpeed(0.0);
        isLockedOn = false;
        isWrappingAround = false;
        
        SmartDashboard.putBoolean(TurretConstants.kSmartDashboardPrefix + TurretConstants.kTrackingLockedKey, false);
        
        if (interrupted) {
            System.out.println("TrackAprilTagCommand: Interrupted");
            SmartDashboard.putString(TurretConstants.kSmartDashboardPrefix + TurretConstants.kStatusKey, 
                                    "Tracking: Stopped");
        } else {
            System.out.println("TrackAprilTagCommand: Ended");
        }
    }
    
    @Override
    public boolean isFinished() {
        // This command runs continuously until interrupted
        return false;
    }
    
    /**
     * Check if the turret is currently locked onto the target
     * 
     * @return true if locked on
     */
    public boolean isLockedOn() {
        return isLockedOn;
    }
}
