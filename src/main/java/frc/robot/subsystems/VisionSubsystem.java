package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;
import java.util.List;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera cameraBL;
    private final PhotonCamera cameraBR;
    
    // Pose estimators for each camera
    private final PhotonPoseEstimator poseEstimatorBL;
    private final PhotonPoseEstimator poseEstimatorBR;
    
    // Latest estimated poses
    private Optional<EstimatedRobotPose> latestEstimatedPoseBL = Optional.empty();
    private Optional<EstimatedRobotPose> latestEstimatedPoseBR = Optional.empty();

    // BL Camera data
    private double targetYawBL = 0.0;
    private double targetPitchBL = 0.0;
    private double targetAreaBL = 0.0;
    private double targetDistanceBL = 0.0;
    private int detectedTagIdBL = -1;
    private boolean targetVisibleBL = false;
    private int totalTargetsBL = 0;

    // BR Camera data
    private double targetYawBR = 0.0;
    private double targetPitchBR = 0.0;
    private double targetAreaBR = 0.0;
    private double targetDistanceBR = 0.0;
    private int detectedTagIdBR = -1;
    private boolean targetVisibleBR = false;
    private int totalTargetsBR = 0;

    private int selectedTagId = 14; // Default tag ID

    // SmartDashboard chooser for tag selection
    private final SendableChooser<Integer> tagIdChooser;

    public VisionSubsystem() {
        // Initialize both cameras with names from Constants
        // Set version check to false to avoid version mismatch errors during startup
        cameraBL = new PhotonCamera(Constants.Vision.kCameraNameBL);
        cameraBL.setVersionCheckEnabled(false); // Disable version check to prevent startup errors
        
        cameraBR = new PhotonCamera(Constants.Vision.kCameraNameBR);
        cameraBR.setVersionCheckEnabled(false); // Disable version check to prevent startup errors
        
        // Initialize pose estimators with MULTI_TAG_PNP_ON_COPROCESSOR strategy
        // This uses multiple tags when available for better accuracy
        poseEstimatorBL = new PhotonPoseEstimator(
            Constants.Vision.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.Vision.kRobotToCamBL
        );

        poseEstimatorBR = new PhotonPoseEstimator(
            Constants.Vision.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.Vision.kRobotToCamBR
        );

        // Set up tag ID chooser
        tagIdChooser = new SendableChooser<>();
        tagIdChooser.setDefaultOption("Tag 14", 14);
        tagIdChooser.addOption("Tag 1", 1);
        tagIdChooser.addOption("Tag 2", 2);
        tagIdChooser.addOption("Tag 3", 3);
        tagIdChooser.addOption("Tag 4", 4);
        tagIdChooser.addOption("Tag 5", 5);
        tagIdChooser.addOption("Tag 6", 6);
        tagIdChooser.addOption("Tag 7", 7);
        tagIdChooser.addOption("Tag 8", 8);
        tagIdChooser.addOption("Tag 9", 9);
        tagIdChooser.addOption("Tag 10", 10);
        tagIdChooser.addOption("Tag 11", 11);
        tagIdChooser.addOption("Tag 12", 12);
        tagIdChooser.addOption("Tag 13", 13);
        tagIdChooser.addOption("Tag 15", 15);
        tagIdChooser.addOption("Tag 16", 16);
        tagIdChooser.addOption("Tag 17", 17);
        tagIdChooser.addOption("Tag 18", 18);
        tagIdChooser.addOption("Tag 19", 19);
        tagIdChooser.addOption("Tag 20", 20);
        tagIdChooser.addOption("Tag 21", 21);
        tagIdChooser.addOption("Tag 22", 22);
        tagIdChooser.addOption("Tag 23", 23);
        tagIdChooser.addOption("Tag 24", 24);
        SmartDashboard.putData("Vision Tag Selector", tagIdChooser);
        
        // Publish camera stream URLs for dashboard viewing
        SmartDashboard.putString("Camera/BL/Stream URL", Constants.Vision.kCameraStreamBL);
        SmartDashboard.putString("Camera/BR/Stream URL", Constants.Vision.kCameraStreamBR);
        SmartDashboard.putString("Camera/BL/Name", Constants.Vision.kCameraNameBL);
        SmartDashboard.putString("Camera/BR/Name", Constants.Vision.kCameraNameBR);
    }

    @Override
    public void periodic() {
        // Update selected tag ID from chooser
        selectedTagId = tagIdChooser.getSelected();

        // Update pose estimates for both cameras
        updatePoseEstimates();

        // Process results for each camera independently
        processCameraBLResults();
        processCameraBRResults();
        
        // Publish combined vision status for easy viewing
        publishCombinedStatus();
        
        // Publish pose estimation data to dashboard
        publishPoseEstimationData();
    }
    
    /**
     * Updates pose estimates from both cameras
     */
    private void updatePoseEstimates() {
        // Update BL camera pose estimate
        var resultsBL = cameraBL.getAllUnreadResults();
        if (!resultsBL.isEmpty()) {
            var resultBL = resultsBL.get(resultsBL.size() - 1);
            if (resultBL.hasTargets()) {
                latestEstimatedPoseBL = poseEstimatorBL.update(resultBL);
            } else {
                latestEstimatedPoseBL = Optional.empty();
            }
        } else {
            latestEstimatedPoseBL = Optional.empty();
        }

        // Update BR camera pose estimate
        var resultsBR = cameraBR.getAllUnreadResults();
        if (!resultsBR.isEmpty()) {
            var resultBR = resultsBR.get(resultsBR.size() - 1);
            if (resultBR.hasTargets()) {
                latestEstimatedPoseBR = poseEstimatorBR.update(resultBR);
            } else {
                latestEstimatedPoseBR = Optional.empty();
            }
        } else {
            latestEstimatedPoseBR = Optional.empty();
        }
    }
    
    /**
     * Publishes pose estimation data to SmartDashboard for Elastic dashboard
     */
    private void publishPoseEstimationData() {
        // BL Camera pose estimation
        if (latestEstimatedPoseBL.isPresent()) {
            EstimatedRobotPose estimatedPose = latestEstimatedPoseBL.get();
            Pose2d pose = estimatedPose.estimatedPose.toPose2d();
            
            SmartDashboard.putBoolean("Vision/BL/Pose Valid", true);
            SmartDashboard.putNumber("Vision/BL/Pose X", pose.getX());
            SmartDashboard.putNumber("Vision/BL/Pose Y", pose.getY());
            SmartDashboard.putNumber("Vision/BL/Pose Rotation", pose.getRotation().getDegrees());
            SmartDashboard.putNumber("Vision/BL/Pose Timestamp", estimatedPose.timestampSeconds);
            SmartDashboard.putNumber("Vision/BL/Tags Used", estimatedPose.targetsUsed.size());
            
            // Publish tag IDs used
            StringBuilder tagIds = new StringBuilder();
            for (var target : estimatedPose.targetsUsed) {
                if (tagIds.length() > 0) tagIds.append(", ");
                tagIds.append(target.getFiducialId());
            }
            SmartDashboard.putString("Vision/BL/Tags Used IDs", tagIds.toString());
        } else {
            SmartDashboard.putBoolean("Vision/BL/Pose Valid", false);
        }
        
        // BR Camera pose estimation
        if (latestEstimatedPoseBR.isPresent()) {
            EstimatedRobotPose estimatedPose = latestEstimatedPoseBR.get();
            Pose2d pose = estimatedPose.estimatedPose.toPose2d();
            
            SmartDashboard.putBoolean("Vision/BR/Pose Valid", true);
            SmartDashboard.putNumber("Vision/BR/Pose X", pose.getX());
            SmartDashboard.putNumber("Vision/BR/Pose Y", pose.getY());
            SmartDashboard.putNumber("Vision/BR/Pose Rotation", pose.getRotation().getDegrees());
            SmartDashboard.putNumber("Vision/BR/Pose Timestamp", estimatedPose.timestampSeconds);
            SmartDashboard.putNumber("Vision/BR/Tags Used", estimatedPose.targetsUsed.size());
            
            // Publish tag IDs used
            StringBuilder tagIds = new StringBuilder();
            for (var target : estimatedPose.targetsUsed) {
                if (tagIds.length() > 0) tagIds.append(", ");
                tagIds.append(target.getFiducialId());
            }
            SmartDashboard.putString("Vision/BR/Tags Used IDs", tagIds.toString());
        } else {
            SmartDashboard.putBoolean("Vision/BR/Pose Valid", false);
        }
        
        // Combined status
        boolean anyPoseValid = latestEstimatedPoseBL.isPresent() || latestEstimatedPoseBR.isPresent();
        SmartDashboard.putBoolean("Vision/Any Pose Valid", anyPoseValid);
        
        int totalTagsUsed = 0;
        if (latestEstimatedPoseBL.isPresent()) {
            totalTagsUsed += latestEstimatedPoseBL.get().targetsUsed.size();
        }
        if (latestEstimatedPoseBR.isPresent()) {
            totalTagsUsed += latestEstimatedPoseBR.get().targetsUsed.size();
        }
        SmartDashboard.putNumber("Vision/Total Tags Used", totalTagsUsed);
    }
    
    /**
     * Publishes combined vision status for easy dashboard viewing
     */
    private void publishCombinedStatus() {
        // Show which tag we're looking for
        SmartDashboard.putNumber("Target Tag ID", selectedTagId);
        
        // Show detected tags from both cameras
        String detectedTags = "";
        if (targetVisibleBL && targetVisibleBR) {
            detectedTags = "BL: Tag " + detectedTagIdBL + " | BR: Tag " + detectedTagIdBR;
        } else if (targetVisibleBL) {
            detectedTags = "BL: Tag " + detectedTagIdBL;
        } else if (targetVisibleBR) {
            detectedTags = "BR: Tag " + detectedTagIdBR;
        } else {
            detectedTags = "No Tags Detected";
        }
        SmartDashboard.putString("Detected Tags", detectedTags);
        
        // Show if target tag is visible
        boolean targetFound = (targetVisibleBL && detectedTagIdBL == selectedTagId) || 
                             (targetVisibleBR && detectedTagIdBR == selectedTagId);
        SmartDashboard.putBoolean("Target Tag Found", targetFound);
        
        // Show which camera sees the target
        String cameraSeeing = "";
        if (targetVisibleBL && detectedTagIdBL == selectedTagId && 
            targetVisibleBR && detectedTagIdBR == selectedTagId) {
            cameraSeeing = "Both Cameras";
        } else if (targetVisibleBL && detectedTagIdBL == selectedTagId) {
            cameraSeeing = "Back-Left Camera";
        } else if (targetVisibleBR && detectedTagIdBR == selectedTagId) {
            cameraSeeing = "Back-Right Camera";
        } else {
            cameraSeeing = "None";
        }
        SmartDashboard.putString("Target Visible On", cameraSeeing);
    }

    private void processCameraBLResults() {
        // Use getAllUnreadResults() to get latest result
        var results = cameraBL.getAllUnreadResults();
        var result = results.isEmpty() ? null : results.get(results.size() - 1);

        // Only reset if no targets detected
        if (result == null || !result.hasTargets()) {
            targetYawBL = 0.0;
            targetPitchBL = 0.0;
            targetAreaBL = 0.0;
            targetDistanceBL = 0.0;
            detectedTagIdBL = -1;
            targetVisibleBL = false;
            totalTargetsBL = 0;
        } else {
            List<PhotonTrackedTarget> targets = result.getTargets();
            totalTargetsBL = targets.size();
            
            // ONLY find the selected tag - do not use fallback
            PhotonTrackedTarget selectedTarget = null;
            for (PhotonTrackedTarget target : targets) {
                if (target.getFiducialId() == selectedTagId) {
                    selectedTarget = target;
                    break;
                }
            }
            
            // Extract data ONLY if we found the selected tag
            if (selectedTarget != null) {
                targetVisibleBL = true;
                detectedTagIdBL = selectedTarget.getFiducialId();
                targetYawBL = selectedTarget.getYaw();
                targetPitchBL = selectedTarget.getPitch();
                targetAreaBL = selectedTarget.getArea();
                
                // Calculate approximate distance using target area (rough estimation)
                if (targetAreaBL > 0) {
                    targetDistanceBL = Math.sqrt(1.0 / targetAreaBL) * 10.0;
                }
            } else {
                // Selected tag not found, but other tags are visible
                targetYawBL = 0.0;
                targetPitchBL = 0.0;
                targetAreaBL = 0.0;
                targetDistanceBL = 0.0;
                detectedTagIdBL = -1;
                targetVisibleBL = false;
            }
        }

        // Update SmartDashboard with comprehensive metrics for BL camera
        SmartDashboard.putBoolean("BL Target Visible", targetVisibleBL);
        SmartDashboard.putNumber("BL Detected Tag ID", detectedTagIdBL);
        SmartDashboard.putNumber("BL Target Yaw (deg)", targetYawBL);
        SmartDashboard.putNumber("BL Target Pitch (deg)", targetPitchBL);
        SmartDashboard.putNumber("BL Target Area (%)", targetAreaBL);
        SmartDashboard.putNumber("BL Approx Distance", targetDistanceBL);
        SmartDashboard.putNumber("BL Total Targets", totalTargetsBL);
        SmartDashboard.putBoolean("BL Camera Connected", cameraBL.isConnected());
        
        // Legacy outputs for compatibility
        SmartDashboard.putNumber("AprilTag " + selectedTagId + " Yaw BL", targetYawBL);
        SmartDashboard.putBoolean("Vision Target Visible BL", targetVisibleBL);
    }

    private void processCameraBRResults() {
        // Use getAllUnreadResults() to get latest result
        var results = cameraBR.getAllUnreadResults();
        var result = results.isEmpty() ? null : results.get(results.size() - 1);

        // Only reset if no targets detected
        if (result == null || !result.hasTargets()) {
            targetYawBR = 0.0;
            targetPitchBR = 0.0;
            targetAreaBR = 0.0;
            targetDistanceBR = 0.0;
            detectedTagIdBR = -1;
            targetVisibleBR = false;
            totalTargetsBR = 0;
        } else {
            List<PhotonTrackedTarget> targets = result.getTargets();
            totalTargetsBR = targets.size();
            
            // ONLY find the selected tag - do not use fallback
            PhotonTrackedTarget selectedTarget = null;
            for (PhotonTrackedTarget target : targets) {
                if (target.getFiducialId() == selectedTagId) {
                    selectedTarget = target;
                    break;
                }
            }
            
            // Extract data ONLY if we found the selected tag
            if (selectedTarget != null) {
                targetVisibleBR = true;
                detectedTagIdBR = selectedTarget.getFiducialId();
                targetYawBR = selectedTarget.getYaw();
                targetPitchBR = selectedTarget.getPitch();
                targetAreaBR = selectedTarget.getArea();
                
                // Calculate approximate distance using target area (rough estimation)
                if (targetAreaBR > 0) {
                    targetDistanceBR = Math.sqrt(1.0 / targetAreaBR) * 10.0;
                }
            } else {
                // Selected tag not found, but other tags are visible
                targetYawBR = 0.0;
                targetPitchBR = 0.0;
                targetAreaBR = 0.0;
                targetDistanceBR = 0.0;
                detectedTagIdBR = -1;
                targetVisibleBR = false;
            }
        }

        // Update SmartDashboard with comprehensive metrics for BR camera
        SmartDashboard.putBoolean("BR Target Visible", targetVisibleBR);
        SmartDashboard.putNumber("BR Detected Tag ID", detectedTagIdBR);
        SmartDashboard.putNumber("BR Target Yaw (deg)", targetYawBR);
        SmartDashboard.putNumber("BR Target Pitch (deg)", targetPitchBR);
        SmartDashboard.putNumber("BR Target Area (%)", targetAreaBR);
        SmartDashboard.putNumber("BR Approx Distance", targetDistanceBR);
        SmartDashboard.putNumber("BR Total Targets", totalTargetsBR);
        SmartDashboard.putBoolean("BR Camera Connected", cameraBR.isConnected());
        
        // Legacy outputs for compatibility
        SmartDashboard.putNumber("AprilTag " + selectedTagId + " Yaw BR", targetYawBR);
        SmartDashboard.putBoolean("Vision Target Visible BR", targetVisibleBR);
    }

    // Getter methods for BL camera
    public double getTargetYawBL() {
        return targetYawBL;
    }

    public boolean isTargetVisibleBL() {
        return targetVisibleBL;
    }
    
    public int getDetectedTagIdBL() {
        return detectedTagIdBL;
    }
    
    public double getTargetDistanceBL() {
        return targetDistanceBL;
    }

    // Getter methods for BR camera
    public double getTargetYawBR() {
        return targetYawBR;
    }

    public boolean isTargetVisibleBR() {
        return targetVisibleBR;
    }
    
    public int getDetectedTagIdBR() {
        return detectedTagIdBR;
    }
    
    public double getTargetDistanceBR() {
        return targetDistanceBR;
    }

    // Public getter for selected tag ID
    public int getSelectedTagId() {
        return selectedTagId;
    }
    
    /**
     * Gets the latest estimated robot pose from the BL camera
     * @return Optional containing the estimated pose, or empty if no estimate available
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseBL() {
        return latestEstimatedPoseBL;
    }
    
    /**
     * Gets the latest estimated robot pose from the BR camera
     * @return Optional containing the estimated pose, or empty if no estimate available
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseBR() {
        return latestEstimatedPoseBR;
    }
    
    /**
     * Sets the reference pose for the pose estimators.
     * This should be called when resetting odometry.
     * @param pose The new reference pose
     */
    public void setReferencePose(Pose2d pose) {
        poseEstimatorBL.setReferencePose(new Pose3d(pose));
        poseEstimatorBR.setReferencePose(new Pose3d(pose));
    }
}
