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
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera cameraBF;  // Back Facing camera (formerly BL)
    private final PhotonCamera cameraFF;  // Front Facing camera (formerly BR)

    // Pose estimators for each camera
    private final PhotonPoseEstimator poseEstimatorBF;
    private final PhotonPoseEstimator poseEstimatorFF;

    // Latest estimated poses
    private Optional<EstimatedRobotPose> latestEstimatedPoseBF = Optional.empty();
    private Optional<EstimatedRobotPose> latestEstimatedPoseFF = Optional.empty();

    // Best (lowest) ambiguity score seen this loop for each camera.
    // -1.0 = no target / multi-tag result (always accepted).
    private double bestAmbiguityBF = -1.0;
    private double bestAmbiguityFF = -1.0;

    // Back Facing Camera data
    private double targetYawBF = 0.0;
    private double targetPitchBF = 0.0;
    private double targetAreaBF = 0.0;
    private double targetDistanceBF = 0.0;
    private int detectedTagIdBF = -1;
    private boolean targetVisibleBF = false;
    private int totalTargetsBF = 0;
    /** All tag IDs currently visible to the BF camera (updated every loop). */
    private List<Integer> allDetectedTagIdsBF = new ArrayList<>();

    // Front Facing Camera data
    private double targetYawFF = 0.0;
    private double targetPitchFF = 0.0;
    private double targetAreaFF = 0.0;
    private double targetDistanceFF = 0.0;
    private int detectedTagIdFF = -1;
    private boolean targetVisibleFF = false;
    private int totalTargetsFF = 0;
    /** All tag IDs currently visible to the FF camera (updated every loop). */
    private List<Integer> allDetectedTagIdsFF = new ArrayList<>();

    private int selectedTagId = 14; // Default tag ID

    // SmartDashboard chooser for tag selection
    private final SendableChooser<Integer> tagIdChooser;

    // Throttle SmartDashboard updates to every 5 loops (~100ms) to reduce NT memory pressure.
    // Camera processing (pose estimation, target tracking) still runs every loop.
    private int periodicCounter = 0;

    public VisionSubsystem() {
        // Initialize both cameras with names from Constants
        cameraBF = new PhotonCamera(Constants.Vision.kCameraNameBF);
        cameraBF.setVersionCheckEnabled(false);

        cameraFF = new PhotonCamera(Constants.Vision.kCameraNameFF);
        cameraFF.setVersionCheckEnabled(false);

        // Initialize pose estimators with MULTI_TAG_PNP_ON_COPROCESSOR strategy.
        // This uses full 3-D Perspective-n-Point (PnP) solving on the coprocessor.
        poseEstimatorBF = new PhotonPoseEstimator(
            Constants.Vision.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.Vision.kRobotToCamBF
        );
        // When only one tag is visible the estimator falls back to single-tag 3-D PnP.
        // CLOSEST_TO_REFERENCE_POSE picks the less-ambiguous of the two possible poses
        // by comparing each candidate against the current odometry estimate — this is
        // the best available single-tag disambiguation strategy.
        poseEstimatorBF.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

        poseEstimatorFF = new PhotonPoseEstimator(
            Constants.Vision.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.Vision.kRobotToCamFF
        );
        poseEstimatorFF.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

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
        tagIdChooser.addOption("Tag 25", 25);
        tagIdChooser.addOption("Tag 26", 26);
        tagIdChooser.addOption("Tag 27", 27);
        tagIdChooser.addOption("Tag 28", 28);
        tagIdChooser.addOption("Tag 29", 29);
        tagIdChooser.addOption("Tag 30", 30);
        tagIdChooser.addOption("Tag 31", 31);
        tagIdChooser.addOption("Tag 32", 32);
        SmartDashboard.putData("Vision Tag Selector", tagIdChooser);

        // Publish camera stream URLs once at startup (not every loop)
        SmartDashboard.putString("Camera/BF/Stream URL", Constants.Vision.kCameraStreamBF);
        SmartDashboard.putString("Camera/FF/Stream URL", Constants.Vision.kCameraStreamFF);
        SmartDashboard.putString("Camera/BF/Name", Constants.Vision.kCameraNameBF);
        SmartDashboard.putString("Camera/FF/Name", Constants.Vision.kCameraNameFF);
    }

    @Override
    public void periodic() {
        selectedTagId = tagIdChooser.getSelected();

        // ── Read camera results ONCE per loop ────────────────────────────────
        // getAllUnreadResults() is DESTRUCTIVE — calling it a second time on the
        // same camera returns an empty list because the buffer was already cleared.
        var allResultsBF = cameraBF.getAllUnreadResults();
        var allResultsFF = cameraFF.getAllUnreadResults();

        var latestBF = allResultsBF.isEmpty() ? null : allResultsBF.get(allResultsBF.size() - 1);
        var latestFF = allResultsFF.isEmpty() ? null : allResultsFF.get(allResultsFF.size() - 1);

        // ── Update pose estimates (must run every loop for accuracy) ──────────
        if (latestBF != null) {
            latestEstimatedPoseBF = latestBF.hasTargets()
                ? poseEstimatorBF.update(latestBF)
                : Optional.empty();
        }
        if (latestFF != null) {
            latestEstimatedPoseFF = latestFF.hasTargets()
                ? poseEstimatorFF.update(latestFF)
                : Optional.empty();
        }

        // ── Process BF camera target tracking (must run every loop) ──────────
        if (latestBF == null || !latestBF.hasTargets()) {
            targetYawBF = 0.0; targetPitchBF = 0.0; targetAreaBF = 0.0;
            targetDistanceBF = 0.0; detectedTagIdBF = -1;
            targetVisibleBF = false; totalTargetsBF = 0;
            bestAmbiguityBF = -1.0;
            allDetectedTagIdsBF = Collections.emptyList();
        } else {
            var targetsBF = latestBF.getTargets();
            totalTargetsBF = targetsBF.size();
            // Build the full list of all visible tag IDs for tower-tag scanning
            List<Integer> idsBF = new ArrayList<>(targetsBF.size());
            for (PhotonTrackedTarget t : targetsBF) { idsBF.add(t.getFiducialId()); }
            allDetectedTagIdsBF = idsBF;
            // Track the selectedTagId target for yaw/pitch/area telemetry
            PhotonTrackedTarget selBF = null;
            for (PhotonTrackedTarget t : targetsBF) {
                if (t.getFiducialId() == selectedTagId) { selBF = t; break; }
            }
            if (selBF != null) {
                bestAmbiguityBF = selBF.getPoseAmbiguity(); // -1 for multi-tag, 0–1 for single-tag
                targetVisibleBF = true; detectedTagIdBF = selBF.getFiducialId();
                targetYawBF = selBF.getYaw(); targetPitchBF = selBF.getPitch();
                targetAreaBF = selBF.getArea();
                if (targetAreaBF > 0) targetDistanceBF = Math.sqrt(1.0 / targetAreaBF) * 10.0;
            } else {
                targetYawBF = 0.0; targetPitchBF = 0.0; targetAreaBF = 0.0;
                targetDistanceBF = 0.0; detectedTagIdBF = -1; targetVisibleBF = false;
                bestAmbiguityBF = -1.0;
            }
        }

        // ── Process FF camera target tracking (must run every loop) ──────────
        if (latestFF == null || !latestFF.hasTargets()) {
            targetYawFF = 0.0; targetPitchFF = 0.0; targetAreaFF = 0.0;
            targetDistanceFF = 0.0; detectedTagIdFF = -1;
            targetVisibleFF = false; totalTargetsFF = 0;
            bestAmbiguityFF = -1.0;
            allDetectedTagIdsFF = Collections.emptyList();
        } else {
            var targetsFF = latestFF.getTargets();
            totalTargetsFF = targetsFF.size();
            // Build the full list of all visible tag IDs for tower-tag scanning
            List<Integer> idsFF = new ArrayList<>(targetsFF.size());
            for (PhotonTrackedTarget t : targetsFF) { idsFF.add(t.getFiducialId()); }
            allDetectedTagIdsFF = idsFF;
            // Track the selectedTagId target for yaw/pitch/area telemetry
            PhotonTrackedTarget selFF = null;
            for (PhotonTrackedTarget t : targetsFF) {
                if (t.getFiducialId() == selectedTagId) { selFF = t; break; }
            }
            if (selFF != null) {
                bestAmbiguityFF = selFF.getPoseAmbiguity(); // -1 for multi-tag, 0–1 for single-tag
                targetVisibleFF = true; detectedTagIdFF = selFF.getFiducialId();
                targetYawFF = selFF.getYaw(); targetPitchFF = selFF.getPitch();
                targetAreaFF = selFF.getArea();
                if (targetAreaFF > 0) targetDistanceFF = Math.sqrt(1.0 / targetAreaFF) * 10.0;
            } else {
                targetYawFF = 0.0; targetPitchFF = 0.0; targetAreaFF = 0.0;
                targetDistanceFF = 0.0; detectedTagIdFF = -1; targetVisibleFF = false;
                bestAmbiguityFF = -1.0;
            }
        }

        // ── Throttle ALL SmartDashboard puts to every 5 loops (~100ms) ───────
        // Camera processing above runs every loop for pose accuracy.
        // Only the NT writes are gated here to prevent roboRIO memory overload.
        periodicCounter++;
        if (periodicCounter < 5) return;
        periodicCounter = 0;

        // BF camera telemetry
        SmartDashboard.putBoolean("BF Target Visible",        targetVisibleBF);
        SmartDashboard.putNumber( "BF Detected Tag ID",       detectedTagIdBF);
        SmartDashboard.putNumber( "BF Target Yaw (deg)",      targetYawBF);
        SmartDashboard.putNumber( "BF Target Pitch (deg)",    targetPitchBF);
        SmartDashboard.putNumber( "BF Target Area (%)",       targetAreaBF);
        SmartDashboard.putNumber( "BF Approx Distance",       targetDistanceBF);
        SmartDashboard.putNumber( "BF Total Targets",         totalTargetsBF);
        SmartDashboard.putBoolean("BF Camera Connected",      cameraBF.isConnected());
        SmartDashboard.putNumber( "AprilTag " + selectedTagId + " Yaw BF", targetYawBF);
        SmartDashboard.putBoolean("Vision Target Visible BF", targetVisibleBF);
        // Ambiguity: -1 = multi-tag/no target (always accepted), 0–1 = single-tag score
        SmartDashboard.putNumber( "BF Best Ambiguity",        bestAmbiguityBF);

        // FF camera telemetry
        SmartDashboard.putBoolean("FF Target Visible",        targetVisibleFF);
        SmartDashboard.putNumber( "FF Detected Tag ID",       detectedTagIdFF);
        SmartDashboard.putNumber( "FF Target Yaw (deg)",      targetYawFF);
        SmartDashboard.putNumber( "FF Target Pitch (deg)",    targetPitchFF);
        SmartDashboard.putNumber( "FF Target Area (%)",       targetAreaFF);
        SmartDashboard.putNumber( "FF Approx Distance",       targetDistanceFF);
        SmartDashboard.putNumber( "FF Total Targets",         totalTargetsFF);
        SmartDashboard.putBoolean("FF Camera Connected",      cameraFF.isConnected());
        SmartDashboard.putNumber( "AprilTag " + selectedTagId + " Yaw FF", targetYawFF);
        SmartDashboard.putBoolean("Vision Target Visible FF", targetVisibleFF);
        // Ambiguity: -1 = multi-tag/no target (always accepted), 0–1 = single-tag score
        SmartDashboard.putNumber( "FF Best Ambiguity",        bestAmbiguityFF);

        publishCombinedStatus();
        publishPoseEstimationData();
    }

    private void publishPoseEstimationData() {
        // Back Facing Camera pose estimation
        if (latestEstimatedPoseBF.isPresent()) {
            EstimatedRobotPose estimatedPose = latestEstimatedPoseBF.get();
            Pose2d pose = estimatedPose.estimatedPose.toPose2d();

            SmartDashboard.putBoolean("Vision/BF/Pose Valid",    true);
            SmartDashboard.putNumber( "Vision/BF/Pose X",        pose.getX());
            SmartDashboard.putNumber( "Vision/BF/Pose Y",        pose.getY());
            SmartDashboard.putNumber( "Vision/BF/Pose Rotation", pose.getRotation().getDegrees());
            SmartDashboard.putNumber( "Vision/BF/Pose Timestamp",estimatedPose.timestampSeconds);
            SmartDashboard.putNumber( "Vision/BF/Tags Used",     estimatedPose.targetsUsed.size());

            StringBuilder tagIds = new StringBuilder();
            for (var target : estimatedPose.targetsUsed) {
                if (tagIds.length() > 0) tagIds.append(", ");
                tagIds.append(target.getFiducialId());
            }
            SmartDashboard.putString("Vision/BF/Tags Used IDs", tagIds.toString());
        } else {
            SmartDashboard.putBoolean("Vision/BF/Pose Valid", false);
        }

        // Front Facing Camera pose estimation
        if (latestEstimatedPoseFF.isPresent()) {
            EstimatedRobotPose estimatedPose = latestEstimatedPoseFF.get();
            Pose2d pose = estimatedPose.estimatedPose.toPose2d();

            SmartDashboard.putBoolean("Vision/FF/Pose Valid",    true);
            SmartDashboard.putNumber( "Vision/FF/Pose X",        pose.getX());
            SmartDashboard.putNumber( "Vision/FF/Pose Y",        pose.getY());
            SmartDashboard.putNumber( "Vision/FF/Pose Rotation", pose.getRotation().getDegrees());
            SmartDashboard.putNumber( "Vision/FF/Pose Timestamp",estimatedPose.timestampSeconds);
            SmartDashboard.putNumber( "Vision/FF/Tags Used",     estimatedPose.targetsUsed.size());

            StringBuilder tagIds = new StringBuilder();
            for (var target : estimatedPose.targetsUsed) {
                if (tagIds.length() > 0) tagIds.append(", ");
                tagIds.append(target.getFiducialId());
            }
            SmartDashboard.putString("Vision/FF/Tags Used IDs", tagIds.toString());
        } else {
            SmartDashboard.putBoolean("Vision/FF/Pose Valid", false);
        }

        boolean anyPoseValid = latestEstimatedPoseBF.isPresent() || latestEstimatedPoseFF.isPresent();
        SmartDashboard.putBoolean("Vision/Any Pose Valid", anyPoseValid);

        int totalTagsUsed = 0;
        if (latestEstimatedPoseBF.isPresent()) totalTagsUsed += latestEstimatedPoseBF.get().targetsUsed.size();
        if (latestEstimatedPoseFF.isPresent()) totalTagsUsed += latestEstimatedPoseFF.get().targetsUsed.size();
        SmartDashboard.putNumber("Vision/Total Tags Used", totalTagsUsed);
    }

    private void publishCombinedStatus() {
        SmartDashboard.putNumber("Target Tag ID", selectedTagId);

        String detectedTags;
        if (targetVisibleBF && targetVisibleFF) {
            detectedTags = "BF: Tag " + detectedTagIdBF + " | FF: Tag " + detectedTagIdFF;
        } else if (targetVisibleBF) {
            detectedTags = "BF: Tag " + detectedTagIdBF;
        } else if (targetVisibleFF) {
            detectedTags = "FF: Tag " + detectedTagIdFF;
        } else {
            detectedTags = "No Tags Detected";
        }
        SmartDashboard.putString("Detected Tags", detectedTags);

        boolean targetFound = (targetVisibleBF && detectedTagIdBF == selectedTagId) ||
                              (targetVisibleFF && detectedTagIdFF == selectedTagId);
        SmartDashboard.putBoolean("Target Tag Found", targetFound);

        String cameraSeeing;
        if (targetVisibleBF && detectedTagIdBF == selectedTagId &&
            targetVisibleFF && detectedTagIdFF == selectedTagId) {
            cameraSeeing = "Both Cameras";
        } else if (targetVisibleBF && detectedTagIdBF == selectedTagId) {
            cameraSeeing = "Back-Facing Camera";
        } else if (targetVisibleFF && detectedTagIdFF == selectedTagId) {
            cameraSeeing = "Front-Facing Camera";
        } else {
            cameraSeeing = "None";
        }
        SmartDashboard.putString("Target Visible On", cameraSeeing);

        SmartDashboard.putString( "Vision/Active Camera",       getActiveCameraName());
        SmartDashboard.putBoolean("Vision/Any Target Visible",  isAnyTargetVisible());
        SmartDashboard.putNumber( "Vision/Best Target Yaw",     getBestTargetYaw());
        SmartDashboard.putNumber( "Vision/Best Target Distance",getBestTargetDistance());
    }

    // -------------------------------------------------------------------------
    // Priority / fallback camera selection methods
    // -------------------------------------------------------------------------

    public boolean isAnyTargetVisible() {
        return targetVisibleBF || targetVisibleFF;
    }

    public double getBestTargetYaw() {
        if (targetVisibleBF && targetVisibleFF) {
            return targetAreaBF >= targetAreaFF ? targetYawBF : targetYawFF;
        } else if (targetVisibleBF) {
            return targetYawBF;
        } else if (targetVisibleFF) {
            return targetYawFF;
        }
        return 0.0;
    }

    public double getBestTargetDistance() {
        if (targetVisibleBF && targetVisibleFF) {
            return targetAreaBF >= targetAreaFF ? targetDistanceBF : targetDistanceFF;
        } else if (targetVisibleBF) {
            return targetDistanceBF;
        } else if (targetVisibleFF) {
            return targetDistanceFF;
        }
        return 0.0;
    }

    public String getActiveCameraName() {
        if (targetVisibleBF && targetVisibleFF) {
            return targetAreaBF >= targetAreaFF ? "Back-Facing (Primary)" : "Front-Facing (Primary)";
        } else if (targetVisibleBF) {
            return "Back-Facing (Only)";
        } else if (targetVisibleFF) {
            return "Front-Facing (Only)";
        }
        return "None";
    }

    // -------------------------------------------------------------------------
    // Individual camera getters
    // -------------------------------------------------------------------------

    public double getTargetYawBF()      { return targetYawBF; }
    public boolean isTargetVisibleBF()  { return targetVisibleBF; }
    public int getDetectedTagIdBF()     { return detectedTagIdBF; }
    public double getTargetDistanceBF() { return targetDistanceBF; }
    /**
     * Returns all AprilTag IDs currently visible to the Back-Facing camera.
     * Unlike {@link #getDetectedTagIdBF()} (which only tracks the dashboard-selected tag),
     * this list contains EVERY tag the camera sees — used for tower-tag identification.
     */
    public List<Integer> getAllDetectedTagIdsBF() { return allDetectedTagIdsBF; }

    public double getTargetYawFF()      { return targetYawFF; }
    public boolean isTargetVisibleFF()  { return targetVisibleFF; }
    public int getDetectedTagIdFF()     { return detectedTagIdFF; }
    public double getTargetDistanceFF() { return targetDistanceFF; }
    /**
     * Returns all AprilTag IDs currently visible to the Front-Facing camera.
     * Unlike {@link #getDetectedTagIdFF()} (which only tracks the dashboard-selected tag),
     * this list contains EVERY tag the camera sees — used for tower-tag identification.
     */
    public List<Integer> getAllDetectedTagIdsFF() { return allDetectedTagIdsFF; }

    public int getSelectedTagId() { return selectedTagId; }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseBF() { return latestEstimatedPoseBF; }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFF() { return latestEstimatedPoseFF; }

    public void setReferencePose(Pose2d pose) {
        poseEstimatorBF.setReferencePose(new Pose3d(pose));
        poseEstimatorFF.setReferencePose(new Pose3d(pose));
    }
}
