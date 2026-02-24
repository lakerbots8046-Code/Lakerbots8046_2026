package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.units.Units;

/**
 * Centers the robot on an AprilTag using live camera data.
 *
 * Works with BOTH back-facing cameras (BL and BR). Automatically uses whichever
 * camera currently has the best view of the tag (largest target area = closest /
 * most confident). If one camera loses the tag, the other takes over seamlessly.
 *
 * Coordinate conventions for backward-facing cameras (Yaw ≈ 180°):
 *   - Positive yaw  → tag is to the camera's right → robot's LEFT  → strafe left  (+Y)
 *   - Negative yaw  → tag is to the camera's left  → robot's RIGHT → strafe right (-Y)
 *   - Distance too large → drive BACKWARD (-X) to approach the tag
 *   - Distance too small → drive FORWARD  (+X) to back away from the tag
 *
 * Dashboard keys published every loop:
 *   CenterOnTag/Status          – human-readable state string
 *   CenterOnTag/Active Camera   – which camera is providing measurements
 *   CenterOnTag/Yaw Error (deg) – current yaw error from best camera
 *   CenterOnTag/Distance (m)    – current distance from best camera
 *   CenterOnTag/Distance Error  – distance - targetDistance
 *   CenterOnTag/Centered        – true when both yaw and distance are within tolerance
 */
public class CenterOnAprilTagCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    private final double targetDistance;

    // PID controllers
    private final PIDController strafeController;   // drives yaw → 0
    private final PIDController driveController;    // drives distance → targetDistance

    // Robot-centric swerve request (strafe/drive relative to robot heading)
    // OpenLoopVoltage matches the drive request type used by the default teleop drive command
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Yaw tolerance in degrees for "centered" check
    private static final double kYawToleranceDegrees = 2.0;

    /**
     * Creates a CenterOnAprilTagCommand with a custom target distance.
     *
     * @param drivetrain     The swerve drivetrain subsystem
     * @param visionSubsystem The vision subsystem (provides best-camera data)
     * @param targetDistance  Desired distance from the tag in meters
     */
    public CenterOnAprilTagCommand(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem visionSubsystem,
            double targetDistance) {

        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.targetDistance = targetDistance;

        // Strafe PID: drives camera yaw to 0
        strafeController = new PIDController(
            Constants.Vision.kStrafeP,
            Constants.Vision.kStrafeI,
            Constants.Vision.kStrafeD
        );
        strafeController.setTolerance(kYawToleranceDegrees);

        // Drive PID: drives distance to targetDistance
        driveController = new PIDController(
            Constants.Vision.kDriveP,
            Constants.Vision.kDriveI,
            Constants.Vision.kDriveD
        );
        driveController.setTolerance(Constants.Vision.kPositionToleranceMeters);

        addRequirements(drivetrain);
    }

    /**
     * Convenience constructor using the default target distance from Constants.
     */
    public CenterOnAprilTagCommand(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem visionSubsystem) {
        this(drivetrain, visionSubsystem, Constants.Vision.kTargetDistanceMeters);
    }

    @Override
    public void initialize() {
        strafeController.reset();
        driveController.reset();
        SmartDashboard.putString("CenterOnTag/Status", "Initializing");
        SmartDashboard.putBoolean("CenterOnTag/Centered", false);
        SmartDashboard.putBoolean("CenterOnTag/Command Active", true);
        // Log which tag we're looking for
        SmartDashboard.putNumber("CenterOnTag/Looking For Tag", visionSubsystem.getSelectedTagId());
    }

    @Override
    public void execute() {
        // Always publish diagnostic info so we can see what's happening
        boolean bfVisible = visionSubsystem.isTargetVisibleBF();
        boolean ffVisible = visionSubsystem.isTargetVisibleFF();
        SmartDashboard.putBoolean("CenterOnTag/BF Sees Tag", bfVisible);
        SmartDashboard.putBoolean("CenterOnTag/FF Sees Tag", ffVisible);
        SmartDashboard.putBoolean("CenterOnTag/Command Active", true);
        SmartDashboard.putNumber("CenterOnTag/Looking For Tag", visionSubsystem.getSelectedTagId());

        // ── No target visible ────────────────────────────────────────────────
        if (!visionSubsystem.isAnyTargetVisible()) {
            SmartDashboard.putString("CenterOnTag/Status",
                "No Target Visible – BF:" + bfVisible + " FF:" + ffVisible
                + " Tag:" + visionSubsystem.getSelectedTagId());
            SmartDashboard.putBoolean("CenterOnTag/Centered", false);
            return;
        }

        // ── Read best-camera measurements ────────────────────────────────────
        double targetYaw      = visionSubsystem.getBestTargetYaw();
        double currentDist    = visionSubsystem.getBestTargetDistance();
        String activeCamera   = visionSubsystem.getActiveCameraName();
        double distanceError  = currentDist - targetDistance;

        // ── Strafe control (yaw → 0) ─────────────────────────────────────────
        // Cameras face backward (~180°):
        //   positive yaw = tag to camera's right = robot's LEFT → strafe left (+Y)
        // calculate(measurement=0, setpoint=targetYaw) → output = kP * targetYaw
        //   targetYaw > 0 → positive output → +Y velocity → strafe left ✓
        double strafeSpeed = strafeController.calculate(0, targetYaw);

        // ── Drive control (distance → targetDistance) ────────────────────────
        // Cameras face backward: to reduce distance, drive backward (-X).
        // calculate(currentDist, targetDistance) → output = kP*(targetDistance - currentDist)
        //   currentDist > targetDistance → output < 0 → -X velocity → drive backward ✓
        double driveSpeed = driveController.calculate(currentDist, targetDistance);

        // ── Clamp to slower centering speeds (safe for testing) ───────────────
        double maxDrive  = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond)
                           * Constants.Vision.kMaxCenteringDriveSpeed;
        double maxStrafe = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond)
                           * Constants.Vision.kMaxCenteringStrafe;

        strafeSpeed = Math.max(-maxStrafe, Math.min(maxStrafe, strafeSpeed));
        driveSpeed  = Math.max(-maxDrive,  Math.min(maxDrive,  driveSpeed));

        // ── Apply drive request ───────────────────────────────────────────────
        drivetrain.setControl(
            driveRequest
                .withVelocityX(driveSpeed)   // forward/backward (negative = toward tag)
                .withVelocityY(strafeSpeed)  // left/right centering
                .withRotationalRate(0)       // hold current heading
        );

        // ── Dashboard ─────────────────────────────────────────────────────────
        boolean centered = strafeController.atSetpoint() && driveController.atSetpoint();

        // Simple camera label: "BF", "FF", or "Both"
        String cameraLabel;
        if (visionSubsystem.isTargetVisibleBF() && visionSubsystem.isTargetVisibleFF()) {
            cameraLabel = activeCamera.contains("Back") ? "BF (Primary)" : "FF (Primary)";
        } else if (visionSubsystem.isTargetVisibleBF()) {
            cameraLabel = "BF";
        } else {
            cameraLabel = "FF";
        }

        // Top-level simple keys for Elastic widgets
        SmartDashboard.putString("Vision/Centering Camera",     cameraLabel);
        SmartDashboard.putNumber("Vision/Tag Distance (ft)",    currentDist * 3.28084); // meters → feet
        SmartDashboard.putNumber("Vision/Tag Distance (m)",     currentDist);
        SmartDashboard.putBoolean("Vision/Centered",            centered);

        // Detailed keys for debugging
        SmartDashboard.putString("CenterOnTag/Active Camera",   activeCamera);
        SmartDashboard.putNumber("CenterOnTag/Yaw Error (deg)", targetYaw);
        SmartDashboard.putNumber("CenterOnTag/Distance (m)",    currentDist);
        SmartDashboard.putNumber("CenterOnTag/Distance (ft)",   currentDist * 3.28084);
        SmartDashboard.putNumber("CenterOnTag/Distance Error",  distanceError);
        SmartDashboard.putBoolean("CenterOnTag/Centered",       centered);
        SmartDashboard.putNumber("CenterOnTag/Strafe Speed",    strafeSpeed);
        SmartDashboard.putNumber("CenterOnTag/Drive Speed",     driveSpeed);

        String status = centered
            ? "CENTERED [" + cameraLabel + "] @ " + String.format("%.1f ft", currentDist * 3.28084)
            : String.format("Centering – Yaw: %.1f° | %.1f ft [%s]",
                            targetYaw, currentDist * 3.28084, cameraLabel);
        SmartDashboard.putString("CenterOnTag/Status", status);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
        SmartDashboard.putString("CenterOnTag/Status",
            interrupted ? "Interrupted" : "Complete – Centered");
        SmartDashboard.putBoolean("CenterOnTag/Centered", !interrupted);
        SmartDashboard.putBoolean("CenterOnTag/Command Active", false);
    }

    @Override
    public boolean isFinished() {
        // Finish when both yaw and distance are within tolerance
        return visionSubsystem.isAnyTargetVisible()
            && strafeController.atSetpoint()
            && driveController.atSetpoint();
    }
}
