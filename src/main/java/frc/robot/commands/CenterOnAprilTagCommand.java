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

    /** Throttle counter — SmartDashboard writes every 5 execute() calls (~100 ms). */
    private int dashboardCounter = 0;

    /**
     * Rounds a double to the specified number of decimal places for cleaner dashboard display.
     * Position/distance values use 3 decimals; angles and ft distances use 2 decimals.
     */
    private static double round(double value, int decimals) {
        double scale = Math.pow(10, decimals);
        return Math.round(value * scale) / scale;
    }

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
        dashboardCounter = 0;
        SmartDashboard.putString("CenterOnTag/Status", "Initializing");
        SmartDashboard.putBoolean("CenterOnTag/Centered", false);
        SmartDashboard.putBoolean("CenterOnTag/Command Active", true);
        SmartDashboard.putNumber("CenterOnTag/Looking For Tag", visionSubsystem.getSelectedTagId());
    }

    @Override
    public void execute() {
        boolean bfVisible = visionSubsystem.isTargetVisibleBF();
        boolean ffVisible = visionSubsystem.isTargetVisibleFF();

        // ── No target visible ────────────────────────────────────────────────
        if (!visionSubsystem.isAnyTargetVisible()) {
            // Throttled dashboard writes — no motor output needed when no target
            dashboardCounter++;
            if (dashboardCounter >= 5) {
                dashboardCounter = 0;
                SmartDashboard.putBoolean("CenterOnTag/BF Sees Tag", bfVisible);
                SmartDashboard.putBoolean("CenterOnTag/FF Sees Tag", ffVisible);
                SmartDashboard.putBoolean("CenterOnTag/Command Active", true);
                SmartDashboard.putNumber("CenterOnTag/Looking For Tag", visionSubsystem.getSelectedTagId());
                SmartDashboard.putString("CenterOnTag/Status",
                    "No Target Visible – BF:" + bfVisible + " FF:" + ffVisible
                    + " Tag:" + visionSubsystem.getSelectedTagId());
                SmartDashboard.putBoolean("CenterOnTag/Centered", false);
            }
            return;
        }

        // ── Read best-camera measurements ────────────────────────────────────
        double targetYaw     = visionSubsystem.getBestTargetYaw();
        double currentDist   = visionSubsystem.getBestTargetDistance();
        String activeCamera  = visionSubsystem.getActiveCameraName();
        double distanceError = currentDist - targetDistance;

        // ── Strafe control (yaw → 0) ─────────────────────────────────────────
        double strafeSpeed = strafeController.calculate(0, targetYaw);

        // ── Drive control (distance → targetDistance) ────────────────────────
        double driveSpeed = driveController.calculate(currentDist, targetDistance);

        // ── Clamp to slower centering speeds ─────────────────────────────────
        double maxDrive  = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond)
                           * Constants.Vision.kMaxCenteringDriveSpeed;
        double maxStrafe = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond)
                           * Constants.Vision.kMaxCenteringStrafe;

        strafeSpeed = Math.max(-maxStrafe, Math.min(maxStrafe, strafeSpeed));
        driveSpeed  = Math.max(-maxDrive,  Math.min(maxDrive,  driveSpeed));

        // ── Apply drive request (unthrottled — motor control every loop) ──────
        drivetrain.setControl(
            driveRequest
                .withVelocityX(driveSpeed)
                .withVelocityY(strafeSpeed)
                .withRotationalRate(0)
        );

        // ── Throttled dashboard writes (~10 Hz) ───────────────────────────────
        dashboardCounter++;
        if (dashboardCounter >= 5) {
            dashboardCounter = 0;

            boolean centered = strafeController.atSetpoint() && driveController.atSetpoint();

            String cameraLabel;
            if (visionSubsystem.isTargetVisibleBF() && visionSubsystem.isTargetVisibleFF()) {
                cameraLabel = activeCamera.contains("Back") ? "BF (Primary)" : "FF (Primary)";
            } else if (visionSubsystem.isTargetVisibleBF()) {
                cameraLabel = "BF";
            } else {
                cameraLabel = "FF";
            }

            SmartDashboard.putBoolean("CenterOnTag/BF Sees Tag",        bfVisible);
            SmartDashboard.putBoolean("CenterOnTag/FF Sees Tag",        ffVisible);
            SmartDashboard.putBoolean("CenterOnTag/Command Active",     true);
            SmartDashboard.putNumber( "CenterOnTag/Looking For Tag",    visionSubsystem.getSelectedTagId());
            SmartDashboard.putString( "Vision/Centering Camera",        cameraLabel);
            SmartDashboard.putNumber( "Vision/Tag Distance (ft)",       round(currentDist * 3.28084, 2));
            SmartDashboard.putNumber( "Vision/Tag Distance (m)",        round(currentDist, 3));
            SmartDashboard.putBoolean("Vision/Centered",                centered);
            SmartDashboard.putString( "CenterOnTag/Active Camera",      activeCamera);
            SmartDashboard.putNumber( "CenterOnTag/Yaw Error (deg)",    round(targetYaw, 2));
            SmartDashboard.putNumber( "CenterOnTag/Distance (m)",       round(currentDist, 3));
            SmartDashboard.putNumber( "CenterOnTag/Distance (ft)",      round(currentDist * 3.28084, 2));
            SmartDashboard.putNumber( "CenterOnTag/Distance Error",     round(distanceError, 3));
            SmartDashboard.putBoolean("CenterOnTag/Centered",           centered);
            SmartDashboard.putNumber( "CenterOnTag/Strafe Speed",       round(strafeSpeed, 3));
            SmartDashboard.putNumber( "CenterOnTag/Drive Speed",        round(driveSpeed, 3));

            String status = centered
                ? "CENTERED [" + cameraLabel + "] @ " + String.format("%.1f ft", currentDist * 3.28084)
                : String.format("Centering – Yaw: %.1f° | %.1f ft [%s]",
                                targetYaw, currentDist * 3.28084, cameraLabel);
            SmartDashboard.putString("CenterOnTag/Status", status);
        }
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
