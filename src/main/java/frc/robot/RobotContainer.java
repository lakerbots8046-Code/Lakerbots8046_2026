// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.TestMotorSubsystem;
import frc.robot.commands.AimTurretAuto;
import frc.robot.commands.DriveToAprilTag;
import frc.robot.commands.DriveToAprilTagWithPathPlanner;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

public class RobotContainer {
    // Vision subsystem
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    
    // Turret subsystem (CAN ID 6 on RIO bus)
    private final TestMotorSubsystem turretSubsystem = new TestMotorSubsystem();
    
    // Field2D for visualization on dashboard
    private final Field2d field2d = new Field2d();
    
    // Autonomous chooser for dashboard
    private final SendableChooser<Command> autoChooser;
    
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        registerNamedCommands();
        configureBindings();
        
        // Initialize autonomous chooser
        autoChooser = buildAutoChooser();
        
        // Publish Field2D to SmartDashboard for visualization
        SmartDashboard.putData("Field2d", field2d);
        
        // Publish autonomous chooser to SmartDashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("AimTurretAuto",
            new AimTurretAuto(turretSubsystem));
    }
    
    /**
     * Builds the autonomous chooser with available autonomous routines.
     * 
     * @return SendableChooser with autonomous command options
     */
    private SendableChooser<Command> buildAutoChooser() {
        SendableChooser<Command> chooser = new SendableChooser<>();
        
        // Default option: Simple drive forward
        chooser.setDefaultOption("Drive Forward", getDriveForwardAuto());
        
        // PathPlanner autonomous routines
        // Center Auto: Center Part 1 -> AimTurretAuto -> Center Part 2
        try {
            Command centerAuto = AutoBuilder.buildAuto("Center Auto");
            chooser.addOption("Center Auto (PathPlanner)", centerAuto);
        } catch (Exception e) {
            System.err.println("Failed to load PathPlanner auto 'Center Auto': " + e.getMessage());
            // Add a fallback option if PathPlanner auto fails to load
            chooser.addOption("Center Auto (PathPlanner) - FAILED TO LOAD", Commands.none());
        }
        
        // Left Side Auto: AimTurretAuto -> Left Side Part 1
        try {
            Command leftSideAuto = AutoBuilder.buildAuto("Left Side Auto");
            chooser.addOption("Left Side Auto (PathPlanner)", leftSideAuto);
        } catch (Exception e) {
            System.err.println("Failed to load PathPlanner auto 'Left Side Auto': " + e.getMessage());
            // Add a fallback option if PathPlanner auto fails to load
            chooser.addOption("Left Side Auto (PathPlanner) - FAILED TO LOAD", Commands.none());
        }
        
        // Do nothing option (useful for testing)
        chooser.addOption("Do Nothing", Commands.none());
        
        return chooser;
    }
    
    /**
     * Creates the simple drive forward autonomous command.
     * Drives forward at 0.5 m/s for 5 seconds.
     * 
     * @return Command that drives forward
     */
    private Command getDriveForwardAuto() {
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> {
            drivetrain.seedFieldCentric();
            // Also update vision pose estimators with the new pose
            Pose2d currentPose = drivetrain.getState().Pose;
            visionSubsystem.setReferencePose(currentPose);
            SmartDashboard.putString("Field-Centric Status", "Reset at: " + System.currentTimeMillis());
            System.out.println("Field-centric heading reset! Current pose: " + currentPose);
        }));

        // Drive to AprilTag commands
        // Right bumper: Drive to the currently selected tag using PathPlanner (advanced)
        joystick.rightBumper().whileTrue(
            new DriveToAprilTagWithPathPlanner(drivetrain, visionSubsystem, visionSubsystem.getSelectedTagId())
        );
        
        // Right trigger: Drive to selected tag using basic PID (fallback/simple mode)
        joystick.rightTrigger(0.5).whileTrue(
            new DriveToAprilTag(drivetrain, visionSubsystem, visionSubsystem.getSelectedTagId())
        );
        
        // POV buttons for specific tags using PathPlanner (for quick access to common tags)
        // POV Up (0°): Drive to tag 14 (common scoring position)
        joystick.pov(0).whileTrue(new DriveToAprilTagWithPathPlanner(drivetrain, visionSubsystem, 14));
        
        // POV Right (90°): Drive to tag 15
        joystick.pov(90).whileTrue(new DriveToAprilTagWithPathPlanner(drivetrain, visionSubsystem, 15));
        
        // POV Down (180°): Drive to tag 16
        joystick.pov(180).whileTrue(new DriveToAprilTagWithPathPlanner(drivetrain, visionSubsystem, 16));
        
        // POV Left (270°): Drive to tag 13
        joystick.pov(270).whileTrue(new DriveToAprilTagWithPathPlanner(drivetrain, visionSubsystem, 13));

        // Turret control - X button: Hold to spin, release to stop
        joystick.x().whileTrue(
            turretSubsystem.run(() -> turretSubsystem.startSpin())
        ).onFalse(
            turretSubsystem.runOnce(() -> turretSubsystem.stopSpin())
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    
    /**
     * Updates the drivetrain's pose estimator with vision measurements.
     * This should be called periodically (e.g., from Robot.robotPeriodic()).
     */
    public void updateVisionMeasurements() {
        // Process BL camera vision measurement
        var poseBL = visionSubsystem.getEstimatedGlobalPoseBL();
        if (poseBL.isPresent()) {
            var estimatedPose = poseBL.get();
            Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();
            
            // Determine standard deviation based on number of tags
            var stdDevs = estimatedPose.targetsUsed.size() >= 2 
                ? Constants.Vision.kMultiTagStdDevs 
                : Constants.Vision.kSingleTagStdDevs;
            
            // Add vision measurement to drivetrain
            drivetrain.addVisionMeasurement(
                pose2d,
                estimatedPose.timestampSeconds,
                stdDevs
            );
            
            // Publish to dashboard for debugging
            SmartDashboard.putString("Vision/BL/Measurement Status", "Applied");
        } else {
            SmartDashboard.putString("Vision/BL/Measurement Status", "No Pose");
        }
        
        // Process BR camera vision measurement
        var poseBR = visionSubsystem.getEstimatedGlobalPoseBR();
        if (poseBR.isPresent()) {
            var estimatedPose = poseBR.get();
            Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();
            
            // Determine standard deviation based on number of tags
            var stdDevs = estimatedPose.targetsUsed.size() >= 2 
                ? Constants.Vision.kMultiTagStdDevs 
                : Constants.Vision.kSingleTagStdDevs;
            
            // Add vision measurement to drivetrain
            drivetrain.addVisionMeasurement(
                pose2d,
                estimatedPose.timestampSeconds,
                stdDevs
            );
            
            // Publish to dashboard for debugging
            SmartDashboard.putString("Vision/BR/Measurement Status", "Applied");
        } else {
            SmartDashboard.putString("Vision/BR/Measurement Status", "No Pose");
        }
    }
    
    /**
     * Periodic method to update Field2D with robot pose and vision targets.
     * This should be called from Robot.robotPeriodic().
     */
    public void updateField2d() {
        // Update robot pose on field
        Pose2d currentPose = drivetrain.getState().Pose;
        field2d.setRobotPose(currentPose);
        
        // Update AprilTag visualization based on detected tags
        updateAprilTagVisualization();
    }
    
    /**
     * Updates Field2D with detected AprilTag positions.
     * Shows which tags are currently visible to the cameras.
     */
    private void updateAprilTagVisualization() {
        // Get detected tag IDs from both cameras
        int detectedTagBL = visionSubsystem.getDetectedTagIdBL();
        int detectedTagBR = visionSubsystem.getDetectedTagIdBR();
        
        // Clear previous tag poses
        field2d.getObject("Detected Tags BL").setPoses();
        field2d.getObject("Detected Tags BR").setPoses();
        
        // Add BL camera detected tag
        if (detectedTagBL > 0 && visionSubsystem.isTargetVisibleBL()) {
            Optional<Pose2d> tagPose = getAprilTagPose(detectedTagBL);
            if (tagPose.isPresent()) {
                field2d.getObject("Detected Tags BL").setPose(tagPose.get());
            }
        }
        
        // Add BR camera detected tag
        if (detectedTagBR > 0 && visionSubsystem.isTargetVisibleBR()) {
            Optional<Pose2d> tagPose = getAprilTagPose(detectedTagBR);
            if (tagPose.isPresent()) {
                field2d.getObject("Detected Tags BR").setPose(tagPose.get());
            }
        }
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
                // Convert Pose3d to Pose2d (just use X, Y, and rotation around Z)
                var pose3d = tagPose3d.get();
                return Optional.of(new Pose2d(
                    pose3d.getX(),
                    pose3d.getY(),
                    pose3d.getRotation().toRotation2d()
                ));
            }
        } catch (Exception e) {
            // Tag not found in layout
        }
        return Optional.empty();
    }
    
    /**
     * Gets the Field2d object for external access if needed.
     * 
     * @return The Field2d object
     */
    public Field2d getField2d() {
        return field2d;
    }
    
    /**
     * Gets the VisionSubsystem for external access if needed.
     * 
     * @return The VisionSubsystem
     */
    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }

    /**
     * Gets the selected autonomous command from the chooser.
     * 
     * @return The selected autonomous command
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
