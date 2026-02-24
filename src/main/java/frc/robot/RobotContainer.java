// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Spindexer;
import frc.robot.commands.AimTurretAuto;
import frc.robot.commands.CenterOnAprilTagCommand;
import frc.robot.commands.DriveToAprilTag;
import frc.robot.commands.DriveToAprilTagWithPathPlanner;
import frc.robot.commands.ShootOnMoveCommand;
import frc.robot.commands.TrackAprilTagCommand;
import frc.robot.util.ShootingArcManager;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import java.util.Optional;
import java.util.Set;

//import org.photonvision.EstimatedRobotPose;

public class RobotContainer {
    // Vision subsystem
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    
    // Turret subsystem (CAN ID 6 on RIO bus)
    private final TurretSubsystem turretSubsystem = new TurretSubsystem();

    // Intake subsystem
    public static Intake intake = new Intake();

    // Spindexer subsystem
    public static Spindexer spindexer = new Spindexer();

    // Launcher subsystem
    public static Launcher launcher = new Launcher();
    
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

        // ================= INTAKE COLLECT testing buttons =================
        SmartDashboard.putData("Run Intake Collect Positive Voltage", intake.setIntakeRollersVoltage(2));
        SmartDashboard.putData("Run Intake Collect Negative Voltage", intake.setIntakeRollersVoltage(-2));
        SmartDashboard.putData("Stop Intake Collect Voltage", intake.setIntakeRollersVoltage(0));
        
        // ================ INTAKE PIVOT testing buttons ================
        SmartDashboard.putData("Run Intake Pivot Positive Voltage", intake.setIntakePivotVoltage(1));
        SmartDashboard.putData("Run Intake Pivot Negative Voltage", intake.setIntakePivotVoltage(-1));
        SmartDashboard.putData("Stop Intake Pivot Voltage", intake.setIntakePivotVoltage(0));

        // ================ SPINDEXER testing buttons ================
        SmartDashboard.putData("Run Spindexer Positive Voltage", spindexer.setSpindexerVoltage(2));
        SmartDashboard.putData("Run Spindexer Negative Voltage", spindexer.setSpindexerVoltage(-2));
        SmartDashboard.putData("Stop Spindexer Voltage", spindexer.setSpindexerVoltage(0));
    
        // =============== FLAPPY WHEEL FEEDER testing buttons ===============
        SmartDashboard.putData("Run Flappy Wheel Positive Voltage", spindexer.setFlappyWheelVoltage(2));
        SmartDashboard.putData("Run Flappy Wheel Negative Voltage", spindexer.setFlappyWheelVoltage(-2));
        SmartDashboard.putData("Stop Flappy Wheel Voltage", spindexer.setFlappyWheelVoltage(0));

        // ================ FEEDER testing buttons ================
        SmartDashboard.putData("Run Feeder Positive Voltage", spindexer.setFeederVoltage(2));
        SmartDashboard.putData("Run Feeder Negative Voltage", spindexer.setFeederVoltage(-2));
        SmartDashboard.putData("Stop Feeder Voltage", spindexer.setFeederVoltage(0));
        
        // =============== LAUNCHER testing buttons ===============
        SmartDashboard.putData("Run Launcher Positive Voltage", launcher.setLauncherVoltage(2));
        SmartDashboard.putData("Run Launcher Negative Voltage", launcher.setLauncherVoltage(-2));
        SmartDashboard.putData("Stop Launcher Voltage", launcher.setLauncherVoltage(0));

        // ================ TURRET testing buttons ================
        SmartDashboard.putData("Run Turret Positive Voltage", launcher.setTurretVoltage(1));
        SmartDashboard.putData("Run Turret Negative Voltage", launcher.setTurretVoltage(-1));
        SmartDashboard.putData("Stop Turret Voltage", launcher.setTurretVoltage(0));

        // ================ HOOD testing buttons ================
        SmartDashboard.putData("Run Hood Positive Voltage", launcher.setHoodVoltage(1));
        SmartDashboard.putData("Run Hood Negative Voltage", launcher.setHoodVoltage(-1));
        SmartDashboard.putData("Stop Hood Voltage", launcher.setHoodVoltage(0));

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
            // squareInput() is applied to translation axes for finer low-speed control
            drivetrain.applyRequest(() ->
                drive.withVelocityX(squareInput(-joystick.getLeftY()) * MaxSpeed) // Drive forward with squared negative Y (forward)
                    .withVelocityY(squareInput(-joystick.getLeftX()) * MaxSpeed) // Drive left with squared negative X (left)
                    .withRotationalRate(squareInput(-joystick.getRightX()) * MaxAngularRate) // Drive counterclockwise with squared negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // A button: LaunchFromTower shoot command
        // Runs all 4 motors simultaneously while held; all stop on release.
        //   Spindexer (CAN 4): -0.2 V | StarFeeder (CAN 5): +0.2 V
        //   Feeder (CAN 6):    +0.2 V | Launcher (CAN 8):   +0.2 V
        // (Temp test values — update to final values after on-robot testing)
        joystick.a().whileTrue(
            Commands.parallel(
                spindexer.launchFromTower(),
                launcher.launchFromTowerLauncher()
            )
        );
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // View (Back) button: resets field-centric heading
        joystick.back().onTrue(drivetrain.runOnce(() -> {
            drivetrain.seedFieldCentric();
            Pose2d currentPose = drivetrain.getState().Pose;
            visionSubsystem.setReferencePose(currentPose);
            SmartDashboard.putString("Field-Centric Status", "Reset at: " + System.currentTimeMillis());
        }));

        // Left bumper: Toggle intake deploy/collect.
        // First press: deploys pivot to -1.2 rot (Motion Magic) and starts rollers.
        // Second press: stops rollers and retracts pivot to home (-0.1 rot).
        joystick.leftBumper().toggleOnTrue(intake.intakeDeployCollect());

        // Right bumper: Toggle intake rollers on/off independently of pivot.
        // First press stops rollers; second press starts them again.
        // Uses no-subsystem command so it does NOT interrupt the pivot control.
        joystick.rightBumper().onTrue(intake.toggleRollers());

        // Left trigger: Center on selected tag using live camera data (vision-based centering).
        // Uses whichever camera (BL or BR) currently has the best view of the tag.
        // Strafes to make yaw=0 and drives backward to reach target distance.
        joystick.leftTrigger(0.5).whileTrue(
            new CenterOnAprilTagCommand(drivetrain, visionSubsystem)
        );

        // Right trigger: Drive to selected tag using basic PID (odometry-based, fallback/simple mode)
        joystick.rightTrigger(0.5).whileTrue(
            new DriveToAprilTag(drivetrain, visionSubsystem, visionSubsystem.getSelectedTagId())
        );

        // X button: Drive to Tag 10 (Tower), aim turret, then fire.
        // Sequence: PathPlanner → aim turret (3s) → launchFromTower (2s)
        joystick.x().onTrue(buildTag10ShootCommand());
        
        // POV buttons for specific tags using PathPlanner (for quick access to common tags)
        // POV Up (0°): Drive to tag 14 (common scoring position)
        joystick.pov(0).whileTrue(new DriveToAprilTagWithPathPlanner(drivetrain, visionSubsystem, 14));
        
        // POV Right (90°): Drive to tag 15
        joystick.pov(90).whileTrue(new DriveToAprilTagWithPathPlanner(drivetrain, visionSubsystem, 15));
        
        // POV Down (180°): Drive to tag 16
        joystick.pov(180).whileTrue(new DriveToAprilTagWithPathPlanner(drivetrain, visionSubsystem, 16));
        
        // POV Left (270°): Drive to tag 13
        joystick.pov(270).whileTrue(new DriveToAprilTagWithPathPlanner(drivetrain, visionSubsystem, 13));

        // ── Y button: Shoot-on-Arc ────────────────────────────────────────────
        // Phase 1: PathPlanner drives to the nearest shooting position on the arc
        //          around the closest tower (alliance-aware, vision-preferred).
        //          Launcher pre-spins and turret pre-aims during the drive.
        // Phase 2: Robot slides left/right along the arc with the left joystick X.
        //          Turret continuously aims using field-relative pose.
        //          Fires automatically when turret is aimed AND launcher is at speed.
        // Release Y to stop everything.
        joystick.y().whileTrue(buildShootOnArcCommand());

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    
    /**
     * Builds the Tag 10 → aim → shoot sequence command.
     *
     * Step 1: PathPlanner drives to a position 2 m in front of Tag 10 on the Tower.
     *         Tag 10 field position: (12.505, 4.021) facing +X.
     *         Robot shooting position: (~14.5m, 4.021m) facing the Tower.
     *
     * Step 2: TrackAprilTagCommand aims the turret at Tag 10 using vision (3 s timeout).
     *
     * Step 3: launchFromTower() fires all motors for 2 seconds.
     *
     * Dashboard key: "Tag10Shoot/Status"
     */
    private Command buildTag10ShootCommand() {
        final int TAG_10_ID = 10;
        final double SHOOT_OFFSET_M = 2.0;   // meters in front of Tag 10
        final double AIM_TIMEOUT_S  = 3.0;   // max seconds to aim turret
        final double FIRE_DURATION_S = 2.0;  // seconds to run launchFromTower

        return Commands.sequence(
            // ── Step 1: Drive to shooting position near Tag 10 ──────────────
            Commands.runOnce(() ->
                SmartDashboard.putString("Tag10Shoot/Status", "Step 1: Driving to Tag 10")),
            new DriveToAprilTagWithPathPlanner(drivetrain, visionSubsystem, TAG_10_ID, SHOOT_OFFSET_M),

            // ── Step 2: Aim turret at Tag 10 using vision ───────────────────
            Commands.runOnce(() ->
                SmartDashboard.putString("Tag10Shoot/Status", "Step 2: Aiming Turret")),
            new TrackAprilTagCommand(turretSubsystem, visionSubsystem, TAG_10_ID)
                .withTimeout(AIM_TIMEOUT_S),

            // ── Step 3: Fire using launchFromTower ──────────────────────────
            Commands.runOnce(() ->
                SmartDashboard.putString("Tag10Shoot/Status", "Step 3: Firing")),
            Commands.parallel(
                spindexer.launchFromTower(),
                launcher.launchFromTowerLauncher()
            ).withTimeout(FIRE_DURATION_S),

            Commands.runOnce(() ->
                SmartDashboard.putString("Tag10Shoot/Status", "Complete"))
        );
    }

    /**
     * Updates the drivetrain's pose estimator with vision measurements.
     * This should be called periodically (e.g., from Robot.robotPeriodic()).
     */
    public void updateVisionMeasurements() {
        // ── Back Facing camera: pose estimation DISABLED for now ─────────────
        // BF camera is reserved for target tracking/measurements only.
        // Re-enable this block when BF camera pose estimation is needed.
        // var poseBF = visionSubsystem.getEstimatedGlobalPoseBF();
        // if (poseBF.isPresent()) {
        //     var estimatedPose = poseBF.get();
        //     Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();
        //     var stdDevs = estimatedPose.targetsUsed.size() >= 2
        //         ? Constants.Vision.kMultiTagStdDevs
        //         : Constants.Vision.kSingleTagStdDevs;
        //     drivetrain.addVisionMeasurement(pose2d, estimatedPose.timestampSeconds, stdDevs);
        //     SmartDashboard.putString("Vision/BF/Measurement Status", "Applied");
        // } else {
        //     SmartDashboard.putString("Vision/BF/Measurement Status", "No Pose");
        // }
        SmartDashboard.putString("Vision/BF/Measurement Status", "Disabled (FF only mode)");

        // ── Front Facing camera: ACTIVE pose estimator ───────────────────────
        var poseFF = visionSubsystem.getEstimatedGlobalPoseFF();
        if (poseFF.isPresent()) {
            var estimatedPose = poseFF.get();
            Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();

            // Use tighter std devs for multi-tag, looser for single tag
            var stdDevs = estimatedPose.targetsUsed.size() >= 2
                ? Constants.Vision.kMultiTagStdDevs
                : Constants.Vision.kSingleTagStdDevs;

            // Feed FF pose estimate into the drivetrain Kalman filter
            drivetrain.addVisionMeasurement(
                pose2d,
                estimatedPose.timestampSeconds,
                stdDevs
            );

            SmartDashboard.putString("Vision/FF/Measurement Status", "Applied");
        } else {
            SmartDashboard.putString("Vision/FF/Measurement Status", "No Pose");
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
        int detectedTagBF = visionSubsystem.getDetectedTagIdBF();
        int detectedTagFF = visionSubsystem.getDetectedTagIdFF();
        
        // Clear previous tag poses
        field2d.getObject("Detected Tags BF").setPoses();
        field2d.getObject("Detected Tags FF").setPoses();
        
        // Add Back Facing camera detected tag
        if (detectedTagBF > 0 && visionSubsystem.isTargetVisibleBF()) {
            Optional<Pose2d> tagPose = getAprilTagPose(detectedTagBF);
            if (tagPose.isPresent()) {
                field2d.getObject("Detected Tags BF").setPose(tagPose.get());
            }
        }
        
        // Add Front Facing camera detected tag
        if (detectedTagFF > 0 && visionSubsystem.isTargetVisibleFF()) {
            Optional<Pose2d> tagPose = getAprilTagPose(detectedTagFF);
            if (tagPose.isPresent()) {
                field2d.getObject("Detected Tags FF").setPose(tagPose.get());
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
     * Applies a squared curve to a joystick input while preserving the sign.
     * This gives finer control at low speeds while still allowing full speed
     * at full joystick deflection.
     *
     * <p>Formula: {@code Math.copySign(value * value, value)}
     *
     * <p>Examples:
     * <ul>
     *   <li>0.0  → 0.0  (no movement)</li>
     *   <li>0.5  → 0.25 (half stick = quarter speed)</li>
     *   <li>1.0  → 1.0  (full stick = full speed)</li>
     *   <li>-0.5 → -0.25 (sign preserved)</li>
     * </ul>
     *
     * @param value Raw joystick axis value in the range [-1.0, 1.0]
     * @return Squared value with the original sign, in the range [-1.0, 1.0]
     */
    private double squareInput(double value) {
        return Math.copySign(value * value, value);
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

    // =========================================================================
    // Shoot-on-Arc helpers
    // =========================================================================

    /**
     * Builds the full shoot-on-arc command sequence bound to the Y button.
     *
     * <h3>Phase 1 — Drive to arc (PathPlanner)</h3>
     * <ul>
     *   <li>Determines the active tower tag (vision-preferred, then alliance fallback).</li>
     *   <li>Calculates the nearest shooting position on the arc at
     *       {@link Constants.ShootingArc#kPreferredShootingDistance} from the tower.</li>
     *   <li>PathPlanner generates an on-the-fly path and drives there.</li>
     *   <li>Launcher pre-spins to the distance-based target RPS during the drive.</li>
     *   <li>Turret pre-aims at the tower during the drive.</li>
     * </ul>
     *
     * <h3>Phase 2 — Shoot on arc (ShootOnMoveCommand)</h3>
     * <ul>
     *   <li>Left joystick X slides the robot left/right along the arc.</li>
     *   <li>Robot heading is locked to face the tower.</li>
     *   <li>Turret continuously aims using field-relative pose (not camera yaw).</li>
     *   <li>Fires automatically when turret is aimed AND launcher is at speed.</li>
     * </ul>
     *
     * <p>Releasing the Y button interrupts the sequence and stops all mechanisms.
     */
    private Command buildShootOnArcCommand() {
        // TESTING SPEEDS — slow approach for initial on-robot testing.
        // Increase Constants.ShootingArc.kApproach* values once behavior is confirmed.
        PathConstraints constraints = new PathConstraints(
            Constants.ShootingArc.kApproachMaxVelocity,           // 1.0 m/s (testing)
            Constants.ShootingArc.kApproachMaxAcceleration,       // 0.75 m/s² (testing)
            Constants.ShootingArc.kApproachMaxAngularVelocity,    // π/2 rad/s (testing)
            Constants.ShootingArc.kApproachMaxAngularAcceleration // π/2 rad/s² (testing)
        );

        return Commands.sequence(
            // ── Phase 1: Drive to nearest arc position ────────────────────────
            Commands.runOnce(() -> {
                SmartDashboard.putString("ShootArc/Status", "Phase 1: Driving to Arc");
                SmartDashboard.putNumber("ShootArc/Tower Tag ID", getActiveTowerTagId());
            }),
            Commands.deadline(
                // PathPlanner drives to the arc (deadline — ends when path is done)
                Commands.defer(() -> {
                    int tagId = getActiveTowerTagId();
                    var nearestPos = ShootingArcManager.getNearestShootingPose(
                        drivetrain.getState().Pose, tagId);
                    SmartDashboard.putNumber("ShootArc/Target X", nearestPos.getX());
                    SmartDashboard.putNumber("ShootArc/Target Y", nearestPos.getY());
                    return AutoBuilder.pathfindToPose(nearestPos, constraints, 0.0);
                }, Set.of(drivetrain)),

                // Pre-spin flywheel AND pre-aim turret (CAN 7) during drive.
                // Both are in the Launcher subsystem — combined into one Commands.run()
                // to avoid a subsystem conflict between two parallel commands.
                Commands.run(() -> {
                    int tagId = getActiveTowerTagId();
                    var tower = ShootingArcManager.getTowerCenter(tagId);
                    var robotPose = drivetrain.getState().Pose;
                    double dist = ShootingArcManager.calculateDistance(robotPose, tower);

                    // Pre-spin flywheel to distance-based target RPS
                    launcher.setCollectVelocity(ShootingArcManager.calculateLauncherRPS(dist));

                    // Pre-aim turret using Motion Magic (raw motor rotations)
                    double targetAngleDeg = ShootingArcManager.calculateTurretAngle(robotPose, tower);
                    double targetRaw = targetAngleDeg * Constants.TurretConstants.kRotationsPerDegree;
                    // Clamp to physical limits before sending to motor
                    targetRaw = Math.max(-Constants.TurretConstants.kPhysicalLimitRotations,
                                Math.min( Constants.TurretConstants.kPhysicalLimitRotations, targetRaw));
                    launcher.setTurretPosition(targetRaw);
                }, launcher)
            ),

            // ── Phase 2: Shoot on arc ─────────────────────────────────────────
            Commands.runOnce(() ->
                SmartDashboard.putString("ShootArc/Status", "Phase 2: Shooting on Arc")),
            new ShootOnMoveCommand(
                drivetrain,
                launcher,    // turret (CAN 7) + flywheel (CAN 8) + hood (CAN 9)
                spindexer,
                this::getActiveTowerTagId,
                () -> -joystick.getLeftX()   // negative = right stick = clockwise arc
            )
        )
        // ── Safety net: stop all motors when Y is released at ANY point ───────
        // ShootOnMoveCommand.end() handles Phase 2 cleanup correctly.
        // This finallyDo() covers Phase 1 interruption (PathPlanner + pre-spin
        // have no built-in cleanup) and is harmless if called after Phase 2.
        .finallyDo(() -> {
            launcher.stopTurretDirect();
            launcher.stopLauncher();
            spindexer.stopFeedMotorsDirect();
            drivetrain.setControl(new SwerveRequest.Idle());
            SmartDashboard.putBoolean("ShootArc/Firing", false);
            SmartDashboard.putString("ShootArc/Status", "Stopped (Y released)");
        });
    }

    /**
     * Returns the best tower tag ID to use for shooting.
     *
     * <p>Priority:
     * <ol>
     *   <li>Any tower tag currently visible to either camera (most accurate).</li>
     *   <li>Alliance-based primary tag from {@link DriverStation#getAlliance()}.</li>
     *   <li>Red primary tag (Tag 10) as final fallback.</li>
     * </ol>
     */
    private int getActiveTowerTagId() {
        // 1. Prefer a tag that is currently visible
        int visibleTag = getVisibleTowerTag();
        if (visibleTag > 0) return visibleTag;

        // 2. Fall back to alliance-based selection
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return Constants.ShootingArc.kBluePrimaryTagId;
        }
        return Constants.ShootingArc.kRedPrimaryTagId;
    }

    /**
     * Scans both cameras for any currently-visible tower tag.
     *
     * @return The first tower tag ID found, or {@code -1} if none are visible.
     */
    private int getVisibleTowerTag() {
        // All tower tag IDs across both alliances
        int[] allTowerTags = {9, 10, 11, 19, 20, 21};

        // Check Back-Facing camera first
        int bfTag = visionSubsystem.getDetectedTagIdBF();
        for (int tag : allTowerTags) {
            if (tag == bfTag) return tag;
        }

        // Check Front-Facing camera
        int ffTag = visionSubsystem.getDetectedTagIdFF();
        for (int tag : allTowerTags) {
            if (tag == ffTag) return tag;
        }

        return -1; // No tower tag visible
    }

    // =========================================================================
    // Getters
    // =========================================================================

    /**
     * Gets the selected autonomous command from the chooser.
     * 
     * @return The selected autonomous command
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
