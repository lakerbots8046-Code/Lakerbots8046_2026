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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
//import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Climber;
import frc.robot.commands.AutoDeployIntake;
import frc.robot.commands.ClimberGoToSetpoint;
//import frc.robot.commands.CenterOnAprilTagCommand;
//import frc.robot.commands.DriveToAprilTag;
import frc.robot.commands.FeedFromCenterCommand;
import frc.robot.commands.ShootFromPointCommand;
import frc.robot.commands.ShootOnMoveCommand;
import frc.robot.subsystems.LEDSubsystem;
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
    
    // Intake subsystem
    public static Intake intake = new Intake();

    // Spindexer subsystem
    public static Spindexer spindexer = new Spindexer();

    // Launcher subsystem
    public static Launcher launcher = new Launcher();

    // Climber subsystem
    public static Climber climber = new Climber();

    // LED subsystem (WPILib AddressableLED — periodic() auto-called by CommandScheduler)
    public static LEDSubsystem leds = new LEDSubsystem();
    
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

    private final CommandXboxController joystick =
        new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
    private final CommandXboxController operatorController =
        new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /** Throttle counter for vision status string puts (updated every 5 calls ≈ 100 ms). */
    private int visionStatusCounter = 0;

    /** Throttle counter for tower tag info puts (updated every 5 calls ≈ 100 ms). */
    private int towerTagCounter = 0;

    /** Throttle counter for FMS info puts (updated every 5 calls ≈ 100 ms). */
    private int fmsInfoCounter = 0;

    // ── Tower tag hold state to survive brief vision dropouts ────────────────
    /** Last valid alliance-matching tower tag seen by any camera. */
    private int lastSeenTowerTagId = -1;
    /** FPGA timestamp (s) when {@link #lastSeenTowerTagId} was last updated. */
    private double lastSeenTowerTagTimestampSec = -1.0;
    /** Human-readable source for telemetry: Visible / Held / AllianceFallback. */
    private String towerTagSource = "AllianceFallback";
    /** How long to hold a recently seen tag before alliance fallback (seconds). */
    private static final double kTowerTagHoldSeconds = 0.75;

    public RobotContainer() {
        registerNamedCommands();
        configureBindings();

        // Hood default command: retract to position 0 (22° / stowed) whenever no
        // shooting command is active. WPILib automatically resumes this command
        // whenever ShootOnMoveCommand or ShootFromPointCommand ends.
        launcher.setDefaultCommand(launcher.retractHood(intake::isPivotAtHome));
        
        // Initialize autonomous chooser
        autoChooser = buildAutoChooser();
        
        // Publish Field2D to SmartDashboard for visualization
        SmartDashboard.putData("Field2d", field2d);

        // Publish autonomous chooser to SmartDashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Turret auto-aim toggle — set to true to enable pose-based auto-aiming,
        // false to use manual D-pad control (1° per tap via Motion Magic).
        // Eventually this will be mapped to an Xbox controller button.
        SmartDashboard.putBoolean("Turret/Auto Aim Enabled", true);

        // LED master toggle for Elastic/SmartDashboard:
        // true  = LEDs run normal subsystem animations/state logic
        // false = LEDs forced OFF (black)
        SmartDashboard.putBoolean("LED/Enabled", true);

        // Pre-seed ShootOnMove dashboard keys so they are visible in Elastic
        // even when the ShootOnMove command is not currently running.
        ShootOnMoveCommand.seedDashboardKeys();
/* 
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
*/
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("AutoDeployIntake",
            new AutoDeployIntake(intake));

        NamedCommands.registerCommand("ShootOnMoveCommand",
            new ShootOnMoveCommand(
                drivetrain,
                launcher,
                spindexer,
                this::getActiveTowerTagId,
                () -> -joystick.getLeftX()
            )
        );
        
        NamedCommands.registerCommand("ShootFromPointCommand",
            new ShootFromPointCommand(
                drivetrain,
                launcher,
                spindexer,
                this::getActiveTowerTagId,
                () -> {
                    int pov = joystick.getHID().getPOV();
                    if (pov == 270) return  1.0; // POV Left  → nudge CCW
                    if (pov == 90)  return -1.0; // POV Right → nudge CW
                    return 0.0;
                },
                intake
             ));

        NamedCommands.registerCommand("IntakeCollectCommand",
            intake.intakeDeployCollect());

        NamedCommands.registerCommand("toggleIntakeRollers", 
            intake.toggleRollers());

        NamedCommands.registerCommand("FeedFromCenterCommand",
            new FeedFromCenterCommand(drivetrain, launcher, spindexer));

        NamedCommands.registerCommand("AutoIntakeDeployCollect3Secs",
            intake.AutoIntakeDeployCollect3secs());

        NamedCommands.registerCommand("AutoIntakeDeployCollect2Secs",
            intake.AutoIntakeDeployCollect2secs());

        NamedCommands.registerCommand("AutoIntakeDeployCollect8Secs",
            intake.AutoIntakeDeployCollect8secs());

         NamedCommands.registerCommand("AutoIntakeDeployCollect6Secs",
            intake.AutoIntakeDeployCollect6secs());

        NamedCommands.registerCommand("DumpAndReturn", 
            intake.dumpAndReturn());

        NamedCommands.registerCommand("ClimberReach",
            new ClimberGoToSetpoint(climber, Climber.Setpoint.Reach));

        NamedCommands.registerCommand("ClimberPullDown", 
            new ClimberGoToSetpoint(climber, Climber.Setpoint.YButton));

        NamedCommands.registerCommand("IntakeHome",
            Commands.sequence(
                // Move turret to mechanical zero (raw rotations) first.
                Commands.runOnce(() -> launcher.setTurretPosition(0.0)),
                // Wait until turret is near zero before homing intake pivot.
                Commands.waitUntil(() -> Math.abs(launcher.getTurretPosition()) <= 0.003).withTimeout(2.0),
                // Then home the intake pivot.
                intake.goToPivotPosition(Constants.IntakeConstants.kPivotHomePosition)
            ).withName("IntakeHome"));
    
    }
    
    /**
     * Builds the autonomous chooser with all PathPlanner auto routines found in
     * {@code src/main/deploy/pathplanner/autos/}.
     *
     * <p>Auto file names (without .auto extension) must match exactly what
     * PathPlanner saved. Each entry is wrapped in a try/catch so a single
     * missing or broken file does not prevent the others from loading.
     *
     * <p>Available autos (as of last update):
     * <ul>
     *   <li>centerDepotClimb  — start center, score depot, climb</li>
     *   <li>leftNeutralFeed   — start left, neutral zone feed</li>
     *   <li>leftNeutralScore  — start left, neutral zone score</li>
     *   <li>rightNeutralFeed  — start right, neutral zone feed</li>
     *   <li>rightNeutralScore — start right, neutral zone score</li>
     *   <li>rightOutpostScore — start right, outpost score</li>
     * </ul>
     *
     * @return SendableChooser populated with all available autonomous commands
     */
    private SendableChooser<Command> buildAutoChooser() {
        SendableChooser<Command> chooser = new SendableChooser<>();

        // ── Safe fallbacks (always available) ────────────────────────────────
        chooser.setDefaultOption("Do Nothing", Commands.none());
        chooser.addOption("Drive Forward", getDriveForwardAuto());

        // ── PathPlanner autos ─────────────────────────────────────────────────
        // Auto names must match the .auto file names in deploy/pathplanner/autos/
        String[] autoNames = {
        // CENTER AUTOS
            "[CENTER] Depot + Climb",
            
        // RIGHT AUTOS
            // "rightNeutralFeed",
            "[RIGHT] Double Neutral",
            // "rightOutpostScore",
            // "rightNeutralCross",

        // LEFT AUTOS
            "[LEFT] Double Neutral",
            "[LEFT] Cross Field",
            // "leftNeutralFeed",
            // "leftNeutralScore",
            // "leftNeutralBumpScore",
            // "LeftNeutralScoreV2",


            // "CrossField",
            // "CrossFieldLeft",
            // "CrossFieldBackInLeft"
            
        };

        for (String autoName : autoNames) {
            try {
                Command auto = AutoBuilder.buildAuto(autoName);
                chooser.addOption(autoName, auto);
                System.out.println("Auto loaded: " + autoName);
            } catch (Exception e) {
                System.err.println("Failed to load auto '" + autoName + "': " + e.getMessage());
                chooser.addOption(autoName + " (FAILED)", Commands.none());
            }
        }

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
            // Drivetrain will execute this command periodically.
            // When ShootOnMove is active, translation is gated to a constant speed
            // (direction-only from joystick, no magnitude scaling).
            drivetrain.applyRequest(() -> {
                double inputY = -joystick.getLeftY(); // forward/back
                double inputX = -joystick.getLeftX(); // left/right

                double vx;
                double vy;

                if (ShootOnMoveCommand.isShootOnMoveActive()) {
                    double deadband = Constants.ShootingArc.kShootOnMoveDriveDirectionDeadband;
                    double maxShootOnMoveSpeed = Constants.ShootingArc.kShootOnMoveDriveMaxSpeedMps;

                    double xCmd = Math.abs(inputY) > deadband ? inputY : 0.0;
                    double yCmd = Math.abs(inputX) > deadband ? inputX : 0.0;

                    // Scaled translation while ShootOnMove is active, capped by maxShootOnMoveSpeed.
                    vx = xCmd * maxShootOnMoveSpeed;
                    vy = yCmd * maxShootOnMoveSpeed;
                } else {
                    // Normal driver shaping outside ShootOnMove.
                    vx = squareInput(inputY) * MaxSpeed;
                    vy = squareInput(inputX) * MaxSpeed;
                }

                return drive.withVelocityX(vx)
                    .withVelocityY(vy)
                    .withRotationalRate((-joystick.getRightX()) * MaxAngularRate);
            })
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
        // Operator A: while held, run spindexer outtake (dejam Spindexer).
        operatorController.a().whileTrue(spindexer.runSpindexerOuttake());

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
        joystick.start().onTrue(drivetrain.runOnce(() -> {
            drivetrain.seedFieldCentric();
            Pose2d currentPose = drivetrain.getState().Pose;
            visionSubsystem.setReferencePose(currentPose);
            SmartDashboard.putString("Field-Centric Status", "Reset at: " + System.currentTimeMillis());
        }));

        // Right bumper: start intake deploy/collect (non-toggle).
        // Single press deploys pivot and starts rollers; command remains active
        // until interrupted by another intake command.
        joystick.rightBumper().onTrue(intake.intakeDeployCollect());
        operatorController.rightBumper().toggleOnTrue(intake.intakeDeployCollect());

        // Left bumper: Dump-and-return sequence.
        // Lifts the intake pivot to the dump position (-0.75 rot), holds for 0.5 s,
        // then returns to the deploy/collect position (-1.35 rot).
        // Requires the intake subsystem — will interrupt intakeDeployCollect() while running.
        // Re-press right bumper after this command finishes to resume normal intake.
        joystick.leftBumper().onTrue(intake.dumpAndReturn());
        operatorController.leftBumper().whileTrue(intake.runOuttake());

        // Driver X: toggle intake rollers on/off independently of pivot.
        joystick.x().onTrue(intake.toggleRollers());

        // Operator X: reset intake pivot encoder position to home reference.
        operatorController.x().onTrue(intake.resetPivotEncoderCommand());

        // Left trigger (any press, 0.3+): Shoot from current field position.
        // Aims turret, sets hood angle, spins flywheel — all based on distance to tower.
        // Fires (LaunchSequenceOneCommand) once flywheel is at speed.
        // Robot does NOT move — drivetrain is untouched.
        //
        // When "Turret/Auto Aim Enabled" = true  (SmartDashboard):
        //   Turret auto-aims using pose-based calculation.
        //   POV Left (270°) / POV Right (90°) apply a fine-tune offset while held.
        //
        // When "Turret/Auto Aim Enabled" = false (SmartDashboard):
        //   Turret holds its current position.
        //   POV Left (270°) → tap moves turret 1° CCW via Motion Magic.
        //   POV Right (90°) → tap moves turret 1° CW  via Motion Magic.
        operatorController.rightTrigger(0.3).whileTrue(intake.runOuttake());
        operatorController.leftTrigger(0.3).whileTrue(
            new ShootFromPointCommand(
                drivetrain,
                launcher,
                spindexer,
                this::getActiveTowerTagId,
                () -> {
                    int pov = operatorController.getHID().getPOV();
                    if (pov == 270) return  1.0; // POV Left  → CCW
                    if (pov == 90)  return -1.0; // POV Right → CW
                    return 0.0;
                },
                intake
            )
        );

        // Right trigger: Drive to selected tag using basic PID (odometry-based, fallback/simple mode)
       
        /*joystick.rightTrigger(0.5).whileTrue(
            new DriveToAprilTag(drivetrain, visionSubsystem, visionSubsystem.getSelectedTagId())
        );
        */


        // Driver right trigger (hard press, 0.75+): Shoot on move.
        joystick.leftTrigger(0.75).whileTrue(
            new ShootOnMoveCommand(
                drivetrain,
                launcher,
                spindexer,
                this::getActiveTowerTagId,
                () -> -joystick.getLeftX()
            )
        );

        // Operator right trigger remains Feed from center.
        joystick.rightTrigger(0.75).whileTrue(
            new FeedFromCenterCommand(drivetrain, launcher, spindexer)
        );


        // ── D-pad (POV) bindings ──────────────────────────────────────────────
        //
        // POV Left (270°) / POV Right (90°): Manual turret nudge — 1° per tap.
        //   • When NOT shooting: calls nudgeTurretDirect() directly (no subsystem conflict).
        //   • When shooting (left trigger held): ShootFromPointCommand reads the same
        //     POV supplier and handles the nudge internally (auto-aim OFF = 1°/tap via
        //     Motion Magic; auto-aim ON = continuous fine-tune offset).
        //
        // POV Up (0°) / POV Down (180°): Bump intake pivot position.
        //   Active only when NOT shooting so they don't interfere with the trigger.

        // POV Left (270°): nudge turret 1° CCW — when NOT shooting
        joystick.pov(270).and(joystick.leftTrigger(0.3).negate())
            .onTrue(Commands.runOnce(() -> launcher.nudgeTurretDirect(1.0)));
        operatorController.pov(270).and(operatorController.leftTrigger(0.3).negate())
            .onTrue(Commands.runOnce(() -> launcher.nudgeTurretDirect(1.0)));

        // POV Right (90°): nudge turret 1° CW — when NOT shooting
        joystick.pov(90).and(joystick.leftTrigger(0.3).negate())
            .onTrue(Commands.runOnce(() -> launcher.nudgeTurretDirect(-1.0)));
        operatorController.pov(90).and(operatorController.leftTrigger(0.3).negate())
            .onTrue(Commands.runOnce(() -> launcher.nudgeTurretDirect(-1.0)));

        // POV Up (0°): Bump intake pivot position up by kPivotBumpFactor
        joystick.pov(0).and(joystick.leftTrigger(0.3).negate())
            .onTrue(intake.bumpPivotUp());

        // Operator POV Up (0°): Climber to high setpoint (Reach)
        operatorController.pov(0).and(operatorController.leftTrigger(0.3).negate())
            .onTrue(climber.goToSetpoint(() -> Climber.Setpoint.Reach));

        // POV Down (180°): Bump intake pivot position down by kPivotBumpFactor
        joystick.pov(180).and(joystick.leftTrigger(0.3).negate())
            .onTrue(intake.bumpPivotDown());

        // Operator POV Down (180°): Climber to lower setpoint (PullDown)
        operatorController.pov(180).and(operatorController.leftTrigger(0.3).negate())
            .onTrue(climber.goToSetpoint(() -> Climber.Setpoint.PullDown));

        // Operator B: while held, move intake pivot opposite deploy direction (upward) at low speed.
        operatorController.b().whileTrue(intake.raiseIntakeManualLowSpeed());

        // ── Y button: Shoot-on-Arc ────────────────────────────────────────────
        // Phase 1: PathPlanner drives to the nearest shooting position on the arc
        //          around the closest tower (alliance-aware, vision-preferred).
        //          Launcher pre-spins and turret pre-aims during the drive.
        // Phase 2: Robot slides left/right along the arc with the left joystick X.
        //          Turret continuously aims using field-relative pose.
        //          Fires automatically when turret is aimed AND launcher is at speed.
        // Release Y to stop everything.
        joystick.y().whileTrue(buildShootOnArcCommand());
        
        operatorController.y().onTrue(
            climber.goToSetpoint(() -> Climber.Setpoint.YButton)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    
    /**
     * Publishes tower tag identification and distance info to SmartDashboard/Elastic.
     *
     * <p>Published under the {@code Tower/} namespace so the widgets are always
     * visible in Elastic regardless of whether a shooting command is active.
     * Throttled to every 5 calls (~100 ms) to reduce NetworkTables load.
     *
     * <p>Call this from {@code Robot.robotPeriodic()}.
     *
     * <h3>Elastic widget suggestions</h3>
     * <ul>
     *   <li>{@code Tower/Tag Info}     — Text Display (shows "Tag 10 (Red Tower ★)")</li>
     *   <li>{@code Tower/Alliance}     — Text Display with conditional color</li>
     *   <li>{@code Tower/Distance (m)} — Number Display or Gauge</li>
     *   <li>{@code Tower/In Zone}      — Boolean Box (green = in range)</li>
     * </ul>
     */
    public void updateTowerTagInfo() {
        towerTagCounter++;
        if (towerTagCounter < 1) return;  // Reduced latency: 100ms→20ms (50Hz tracking updates)
        towerTagCounter = 0;

        int           tagId      = getActiveTowerTagId();
        var           robotPose  = drivetrain.getState().Pose;
        var           tower      = ShootingArcManager.getTowerCenter(robotPose, tagId);
        double        distance   = ShootingArcManager.calculateDistance(robotPose, tower);
        boolean       inZone     = ShootingArcManager.isInShootingZone(robotPose, tagId);

        String tagAlliance = ShootingArcManager.isRedTowerTag(tagId)  ? "Red"
                           : ShootingArcManager.isBlueTowerTag(tagId) ? "Blue"
                           : "Unknown";
        boolean isPrimary  = (tagId == Constants.ShootingArc.kRedPrimaryTagId)
                           || (tagId == Constants.ShootingArc.kBluePrimaryTagId);
        // e.g. "Tag 10 (Red Tower ★)" or "Tag 9 (Red Tower)"
        String tagInfo     = String.format("Tag %d (%s Tower%s)",
                                tagId, tagAlliance, isPrimary ? " \u2605" : "");

        SmartDashboard.putNumber( "Tower/Tag ID",        tagId);
        SmartDashboard.putString( "Tower/Tag Info",      tagInfo);
        SmartDashboard.putString( "Tower/Alliance",      tagAlliance);
        SmartDashboard.putNumber( "Tower/Distance (m)",  round(distance, 3));
        SmartDashboard.putBoolean("Tower/In Zone",       inZone);

        // ── Pre-seed locked tag widgets so they appear in Elastic immediately ─
        // When no shooting command is active, these show the tag that WOULD be
        // locked if the trigger were pressed right now. The commands overwrite
        // these values with the actual locked tag once they start running.
        SmartDashboard.putNumber("ShootFromPoint/Locked Tag ID", tagId);
        SmartDashboard.putNumber("ShootArc/Locked Tag ID",       tagId);

        // ── Turret zero offset — always visible for runtime tuning ────────────
        // Published here so the widget appears in Elastic even before
        // ShootFromPointCommand runs. ShootFromPointCommand reads this value
        // every ~100 ms and applies it to the raw turret angle.
        // Change this number in Elastic to nudge the turret without redeploying.
        // Positive = nudge CCW (left), Negative = nudge CW (right).
        // The value is seeded from kTurretZeroOffsetDegrees on first boot;
        // after that, whatever you type in Elastic is used live.
        double liveOffset = SmartDashboard.getNumber(
                "ShootFromPoint/Turret Zero Offset (deg)",
                Constants.TurretConstants.kTurretZeroOffsetDegrees);
        SmartDashboard.putNumber("ShootFromPoint/Turret Zero Offset (deg)", liveOffset);

        // ── Always-on turret monitoring ───────────────────────────────────────
        // These mirror the ShootFromPoint/ widgets so they are visible in Elastic
        // at all times — even when ShootFromPointCommand is not running.
        // Useful for pre-shot verification and live tuning while driving.
        double rawAngle = ShootingArcManager.calculateTurretAngleRaw(robotPose, tower);
        double targetAngle = rawAngle - liveOffset;
        while (targetAngle >  180.0) targetAngle -= 360.0;
        while (targetAngle < -180.0) targetAngle += 360.0;
        double currentTurretDeg = launcher.getTurretPositionDegrees();
        double turretError = targetAngle - currentTurretDeg;
        while (turretError >  180.0) turretError -= 360.0;
        while (turretError < -180.0) turretError += 360.0;

        SmartDashboard.putNumber("ShootFromPoint/Turret Raw Angle (deg)", rawAngle);
        SmartDashboard.putNumber("ShootFromPoint/Turret Error (deg)",     turretError);
        SmartDashboard.putNumber("ShootFromPoint/Distance (m)",           round(distance, 3));
        SmartDashboard.putString("ShootFromPoint/Tower Tag Info",         tagInfo);
        SmartDashboard.putNumber("ShootFromPoint/Turret Target (deg)",    targetAngle);
        SmartDashboard.putNumber("ShootFromPoint/Turret Position (deg)",  currentTurretDeg);

        // ── Always-on turret tracking from pose (turret only) ────────────────
        // Disabled while intake is at home/stowed to avoid unnecessary turret motion.
        boolean intakeAtHome = intake.isPivotAtHome();

        // Kept for dashboard compatibility.
        SmartDashboard.putBoolean("Turret/Alliance Zone Tracking Enabled", true);
        SmartDashboard.putBoolean("Turret/Always Pose Tracking Enabled", !intakeAtHome);
        SmartDashboard.putBoolean("Turret/Tracking Blocked By Intake Home", intakeAtHome);

        Command intakeRequiringCommand = CommandScheduler.getInstance().requiring(intake);
        boolean intakeHomeCommandRunning =
                intakeRequiringCommand != null && "IntakeHome".equals(intakeRequiringCommand.getName());

        // If FeedFromCenter is active, it must fully own turret aiming.
        // Skip always-on tower tracking to avoid conflicting turret setpoints.
        Command launcherRequiringCommand = CommandScheduler.getInstance().requiring(launcher);
        boolean feedFromCenterActive =
                launcherRequiringCommand != null
                && launcherRequiringCommand.getClass().getSimpleName().equals("FeedFromCenterCommand");

        if (feedFromCenterActive) {
            SmartDashboard.putString("Turret/Tracking State", "Suppressed (FeedFromCenter Active)");
        } else if (intakeHomeCommandRunning) {
            // IntakeHome command is actively sequencing turret-first then intake-home.
            // Suppress ALL always-on turret writes here so IntakeHome owns the setpoint flow.
            SmartDashboard.putString("Turret/Tracking State", "IntakeHome Sequence Active");
        } else if (intakeAtHome) {
            // Intake is home/stowed: command turret to mechanical zero instead of tracking.
            double zeroRaw = 0.0;
            double currentRaw = launcher.getTurretPosition();

            // Small deadband avoids spamming identical Motion Magic setpoints every loop.
            if (Math.abs(zeroRaw - currentRaw) > 0.005) {
                launcher.setTurretPosition(zeroRaw);
                SmartDashboard.putString("Turret/Tracking State", "Homing to Zero (Intake Home)");
            } else {
                SmartDashboard.putString("Turret/Tracking State", "At Zero (Intake Home)");
            }
        } else {
            // Track based on robot pose when intake is not home.
            // If alliance is unknown, retain existing fallback behavior (red primary tag).
            var alliance = DriverStation.getAlliance();
            boolean allianceKnown = alliance.isPresent();
            int allianceTagId = allianceKnown && alliance.get() == Alliance.Blue
                    ? Constants.ShootingArc.kBluePrimaryTagId
                    : Constants.ShootingArc.kRedPrimaryTagId;

            var allianceTower = ShootingArcManager.getTowerCenter(robotPose, allianceTagId);
            double targetTrackAngleDeg = ShootingArcManager.calculateTurretAngle(robotPose, allianceTower);
            double desiredRaw = targetTrackAngleDeg * Constants.TurretConstants.kRotationsPerDegree;
            desiredRaw = Math.max(-Constants.TurretConstants.kPhysicalLimitRotations,
                        Math.min( Constants.TurretConstants.kPhysicalLimitRotations, desiredRaw));

            double currentRaw = launcher.getTurretPosition();

            // Deadband tuned to reduce command chatter/hunting while preserving responsiveness.
            // 0.02 raw rotations ≈ 0.19 turret degrees.
            if (Math.abs(desiredRaw - currentRaw) > 0.02) {
                launcher.setTurretPosition(desiredRaw);
            }

            SmartDashboard.putString("Turret/Tracking State", "Active");
        }

        // Tag source diagnostics for dropout troubleshooting near walls.
        double heldAge = (lastSeenTowerTagTimestampSec > 0.0)
                ? Math.max(0.0, Timer.getFPGATimestamp() - lastSeenTowerTagTimestampSec)
                : -1.0;
        SmartDashboard.putString("Tower/Tag Source", towerTagSource);
        SmartDashboard.putNumber("Tower/Held Tag Age (s)", round(heldAge, 3));
        SmartDashboard.putNumber("Tower/Hold Timeout (s)", kTowerTagHoldSeconds);
    }

    /**
     * Publishes FMS/DriverStation info to SmartDashboard/Elastic.
     *
     * <p>Published under the {@code FMS/} namespace:
     * <ul>
     *   <li>{@code FMS/Alliance}</li>
     *   <li>{@code FMS/Alliance Is Red}</li>
     *   <li>{@code FMS/Alliance Is Blue}</li>
     *   <li>{@code FMS/Match Time (s)}</li>
     *   <li>{@code FMS/Shift Time (s)}</li>
     * </ul>
     *
     * <p>Shift Time is defined as elapsed teleop time:
     * {@code 135.0 - matchTime} while in teleop, otherwise {@code 0.0}.
     * Throttled to every 5 calls (~100 ms) to reduce NetworkTables load.
     */
    public void updateFmsInfo() {
        fmsInfoCounter++;
        if (fmsInfoCounter < 5) return;
        fmsInfoCounter = 0;

        var alliance = DriverStation.getAlliance();
        String allianceStr = alliance.map(a -> a == Alliance.Red ? "Red" : "Blue").orElse("Unknown");
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
        boolean isBlue = alliance.isPresent() && alliance.get() == Alliance.Blue;

        double matchTime = DriverStation.getMatchTime();

        double shiftTime = 0.0;
        if (DriverStation.isTeleopEnabled() && matchTime >= 0.0) {
            shiftTime = Math.max(0.0, 135.0 - matchTime);
        }

        String gameMessage = DriverStation.getGameSpecificMessage();
        char activeHubChar = (gameMessage != null && !gameMessage.isEmpty())
                ? Character.toUpperCase(gameMessage.charAt(0))
                : '\0';

        // Official 2026 game data chars are R/B only.
        boolean activeHubRed = activeHubChar == 'R';
        boolean activeHubBlue = activeHubChar == 'B';
        String activeHub = activeHubRed ? "Red" : (activeHubBlue ? "Blue" : "Unknown");

        String displayActiveHub = activeHub;
        String currentShift = "N/A";
        boolean shiftEndingSoon = false;

        // We can only compute shift ownership if we have valid R/B game data.
        boolean hasValidGameData = activeHubRed || activeHubBlue;
        boolean redInactiveFirst = activeHubRed; // R means red goes inactive first.

        if (DriverStation.isAutonomousEnabled()) {
            currentShift = "Auto";
            displayActiveHub = "Both";
        } else if (DriverStation.isTeleopEnabled()) {
            // 2026 timing windows from WPILib example:
            // >130 transition, >105 shift1, >80 shift2, >55 shift3, >30 shift4, else endgame.
            if (matchTime > 130.0) {
                currentShift = "Transition";
                displayActiveHub = "Both";
                // 5-second pre-shift warning before Shift 1 starts at 130s.
                shiftEndingSoon = matchTime <= 135.0;
            } else if (matchTime > 105.0) {
                currentShift = "Shift 1";
                displayActiveHub = hasValidGameData ? (redInactiveFirst ? "Blue" : "Red") : "Unknown";
                shiftEndingSoon = hasValidGameData && matchTime <= 110.0;
            } else if (matchTime > 80.0) {
                currentShift = "Shift 2";
                displayActiveHub = hasValidGameData ? (redInactiveFirst ? "Red" : "Blue") : "Unknown";
                shiftEndingSoon = hasValidGameData && matchTime <= 85.0;
            } else if (matchTime > 55.0) {
                currentShift = "Shift 3";
                displayActiveHub = hasValidGameData ? (redInactiveFirst ? "Blue" : "Red") : "Unknown";
                shiftEndingSoon = hasValidGameData && matchTime <= 60.0;
            } else if (matchTime > 30.0) {
                currentShift = "Shift 4";
                displayActiveHub = hasValidGameData ? (redInactiveFirst ? "Red" : "Blue") : "Unknown";
                shiftEndingSoon = hasValidGameData && matchTime <= 35.0;
            } else {
                currentShift = "Endgame";
                displayActiveHub = "Both";
            }
        } else {
            currentShift = "Disabled";
            displayActiveHub = "Unknown";
        }

        // Explicit fallback behavior:
        if (!hasValidGameData && DriverStation.isTeleopEnabled()) {
            displayActiveHub = "Unknown";
            shiftEndingSoon = false;
        }

        // Dashboard booleans should light for the specific hub OR Both.
        // (Derive strictly from the final display hub used by LEDs/Elastic.)
        boolean displayHubRed = "Red".equalsIgnoreCase(displayActiveHub) || "Both".equalsIgnoreCase(displayActiveHub);
        boolean displayHubBlue = "Blue".equalsIgnoreCase(displayActiveHub) || "Both".equalsIgnoreCase(displayActiveHub);

        // Single source-of-truth for LEDs:
        // true  => my alliance hub is active (or Both)
        // false => opponent hub is active or unknown
        boolean myHubActive = "Both".equalsIgnoreCase(displayActiveHub)
                || (isRed && "Red".equalsIgnoreCase(displayActiveHub))
                || (isBlue && "Blue".equalsIgnoreCase(displayActiveHub));

        SmartDashboard.putString("FMS/Alliance", allianceStr);
        SmartDashboard.putBoolean("FMS/Alliance Is Red", isRed);
        SmartDashboard.putBoolean("FMS/Alliance Is Blue", isBlue);
        SmartDashboard.putNumber("FMS/Match Time (s)", matchTime);
        SmartDashboard.putNumber("FMS/Shift Time (s)", round(shiftTime, 2));

        // Raw FMS game-data view
        SmartDashboard.putString("FMS/Active Hub Raw", activeHub);
        SmartDashboard.putBoolean("FMS/Active Hub Raw Is Red", activeHubRed);
        SmartDashboard.putBoolean("FMS/Active Hub Raw Is Blue", activeHubBlue);
        SmartDashboard.putString("FMS/Game Specific Message Raw", gameMessage == null ? "" : gameMessage);

        // Display value (use this single key for your one colored widget)
        SmartDashboard.putString("FMS/Current Shift", currentShift);
        SmartDashboard.putBoolean("FMS/Shift Ending Soon", shiftEndingSoon);
        SmartDashboard.putString("FMS/Active Hub", displayActiveHub);
        SmartDashboard.putBoolean("FMS/Active Hub Is Red", displayHubRed);
        SmartDashboard.putBoolean("FMS/Active Hub Is Blue", displayHubBlue);
        SmartDashboard.putBoolean("FMS/My Hub Active", myHubActive);
    }

    /**
     * Updates the drivetrain's pose estimator with vision measurements from both cameras.
     *
     * <p>Both the Back-Facing (BF) and Front-Facing (FF) cameras contribute pose estimates
     * to the drivetrain's Kalman filter. This is critical for {@link ShootFromPointCommand}:
     * the BF camera faces the tower and sees the tower AprilTags directly, giving the most
     * accurate distance and angle data for turret aiming. The FF camera faces forward and
     * provides pose corrections from other field tags.
     *
     * <p>Measurements are rejected if ANY target in the result reports a pose ambiguity
     * above {@link Constants.Vision#kMaxAmbiguity} (0.3). Multi-tag PnP results return
     * {@code -1} for ambiguity and are always accepted. This prevents high-ambiguity
     * single-tag detections from injecting bad poses into the Kalman filter and causing
     * the autonomous path planner to generate erratic paths.
     *
     * <p>Standard deviations are tighter (more trusted) for multi-tag estimates and looser
     * for single-tag estimates, matching the reliability of each measurement.
     *
     * <p>This should be called periodically (e.g., from Robot.robotPeriodic()).
     */
    public void updateVisionMeasurements() {
        // ── Keep PhotonPoseEstimator reference pose current ───────────────────
        // MULTI_TAG_PNP_ON_COPROCESSOR falls back to single-tag 3-D PnP when only
        // one tag is visible. The fallback strategy CLOSEST_TO_REFERENCE_POSE picks
        // the less-ambiguous of the two possible 3-D poses by comparing each candidate
        // against the current odometry estimate. Without this update the reference
        // stays wherever it was last set, causing large pose jumps during auto.
        visionSubsystem.setReferencePose(drivetrain.getState().Pose);

        // ── Back Facing camera: tower-facing pose estimator ──────────────────
        // The BF camera (Yaw ≈ 187°) faces the tower during ShootFromPointCommand.
        // Enabling this estimate allows all visible tower tags to correct the robot's
        // pose, giving accurate distance and turret angle calculations.
        var poseBF = visionSubsystem.getEstimatedGlobalPoseBF();
        String bfStatus;
        if (poseBF.isPresent()) {
            var estimatedPose = poseBF.get();

            // ── Ambiguity filter ──────────────────────────────────────────────
            // Reject the entire measurement if ANY target in the result has a valid
            // (non-negative) ambiguity score above the threshold. Multi-tag PnP
            // results return -1 and always pass this check.
            boolean tooAmbiguousBF = estimatedPose.targetsUsed.stream()
                .anyMatch(t -> {
                    double amb = t.getPoseAmbiguity();
                    return amb >= 0.0 && amb > Constants.Vision.kMaxAmbiguity;
                });

            boolean usesIgnoredTagBF = estimatedPose.targetsUsed.stream()
                .anyMatch(t -> {
                    int id = t.getFiducialId();
                    for (int ignoredId : Constants.Vision.kIgnoredPoseTagIds) {
                        if (id == ignoredId) return true;
                    }
                    return false;
                });

            if (usesIgnoredTagBF) {
                bfStatus = "Rejected (Ignored Tag)";
            } else if (tooAmbiguousBF) {
                bfStatus = "Rejected (High Ambiguity)";
            } else {
                Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();

                // ── Distance-priority std devs ────────────────────────────────
                // Average camera-to-tag distance for all tags used in this estimate.
                // getBestCameraToTarget().getTranslation().getNorm() = 3-D Euclidean
                // distance from the camera lens to the tag center (meters).
                double avgDistBF = estimatedPose.targetsUsed.stream()
                    .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                    .average()
                    .orElse(0.0);

                // Scale factor grows quadratically with distance so the camera seeing
                // closer tags gets tighter (smaller) std devs and higher Kalman weight.
                //   1 m → ×1.1  |  2 m → ×1.4  |  3 m → ×1.9  |  4 m → ×2.6
                double distScaleBF = 1.0 + (avgDistBF * avgDistBF
                                            * Constants.Vision.kDistanceScaleFactor);

                var baseStdDevsBF = estimatedPose.targetsUsed.size() >= 2
                    ? Constants.Vision.kMultiTagStdDevs
                    : Constants.Vision.kSingleTagStdDevs;

                // ── Hub-facing camera priority ────────────────────────────────
                // When the BF camera sees a tower tag it has the most direct view
                // of the hub. Apply kHubCameraStdDevBonus (0.5 = 50% tighter) to
                // give its pose estimate higher weight in the Kalman filter.
                boolean bfSeesTowerTag = false;
                var bfTagIds = visionSubsystem.getAllDetectedTagIdsBF();
                for (int tag : Constants.ShootingArc.kRedTowerTagIds) {
                    if (bfTagIds.contains(tag)) { bfSeesTowerTag = true; break; }
                }
                if (!bfSeesTowerTag) {
                    for (int tag : Constants.ShootingArc.kBlueTowerTagIds) {
                        if (bfTagIds.contains(tag)) { bfSeesTowerTag = true; break; }
                    }
                }
                double hubBonus = bfSeesTowerTag ? Constants.Vision.kHubCameraStdDevBonus : 1.0;

                var stdDevs = baseStdDevsBF.times(distScaleBF * hubBonus);

                // Feed BF pose estimate into the drivetrain Kalman filter
                drivetrain.addVisionMeasurement(
                    pose2d,
                    estimatedPose.timestampSeconds,
                    stdDevs
                );
                bfStatus = String.format("Applied (%.1fm%s)", avgDistBF,
                    bfSeesTowerTag ? " HUB★" : "");
            }
        } else {
            bfStatus = "No Pose";
        }

        // ── Front Facing camera: forward-facing pose estimator ───────────────
        var poseFF = visionSubsystem.getEstimatedGlobalPoseFF();
        String ffStatus;
        if (poseFF.isPresent()) {
            var estimatedPose = poseFF.get();

            // ── Ambiguity filter ──────────────────────────────────────────────
            boolean tooAmbiguousFF = estimatedPose.targetsUsed.stream()
                .anyMatch(t -> {
                    double amb = t.getPoseAmbiguity();
                    return amb >= 0.0 && amb > Constants.Vision.kMaxAmbiguity;
                });

            boolean usesIgnoredTagFF = estimatedPose.targetsUsed.stream()
                .anyMatch(t -> {
                    int id = t.getFiducialId();
                    for (int ignoredId : Constants.Vision.kIgnoredPoseTagIds) {
                        if (id == ignoredId) return true;
                    }
                    return false;
                });

            if (usesIgnoredTagFF) {
                ffStatus = "Rejected (Ignored Tag)";
            } else if (tooAmbiguousFF) {
                ffStatus = "Rejected (High Ambiguity)";
            } else {
                Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();

                // ── Distance-priority std devs ────────────────────────────────
                double avgDistFF = estimatedPose.targetsUsed.stream()
                    .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                    .average()
                    .orElse(0.0);

                double distScaleFF = 1.0 + (avgDistFF * avgDistFF
                                            * Constants.Vision.kDistanceScaleFactor);

                var baseStdDevsFF = estimatedPose.targetsUsed.size() >= 2
                    ? Constants.Vision.kMultiTagStdDevs
                    : Constants.Vision.kSingleTagStdDevs;

                var stdDevs = baseStdDevsFF.times(distScaleFF);

                // Feed FF pose estimate into the drivetrain Kalman filter
                drivetrain.addVisionMeasurement(
                    pose2d,
                    estimatedPose.timestampSeconds,
                    stdDevs
                );
                ffStatus = String.format("Applied (%.1fm)", avgDistFF);
            }
        } else {
            ffStatus = "No Pose";
        }

        // ── Left camera: side-facing pose estimator ──────────────────────────
        var poseLeft = visionSubsystem.getEstimatedGlobalPoseLeft();
        String leftStatus;
        if (poseLeft.isPresent()) {
            var estimatedPose = poseLeft.get();

            boolean tooAmbiguousLeft = estimatedPose.targetsUsed.stream()
                .anyMatch(t -> {
                    double amb = t.getPoseAmbiguity();
                    return amb >= 0.0 && amb > Constants.Vision.kMaxAmbiguity;
                });

            boolean usesIgnoredTagLeft = estimatedPose.targetsUsed.stream()
                .anyMatch(t -> {
                    int id = t.getFiducialId();
                    for (int ignoredId : Constants.Vision.kIgnoredPoseTagIds) {
                        if (id == ignoredId) return true;
                    }
                    return false;
                });

            if (usesIgnoredTagLeft) {
                leftStatus = "Rejected (Ignored Tag)";
            } else if (tooAmbiguousLeft) {
                leftStatus = "Rejected (High Ambiguity)";
            } else {
                Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();

                double avgDistLeft = estimatedPose.targetsUsed.stream()
                    .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                    .average()
                    .orElse(0.0);

                double distScaleLeft = 1.0 + (avgDistLeft * avgDistLeft
                                            * Constants.Vision.kDistanceScaleFactor);

                var baseStdDevsLeft = estimatedPose.targetsUsed.size() >= 2
                    ? Constants.Vision.kMultiTagStdDevs
                    : Constants.Vision.kSingleTagStdDevs;

                var stdDevs = baseStdDevsLeft.times(distScaleLeft);

                drivetrain.addVisionMeasurement(
                    pose2d,
                    estimatedPose.timestampSeconds,
                    stdDevs
                );
                leftStatus = String.format("Applied (%.1fm)", avgDistLeft);
            }
        } else {
            leftStatus = "No Pose";
        }

        // ── Right camera: side-facing pose estimator ─────────────────────────
        var poseRight = visionSubsystem.getEstimatedGlobalPoseRight();
        String rightStatus;
        if (poseRight.isPresent()) {
            var estimatedPose = poseRight.get();

            boolean tooAmbiguousRight = estimatedPose.targetsUsed.stream()
                .anyMatch(t -> {
                    double amb = t.getPoseAmbiguity();
                    return amb >= 0.0 && amb > Constants.Vision.kMaxAmbiguity;
                });

            boolean usesIgnoredTagRight = estimatedPose.targetsUsed.stream()
                .anyMatch(t -> {
                    int id = t.getFiducialId();
                    for (int ignoredId : Constants.Vision.kIgnoredPoseTagIds) {
                        if (id == ignoredId) return true;
                    }
                    return false;
                });

            if (usesIgnoredTagRight) {
                rightStatus = "Rejected (Ignored Tag)";
            } else if (tooAmbiguousRight) {
                rightStatus = "Rejected (High Ambiguity)";
            } else {
                Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();

                double avgDistRight = estimatedPose.targetsUsed.stream()
                    .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                    .average()
                    .orElse(0.0);

                double distScaleRight = 1.0 + (avgDistRight * avgDistRight
                                            * Constants.Vision.kDistanceScaleFactor);

                var baseStdDevsRight = estimatedPose.targetsUsed.size() >= 2
                    ? Constants.Vision.kMultiTagStdDevs
                    : Constants.Vision.kSingleTagStdDevs;

                var stdDevs = baseStdDevsRight.times(distScaleRight);

                drivetrain.addVisionMeasurement(
                    pose2d,
                    estimatedPose.timestampSeconds,
                    stdDevs
                );
                rightStatus = String.format("Applied (%.1fm)", avgDistRight);
            }
        } else {
            rightStatus = "No Pose";
        }

        // Throttle status string puts to every 5 calls (~100 ms) — these are
        // informational only and do not need to update at 50 Hz.
        visionStatusCounter++;
        if (visionStatusCounter >= 5) {
            visionStatusCounter = 0;
            SmartDashboard.putString("Vision/BF/Measurement Status", bfStatus);
            SmartDashboard.putString("Vision/FF/Measurement Status", ffStatus);
            SmartDashboard.putString("Vision/Left/Measurement Status", leftStatus);
            SmartDashboard.putString("Vision/Right/Measurement Status", rightStatus);
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
    
    
    private double squareInput(double value) {
        return Math.copySign(value * value, value);
    }

    /**
     * Rounds a double to the specified number of decimal places for cleaner dashboard display.
     */
    private static double round(double value, int decimals) {
        double scale = Math.pow(10, decimals);
        return Math.round(value * scale) / scale;
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
            // ── Stop intake rollers for the duration of the arc shoot ─────────
            // Prevents game pieces from being fed into the launcher during shooting.
            // intake.stopRollersDirect() sets rollersEnabled=false so intakeDeployCollect()
            // will not restart the rollers until enableRollers() is called in finallyDo().
            Commands.runOnce(() -> intake.stopRollersDirect()),

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
                    var robotPose = drivetrain.getState().Pose;
                    var tower = ShootingArcManager.getTowerCenter(robotPose, tagId);
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
            // Re-enable intake rollers now that the flywheel has stopped.
            // If intakeDeployCollect() is still running, it will resume spinning
            // the rollers on its next loop. If it is not running, this is a no-op.
            intake.enableRollers();
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
        double now = Timer.getFPGATimestamp();

        // 1) Prefer a currently visible alliance-matching tower tag.
        int visibleTag = getVisibleTowerTag();
        if (visibleTag > 0) {
            lastSeenTowerTagId = visibleTag;
            lastSeenTowerTagTimestampSec = now;
            towerTagSource = "Visible";
            return visibleTag;
        }

        // 2) If vision briefly drops out, hold the most recently seen tag.
        if (lastSeenTowerTagId > 0 && lastSeenTowerTagTimestampSec > 0.0) {
            double age = now - lastSeenTowerTagTimestampSec;
            if (age <= kTowerTagHoldSeconds) {
                towerTagSource = "Held";
                return lastSeenTowerTagId;
            }
        }

        // 3) Fall back to alliance-based primary tag.
        var alliance = DriverStation.getAlliance();
        towerTagSource = "AllianceFallback";
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return Constants.ShootingArc.kBluePrimaryTagId;
        }
        return Constants.ShootingArc.kRedPrimaryTagId;
    }

    /**
     * Scans all four cameras for any currently-visible tower tag that belongs to
     * the alliance currently selected in the DriverStation.
     *
     * <p>Priority order: BF (hub-facing) → FF → Left → Right.
     * The BF camera is checked first because it directly faces the tower during
     * ShootFromPointCommand and therefore provides the most accurate tag data.
     * Left and Right cameras are checked last as they face sideways and will only
     * see tower tags at certain robot orientations.
     *
     * <p>Only tags for the active alliance are considered. If the alliance is not
     * yet reported (e.g. simulation / pre-FMS), both alliances are searched.
     *
     * @return The first alliance-matching tower tag ID found, or {@code -1} if none.
     */
    private int getVisibleTowerTag() {
        // Use ALL detected tag IDs from every camera — not just the dashboard-selected tag.
        var bfTags    = visionSubsystem.getAllDetectedTagIdsBF();
        var ffTags    = visionSubsystem.getAllDetectedTagIdsFF();
        var leftTags  = visionSubsystem.getAllDetectedTagIdsLeft();
        var rightTags = visionSubsystem.getAllDetectedTagIdsRight();

        var alliance = DriverStation.getAlliance();

        // If alliance is unknown, search both so the robot is never completely blind.
        boolean checkRed  = !alliance.isPresent() || alliance.get() == Alliance.Red;
        boolean checkBlue = !alliance.isPresent() || alliance.get() == Alliance.Blue;

        // Priority: BF (hub-facing) → FF → Left → Right
        if (checkRed) {
            for (int tag : Constants.ShootingArc.kRedTowerTagIds) { if (bfTags.contains(tag))    return tag; }
            for (int tag : Constants.ShootingArc.kRedTowerTagIds) { if (ffTags.contains(tag))    return tag; }
            for (int tag : Constants.ShootingArc.kRedTowerTagIds) { if (leftTags.contains(tag))  return tag; }
            for (int tag : Constants.ShootingArc.kRedTowerTagIds) { if (rightTags.contains(tag)) return tag; }
        }

        if (checkBlue) {
            for (int tag : Constants.ShootingArc.kBlueTowerTagIds) { if (bfTags.contains(tag))    return tag; }
            for (int tag : Constants.ShootingArc.kBlueTowerTagIds) { if (ffTags.contains(tag))    return tag; }
            for (int tag : Constants.ShootingArc.kBlueTowerTagIds) { if (leftTags.contains(tag))  return tag; }
            for (int tag : Constants.ShootingArc.kBlueTowerTagIds) { if (rightTags.contains(tag)) return tag; }
        }

        return -1; // No alliance-matching tower tag visible
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
