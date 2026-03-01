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
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Spindexer;
import frc.robot.commands.AutoDeployIntake;
import frc.robot.commands.CenterOnAprilTagCommand;
import frc.robot.commands.DriveToAprilTag;
import frc.robot.commands.DriveToAprilTagWithPathPlanner;
import frc.robot.commands.FeedFromCenterCommand;
import frc.robot.commands.ShootFromPointCommand;
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

    /** Throttle counter for vision status string puts (updated every 5 calls ≈ 100 ms). */
    private int visionStatusCounter = 0;

    /** Throttle counter for tower tag info puts (updated every 5 calls ≈ 100 ms). */
    private int towerTagCounter = 0;

    public RobotContainer() {
        registerNamedCommands();
        configureBindings();

        // Hood default command: retract to position 0 (22° / stowed) whenever no
        // shooting command is active. WPILib automatically resumes this command
        // whenever ShootOnMoveCommand or ShootFromPointCommand ends.
        launcher.setDefaultCommand(launcher.retractHood());
        
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

        NamedCommands.registerCommand("AutoIntakeDeployCollect",
            intake.AutoIntakeDeployCollect());

        NamedCommands.registerCommand("DumpAndReturn", 
            intake.dumpAndReturn());

    
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
            "centerDepotClimb",
            "leftNeutralFeed",
            "leftNeutralScore",
            "rightNeutralFeed",
            "rightNeutralScore",
            "rightOutpostScore"
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
        joystick.start().onTrue(drivetrain.runOnce(() -> {
            drivetrain.seedFieldCentric();
            Pose2d currentPose = drivetrain.getState().Pose;
            visionSubsystem.setReferencePose(currentPose);
            SmartDashboard.putString("Field-Centric Status", "Reset at: " + System.currentTimeMillis());
        }));

        // Left bumper: Toggle intake deploy/collect.
        // First press: deploys pivot to -1.2 rot (Motion Magic) and starts rollers.
        // Second press: stops rollers and retracts pivot to home (-0.1 rot).
        joystick.rightBumper().toggleOnTrue(intake.intakeDeployCollect());

        // Left bumper: Dump-and-return sequence.
        // Lifts the intake pivot to the dump position (-0.75 rot), holds for 0.5 s,
        // then returns to the deploy/collect position (-1.35 rot).
        // Requires the intake subsystem — will interrupt intakeDeployCollect() while running.
        // Re-press right bumper after this command finishes to resume normal intake.
        joystick.leftBumper().onTrue(intake.dumpAndReturn());

        // X button: Toggle intake rollers on/off independently of pivot.
        // First press stops rollers; second press starts them again.
        // Uses no-subsystem command so it does NOT interrupt the pivot control.
        joystick.x().onTrue(intake.toggleRollers());

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
        joystick.leftTrigger(0.3).whileTrue(
            new ShootFromPointCommand(
                drivetrain,
                launcher,
                spindexer,
                this::getActiveTowerTagId,
                () -> {
                    int pov = joystick.getHID().getPOV();
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


        // Right trigger (hard press, 0.75+): Feed from center.
        // Aims turret at the midpoint between the closer feed-station tag pair
        // (offset 2 ft behind the midpoint), sets hood to FeedFromCenter.kHoodPosition,
        // spins flywheel, and fires via LaunchSequenceOneCommand once at speed.
        // Robot does NOT move — drivetrain is untouched.
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

        // POV Right (90°): nudge turret 1° CW — when NOT shooting
        joystick.pov(90).and(joystick.leftTrigger(0.3).negate())
            .onTrue(Commands.runOnce(() -> launcher.nudgeTurretDirect(-1.0)));

        // POV Up (0°): Bump intake pivot position up by kPivotBumpFactor
        joystick.pov(0).and(joystick.leftTrigger(0.3).negate())
            .onTrue(intake.bumpPivotUp());

        // POV Down (180°): Bump intake pivot position down by kPivotBumpFactor
        joystick.pov(180).and(joystick.leftTrigger(0.3).negate())
            .onTrue(intake.bumpPivotDown());

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
        if (towerTagCounter < 5) return;
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
        SmartDashboard.putNumber( "Tower/Distance (m)",  distance);
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
        SmartDashboard.putNumber("ShootFromPoint/Distance (m)",           distance);
        SmartDashboard.putString("ShootFromPoint/Tower Tag Info",         tagInfo);
        SmartDashboard.putNumber("ShootFromPoint/Turret Target (deg)",    targetAngle);
        SmartDashboard.putNumber("ShootFromPoint/Turret Position (deg)",  currentTurretDeg);
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

            if (tooAmbiguousBF) {
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

                var stdDevs = baseStdDevsBF.times(distScaleBF);

                // Feed BF pose estimate into the drivetrain Kalman filter
                drivetrain.addVisionMeasurement(
                    pose2d,
                    estimatedPose.timestampSeconds,
                    stdDevs
                );
                bfStatus = String.format("Applied (%.1fm)", avgDistBF);
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

            if (tooAmbiguousFF) {
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

        // Throttle status string puts to every 5 calls (~100 ms) — these are
        // informational only and do not need to update at 50 Hz.
        visionStatusCounter++;
        if (visionStatusCounter >= 5) {
            visionStatusCounter = 0;
            SmartDashboard.putString("Vision/BF/Measurement Status", bfStatus);
            SmartDashboard.putString("Vision/FF/Measurement Status", ffStatus);
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
     * Scans both cameras for any currently-visible tower tag that belongs to
     * the alliance currently selected in the DriverStation.
     *
     * <p>Only tags for the active alliance are considered — a red-alliance robot
     * will never accidentally lock onto a blue tower tag even if one is visible.
     * If the alliance is not yet reported by the DriverStation (e.g. during
     * simulation or before FMS connection), both alliances are searched as a
     * safe fallback.
     *
     * <p>Back-Facing camera is checked first (typically has a better view of
     * the tower when the robot is approaching). Front-Facing camera is the fallback.
     *
     * @return The first alliance-matching tower tag ID found, or {@code -1} if none.
     */
    private int getVisibleTowerTag() {
        // Use ALL detected tag IDs from each camera — not just the dashboard-selected tag.
        // Previously, getDetectedTagIdBF/FF() only returned the tag matching the
        // SmartDashboard chooser (default: tag 14), so tower tags were never found
        // and getActiveTowerTagId() always fell back to the primary tag (10/20).
        var bfTags = visionSubsystem.getAllDetectedTagIdsBF();
        var ffTags = visionSubsystem.getAllDetectedTagIdsFF();

        var alliance = DriverStation.getAlliance();

        // Determine which tag arrays to search based on the DriverStation alliance.
        // If alliance is unknown, search both so the robot is never completely blind.
        boolean checkRed  = !alliance.isPresent() || alliance.get() == Alliance.Red;
        boolean checkBlue = !alliance.isPresent() || alliance.get() == Alliance.Blue;

        // BF camera first (faces tower during shooting), then FF camera — for each alliance set.
        // Return the first tower tag found so the pair-midpoint goal center is correct.
        if (checkRed) {
            for (int tag : Constants.ShootingArc.kRedTowerTagIds) {
                if (bfTags.contains(tag)) return tag;
            }
            for (int tag : Constants.ShootingArc.kRedTowerTagIds) {
                if (ffTags.contains(tag)) return tag;
            }
        }

        if (checkBlue) {
            for (int tag : Constants.ShootingArc.kBlueTowerTagIds) {
                if (bfTags.contains(tag)) return tag;
            }
            for (int tag : Constants.ShootingArc.kBlueTowerTagIds) {
                if (ffTags.contains(tag)) return tag;
            }
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
