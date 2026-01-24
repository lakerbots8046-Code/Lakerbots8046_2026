// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private XboxController controller;
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        
        // Controller (if needed separately from RobotContainer)
        controller = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
        
        // Initialize camera server for streaming - matches old project implementation
        // Create a network table
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient3("10.80.46.2"); // IP of the PhotonVision device (from PhotonVision tuner: 10.80.46.2:5810)
        CameraServer.addServer(Constants.Vision.kCameraStreamBL); // BL Camera
        CameraServer.addServer(Constants.Vision.kCameraStreamBR); // BR Camera
        NetworkTable visionTable = inst.getTable("photonvision");
        
        System.out.println("Camera streams configured:");
        System.out.println("  BL: " + Constants.Vision.kCameraStreamBL);
        System.out.println("  BR: " + Constants.Vision.kCameraStreamBR);
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        
        // Update vision measurements for pose estimation
        m_robotContainer.updateVisionMeasurements();
        
        // Update Field2D with current robot pose and vision targets
        m_robotContainer.updateField2d();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
        // Get vision data from subsystem
        var visionSubsystem = m_robotContainer.getVisionSubsystem();
        boolean targetVisibleBL = visionSubsystem.isTargetVisibleBL();
        boolean targetVisibleBR = visionSubsystem.isTargetVisibleBR();
        double targetYawBL = visionSubsystem.getTargetYawBL();
        double targetYawBR = visionSubsystem.getTargetYawBR();
        
        // Output all Xbox controller button states to SmartDashboard
        SmartDashboard.putBoolean("Controller A Button", controller.getAButton());
        SmartDashboard.putBoolean("Controller B Button", controller.getBButton());
        SmartDashboard.putBoolean("Controller X Button", controller.getXButton());
        SmartDashboard.putBoolean("Controller Y Button", controller.getYButton());
        SmartDashboard.putBoolean("Controller Left Bumper", controller.getLeftBumper());
        SmartDashboard.putBoolean("Controller Right Bumper", controller.getRightBumper());
        SmartDashboard.putBoolean("Controller Back Button", controller.getBackButton());
        SmartDashboard.putBoolean("Controller Start Button", controller.getStartButton());
        SmartDashboard.putBoolean("Controller Left Stick Button", controller.getLeftStickButton());
        SmartDashboard.putBoolean("Controller Right Stick Button", controller.getRightStickButton());
        
        // Output all Xbox controller axis values to SmartDashboard
        SmartDashboard.putNumber("Controller Left Stick X", controller.getLeftX());
        SmartDashboard.putNumber("Controller Left Stick Y", controller.getLeftY());
        SmartDashboard.putNumber("Controller Right Stick X", controller.getRightX());
        SmartDashboard.putNumber("Controller Right Stick Y", controller.getRightY());
        SmartDashboard.putNumber("Controller Left Trigger", controller.getLeftTriggerAxis());
        SmartDashboard.putNumber("Controller Right Trigger", controller.getRightTriggerAxis());
        
        // Output D-Pad (POV) state
        SmartDashboard.putNumber("Controller POV", controller.getPOV());

        // OPTIONAL: Auto-turn using BL camera when 'A' button is held
        if (controller.getAButton() && targetVisibleBL) {
            double turnCommand = -1.0 * targetYawBL * 0.02; // Simplified - adjust multiplier as needed
            SmartDashboard.putNumber("Auto Turn Command BL", turnCommand);
            // If you want to inject this into drivetrain, you'd need access
            // to drivetrain object, or push this to NetworkTables / a global state
        }
        
        // OPTIONAL: Auto-turn using BR camera when 'B' button is held
        if (controller.getBButton() && targetVisibleBR) {
            double turnCommand = -1.0 * targetYawBR * 0.02; // Simplified - adjust multiplier as needed
            SmartDashboard.putNumber("Auto Turn Command BR", turnCommand);
        }
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
