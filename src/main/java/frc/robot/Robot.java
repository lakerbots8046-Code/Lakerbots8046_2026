// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    /**
     * Throttle counter for Field2d visualization updates.
     * Field2d only needs ~10 Hz — updating it at 50 Hz wastes CPU on
     * AprilTagFieldLayout map lookups and NetworkTables writes every 20 ms.
     * Updated every 5 calls ≈ 100 ms.
     */
    private int field2dCounter = 0;

    public Robot() {
        // Stop Phoenix 6 Signal Logger to prevent .hoot log files from filling
        // the roboRIO's limited disk space (~62 MB free). Safe for competition use —
        // disables binary log recording only; all robot functionality is unaffected.
        // To re-enable for SysId characterization, comment out this line.
        SignalLogger.stop();// Comment this back IN!!!!
        //SignalLogger.start();

        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        
        // Update vision measurements for pose estimation
        m_robotContainer.updateVisionMeasurements();
        
        // Update Field2D with current robot pose and vision targets (~10 Hz)
        // Field visualization does not need 50 Hz — throttle to every 5 loops.
        field2dCounter++;
        if (field2dCounter >= 5) {
            field2dCounter = 0;
            m_robotContainer.updateField2d();
        }

        // Publish tower tag identification + distance to Elastic (always-on)
        m_robotContainer.updateTowerTagInfo();
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
    public void teleopPeriodic() {}

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
