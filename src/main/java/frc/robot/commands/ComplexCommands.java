package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Launcher;

public class ComplexCommands {

    public static Intake m_intake = RobotContainer.intake;
    public static Spindexer m_spindexer = RobotContainer.spindexer;
    public static Launcher m_launcher = RobotContainer.launcher;

    public static Command feedSequence() {
        return Commands.sequence(
            m_spindexer.setFeederVoltage(2),
            Commands.waitSeconds(0.5),
            m_spindexer.setFlappyWheelVoltage(2),
            Commands.waitSeconds(0.5),
            m_spindexer.setSpindexerVoltage(2));
    }

    public static Command intakeSequence() {
        return Commands.sequence(
            m_intake.setIntakeRollersVoltage(2),
            m_intake.setIntakePivotVoltage(1),
            Commands.waitSeconds(1),
            m_intake.setIntakeRollersVoltage(0),
            m_intake.setIntakePivotVoltage(-1),
            m_intake.setIntakePivotVoltage(0));
    }

     public static Command shootSequence() {
        return Commands.sequence(
            m_launcher.setLauncherVoltage(2));
    }

}
