package frc.robot.commands;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/**
 * Drives the climber to the Reach setpoint and finishes once the climber is at target.
 * Can be used in PathPlanner autonomous routines.
 */
public class ClimberGoToSetpoint extends Command {
    private final Climber climberSubsystem;
    private final Climber.Setpoint targetSetpoint;

    /**
     * Creates a new ClimberGoToSetpoint command.
     *
     * @param climberSubsystem Climber subsystem
     * @param targetSetpoint target climber setpoint
     */
    public ClimberGoToSetpoint(Climber climberSubsystem, Climber.Setpoint targetSetpoint) {
        this.climberSubsystem = climberSubsystem;
        this.targetSetpoint = targetSetpoint;
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        climberSubsystem.setSetpoint(targetSetpoint);
    }

    @Override
    public void end(boolean interrupted) {
        // No explicit stop needed; default command resumes after this command ends.
    }

    @Override
    public boolean isFinished() {
        return climberSubsystem.isAtSetpoint(targetSetpoint);
    }
}
