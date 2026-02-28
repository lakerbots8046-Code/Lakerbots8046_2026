package frc.robot.commands;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;

/**
 * Command to spin the turret for 1 second during autonomous.
 * This command starts the turret spinning and stops after 1 second.
 * Can be used in PathPlanner autonomous routines.
 */
public class AutoDeployIntake extends Command {
    private final Intake intakeSubsystem;
    //private final Timer timer;
    //private static final double SPIN_DURATION = 1.0; // seconds
    
    /**
     * Creates a new AimTurretAuto command.
     * 
     * @param intakeSubsystem The turret subsystem to use
     */
    public AutoDeployIntake(Intake intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        //this.timer = new Timer();
        addRequirements(intakeSubsystem);
    }
    
    @Override
    public void initialize() {
        intakeSubsystem.setRollersVelocity(Constants.IntakeConstants.kRollersIntakeVelocity);
    }
    
    @Override
    public void execute() {
        intakeSubsystem.goToPivotPosition(Constants.IntakeConstants.kPivotDeployCollectPosition);
        // Turret continues spinning until timer elapses
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }
    
    @Override
    public boolean isFinished() {
        return(intakeSubsystem.isPivotAtTarget());
    }
}
