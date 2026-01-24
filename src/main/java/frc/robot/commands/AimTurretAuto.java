package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestMotorSubsystem;

/**
 * Command to spin the turret for 1 second during autonomous.
 * This command starts the turret spinning and stops after 1 second.
 * Can be used in PathPlanner autonomous routines.
 */
public class AimTurretAuto extends Command {
    private final TestMotorSubsystem turretSubsystem;
    private final Timer timer;
    private static final double SPIN_DURATION = 1.0; // seconds
    
    /**
     * Creates a new AimTurretAuto command.
     * 
     * @param turretSubsystem The turret subsystem to use
     */
    public AimTurretAuto(TestMotorSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;
        this.timer = new Timer();
        addRequirements(turretSubsystem);
    }
    
    @Override
    public void initialize() {
        System.out.println("AimTurretAuto: Starting - Spinning turret for " + SPIN_DURATION + " seconds");
        turretSubsystem.startSpin();
        timer.restart();
    }
    
    @Override
    public void execute() {
        // Turret continues spinning
        double elapsed = timer.get();
        System.out.println("AimTurretAuto: Spinning - " + String.format("%.2f", elapsed) + "s elapsed");
    }
    
    @Override
    public void end(boolean interrupted) {
        turretSubsystem.stopSpin();
        timer.stop();
        if (interrupted) {
            System.out.println("AimTurretAuto: Interrupted after " + String.format("%.2f", timer.get()) + " seconds");
        } else {
            System.out.println("AimTurretAuto: Completed - Spun for " + SPIN_DURATION + " seconds");
        }
    }
    
    @Override
    public boolean isFinished() {
        // Command finishes after 1 second
        return timer.hasElapsed(SPIN_DURATION);
    }
}
