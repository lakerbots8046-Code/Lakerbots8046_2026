package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeedFromCenter;
import frc.robot.subsystems.Spindexer;

/**
 * LaunchSequenceOne command.
 *
 * <p>Runs motors CAN IDs 4, 5, and 6 at fixed independent RPS targets using
 * VelocityVoltage closed-loop control. Targets are sourced from
 * {@link FeedFromCenter} constants for centralized tuning.
 *
 * <p>Motor mapping:
 * <ul>
 *   <li>CAN 4: Spindexer motor  — target {@link FeedFromCenter#kFeedSpindexerRPS}</li>
 *   <li>CAN 5: FlappyWheel (Stars) — target {@link FeedFromCenter#kFeedFlappyWheelRPS}</li>
 *   <li>CAN 6: Feeder motor     — target {@link FeedFromCenter#kFeedFeederRPS}</li>
 * </ul>
 */
public class LaunchSequenceOneCommand extends Command {

    // ── Spindexer subsystem ───────────────────────────────────────────────────
    private final Spindexer spindexer;

    // ── Flywheel RPS supplier ─────────────────────────────────────────────────
    /** Supplier for the flywheel RPS (rotations per second). */
    private final DoubleSupplier flywheelRPSSupplier;


    /**
     * Creates a new LaunchSequenceOneCommand.
     * 
     * @param spindexer             The Spindexer subsystem (controls motors CAN 4, 5, 6)
     * @param flywheelRPSSupplier   Supplier for the flywheel RPS value
     */
    public LaunchSequenceOneCommand(
            Spindexer spindexer,
            DoubleSupplier flywheelRPSSupplier) {
        
        this.spindexer = spindexer;
        this.flywheelRPSSupplier = flywheelRPSSupplier;

        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        // No initialization needed for this command
    }

    @Override
    public void execute() {
        // Feed velocities are centralized in FeedFromCenter constants.
        // Clamp to safe command range.
        double spindexerRPS = Math.max(-90.0, Math.min(90.0, FeedFromCenter.kFeedSpindexerRPS));
        double flappyWheelRPS = Math.max(-90.0, Math.min(90.0, FeedFromCenter.kFeedFlappyWheelRPS));
        double feederRPS = Math.max(-90.0, Math.min(90.0, FeedFromCenter.kFeedFeederRPS));

        // Send velocity targets to motors via VelocityVoltage closed-loop control.
        spindexer.runFeedMotorsDirect(spindexerRPS, flappyWheelRPS, feederRPS);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all three motors when command ends
        spindexer.stopFeedMotorsDirect();
    }

    @Override
    public boolean isFinished() {
        // Runs until interrupted (button released)
        return false;
    }
}
