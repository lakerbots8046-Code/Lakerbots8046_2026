package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeedFromCenter;
import frc.robot.subsystems.Spindexer;

/**
 * Runs the three feed motors (CAN 4/5/6) at fixed RPS targets from
 * {@link FeedFromCenter} while held.
 */
public class LaunchSequenceOneCommand extends Command {

    private final Spindexer spindexer;

    /** Creates a new launch-sequence command. */
    public LaunchSequenceOneCommand(Spindexer spindexer) {
        this.spindexer = spindexer;

        addRequirements(spindexer);
    }

    /** Backward-compatible overload; supplier is intentionally unused. */
    public LaunchSequenceOneCommand(Spindexer spindexer, @SuppressWarnings("unused") DoubleSupplier flywheelRPSSupplier) {
        this(spindexer);
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
