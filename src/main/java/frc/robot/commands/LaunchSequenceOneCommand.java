package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Spindexer;

/**
 * LaunchSequenceOne command.
 *
 * <p>Runs motors CAN IDs 4, 5, and 6 at fixed independent RPS targets using
 * VelocityVoltage closed-loop control. Each motor has its own target and minimum
 * floor so they can be tuned independently in {@code SpindexerConstants}.
 *
 * <p>Motor mapping:
 * <ul>
 *   <li>CAN 4: Spindexer motor  — target {@link #SPINDEXER_RPS}, floor {@link #SPINDEXER_MIN_RPS}</li>
 *   <li>CAN 5: FlappyWheel (Stars) — target {@link #FLAPPYWHEEL_RPS}, floor {@link #FLAPPYWHEEL_MIN_RPS}</li>
 *   <li>CAN 6: Feeder motor     — target {@link #FEEDER_RPS}, floor {@link #FEEDER_MIN_RPS}</li>
 * </ul>
 */
public class LaunchSequenceOneCommand extends Command {

    // ── Spindexer subsystem ───────────────────────────────────────────────────
    private final Spindexer spindexer;

    // ── Flywheel RPS supplier ─────────────────────────────────────────────────
    /** Supplier for the flywheel RPS (rotations per second). */
    private final DoubleSupplier flywheelRPSSupplier;

    // ── Fixed independent RPS targets ────────────────────────────────────────
    // Each motor is tuned independently — adjust in Constants.SpindexerConstants
    // or directly here. Negative = intake direction for spindexer.
    // Positive = intake direction for flappy wheel and feeder.
    private static final double SPINDEXER_RPS    = -90.0; // CAN 4 — tune independently
    private static final double FLAPPYWHEEL_RPS  =  90.0; // CAN 5 — tune independently
    private static final double FEEDER_RPS       =  90.0; // CAN 6 — tune independently

    // ── Minimum RPS floors ────────────────────────────────────────────────────
    // Guarantees motors always run at at least this speed.
    // Negative motor (spindexer): floor is more-negative than target → no-op at full speed.
    // Positive motors: floor is less-positive than target → no-op at full speed.
    private static final double SPINDEXER_MIN_RPS   = -80.0; // guaranteed minimum (RPS)
    private static final double FLAPPYWHEEL_MIN_RPS =  80.0; // guaranteed minimum (RPS)
    private static final double FEEDER_MIN_RPS      =  80.0; // guaranteed minimum (RPS)

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
        // Fixed RPS targets — always run at full speed for fast shooting.
        // Apply minimum floor: for negative motor use Math.min (more-negative wins),
        // for positive motors use Math.max (more-positive wins).
        // Then clamp to the valid RPS range [-90, 90].
        double spindexerRPS   = Math.max(-90.0, Math.min(SPINDEXER_RPS,   SPINDEXER_MIN_RPS));
        double flappyWheelRPS = Math.min( 90.0, Math.max(FLAPPYWHEEL_RPS, FLAPPYWHEEL_MIN_RPS));
        double feederRPS      = Math.min( 90.0, Math.max(FEEDER_RPS,       FEEDER_MIN_RPS));

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
