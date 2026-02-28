package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Spindexer;

/**
 * LaunchSequenceOne command.
 * 
 * <p>Spins motors CAN IDs 4, 5, and 6 at duty cycle values that are proportional
 * to the flywheel RPS (rotations per second). This command is designed to work
 * in parallel with the Launcher flywheel control.
 * 
 * <p>Motor mapping:
 * <ul>
 *   <li>CAN 4: Spindexer motor</li>
 *   <li>CAN 5: FlappyWheelFeeder motor</li>
 *   <li>CAN 6: Feeder motor</li>
 * </ul>
 * 
 * <p>The duty cycle values are calculated proportionally based on the flywheel RPS
 * using the same ratio pattern as the launchFromTower command:
 * <ul>
 *   <li>Spindexer (CAN 4): -0.6 at base RPS</li>
 *   <li>FlappyWheel (CAN 5): -0.1 at base RPS</li>
 *   <li>Feeder (CAN 6): 0.6 at base RPS</li>
 * </ul>
 * 
 * <p>Note: The flywheel uses negative RPS values, so the duty cycles will be
 * scaled accordingly.
 */
public class LaunchSequenceOneCommand extends Command {

    // ── Spindexer subsystem ───────────────────────────────────────────────────
    private final Spindexer spindexer;

    // ── Flywheel RPS supplier ─────────────────────────────────────────────────
    /** Supplier for the flywheel RPS (rotations per second). */
    private final DoubleSupplier flywheelRPSSupplier;

    // ── Base duty cycle values (from launchFromTower) ────────────────────────
    // These are the duty cycles at the base flywheel reference RPS
    private static final double SPINDEXER_DUTY_AT_BASE = -0.6;
    private static final double FLAPPYWHEEL_DUTY_AT_BASE = -0.1;
    private static final double FEEDER_DUTY_AT_BASE = 0.6;

    // Base flywheel reference RPS (the RPS at which the base duty cycles were tuned)
    // This is approximately the RPS at -0.75 duty cycle
    private static final double BASE_FLYWHEEL_RPS = -75.0;

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
        // Get the current flywheel RPS. Use the absolute value so the scale factor
        // is always positive regardless of whether the motor sensor reports the
        // shooting direction as positive or negative.
        double flywheelRPS = Math.abs(flywheelRPSSupplier.getAsDouble());

        // Calculate the scaling factor based on current flywheel speed magnitude.
        // At |BASE_FLYWHEEL_RPS| (75 RPS), scaleFactor = 1.0
        // At higher speed (e.g., 85 RPS), scaleFactor > 1.0
        // Using absolute values on both sides keeps scaleFactor always ≥ 0,
        // preserving the correct sign of each duty cycle constant below.
        double scaleFactor = 1.0;
        if (Math.abs(BASE_FLYWHEEL_RPS) > 0.01) {
            scaleFactor = flywheelRPS / Math.abs(BASE_FLYWHEEL_RPS);
        }
        
        // Calculate proportional duty cycles for each motor
        // These scale linearly with the flywheel RPS
        double spindexerDC = SPINDEXER_DUTY_AT_BASE * scaleFactor;
        double flappyWheelDC = FLAPPYWHEEL_DUTY_AT_BASE * scaleFactor;
        double feederDC = FEEDER_DUTY_AT_BASE * scaleFactor;

        // Apply duty cycles to motors using Spindexer subsystem
        spindexer.runFeedMotorsDirect(spindexerDC, flappyWheelDC, feederDC);
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
