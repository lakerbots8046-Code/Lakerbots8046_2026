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

    // ── Base duty cycle values (at BASE_FLYWHEEL_RPS) ────────────────────────
    // Increased from original values (-0.6 / -0.1 / 0.6) to raise BPS.
    // These are the duty cycles when the flywheel is running at BASE_FLYWHEEL_RPS.[]\
    
    // At lower flywheel speeds the values scale down proportionally, but are
    // always clamped to at least the minimum floor values below.
    private static final double SPINDEXER_DUTY_AT_BASE   = -1.0; // was -0.85 → max for higher BPS
    private static final double FLAPPYWHEEL_DUTY_AT_BASE = 0.75; // reduced — was -0.50 // -0.25
    private static final double FEEDER_DUTY_AT_BASE      =  1.0; // was 0.85 → max for higher BPS

    // ── Minimum duty cycle floors ─────────────────────────────────────────────
    // The flywheel encoder often reports ~0 RPS even when the motor is spinning
    // (known hardware issue — see ShootOnMoveCommand comments). Without a floor,
    // scaleFactor ≈ 0 and all feed motors stall, killing BPS.
    // These floors guarantee a minimum feed rate regardless of encoder reading.
    private static final double SPINDEXER_MIN_DUTY   = -0.85; // guaranteed minimum
    private static final double FLAPPYWHEEL_MIN_DUTY = 0.65; // guaranteed minimum (reduced) // -0.15
    private static final double FEEDER_MIN_DUTY      =  0.85; // guaranteed minimum

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
        
        // Calculate proportional duty cycles for each motor (scale with flywheel RPS)
        double spindexerDC   = SPINDEXER_DUTY_AT_BASE   * scaleFactor;
        double flappyWheelDC = FLAPPYWHEEL_DUTY_AT_BASE * scaleFactor;
        double feederDC      = FEEDER_DUTY_AT_BASE       * scaleFactor;

        // Apply minimum floor so motors always run at a guaranteed minimum speed
        // even when the flywheel encoder reports 0 RPS (known hardware issue).
        // Negative motors: take the more-negative of scaled vs floor (Math.min).
        // Positive motors: take the more-positive of scaled vs floor (Math.max).
        // Then clamp to the valid duty cycle range [-1.0, 1.0].
        spindexerDC   = Math.max(-1.0, Math.min(spindexerDC,   SPINDEXER_MIN_DUTY));
        flappyWheelDC = Math.max(-1.0, Math.min(flappyWheelDC, FLAPPYWHEEL_MIN_DUTY));
        feederDC      = Math.min( 1.0, Math.max(feederDC,       FEEDER_MIN_DUTY));

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
