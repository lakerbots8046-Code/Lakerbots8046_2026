package frc.robot.subsystems;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * LED subsystem — controls the CANdle (CAN ID 0) via Phoenix 6.
 *
 * <p>The CANdle has 8 onboard LEDs (indices 0-7); the external strip
 * starts at index 8. All animations target the external strip only.
 *
 * <p>State machine:
 * <ul>
 *   <li>Disabled: LEDs off.</li>
 *   <li>Enabled, passive: Blue/White chase (default).</li>
 *   <li>Enabled, firing: Fast Yellow/Black chase.</li>
 * </ul>
 */
public class LEDSubsystem extends SubsystemBase {

    // ── Hardware constants ────────────────────────────────────────────────────
    private static final int CANDLE_CAN_ID = 0;
    /** Number of LEDs on the external strip. */
    private static final int NUM_LEDS      = 94;
    /** External strip starts after the 8 onboard LEDs. */
    private static final int STRIP_OFFSET  = 8;

    private final CANdle candle;

    // ── Pre-defined colors ────────────────────────────────────────────────────
    private static final RGBWColor BLUE   = new RGBWColor(0,   0,   255);
    private static final RGBWColor WHITE  = new RGBWColor(255, 255, 255);
    private static final RGBWColor YELLOW = new RGBWColor(255, 180, 0);
    private static final RGBWColor BLACK  = new RGBWColor(0,   0,   0);

    // ── State ─────────────────────────────────────────────────────────────────
    private boolean isFiring = false;

    // ── Animation trackers ────────────────────────────────────────────────────
    private int    chaseOffset          = 0;
    private double lastAnimationUpdate  = 0.0;
    /** -1 = unset, 0 = passive, 1 = firing */
    private int    currentAnimationState = -1;

    // ── Timing constants ──────────────────────────────────────────────────────
    // Strip density: 144 LEDs/m — sections are 10 LEDs each (period = 20).
    private static final double PASSIVE_CHASE_PERIOD_S = 0.05;  // ~20 Hz  (blue/white)
    private static final double FIRING_CHASE_PERIOD_S  = 0.010; // ~100 Hz → effectively every loop at 50 Hz (fast yellow/black)

    public LEDSubsystem() {
        candle = new CANdle(CANDLE_CAN_ID);
        setAll(BLACK);
    }

    /**
     * Update the LED state from robot logic.
     *
     * @param firing true when a shooting command is actively firing
     */
    public void setRobotState(boolean firing) {
        this.isFiring = firing;
    }

    /**
     * Convenience overload — timeLeft parameter accepted but unused.
     * Kept for backwards compatibility with any existing callers.
     *
     * @param firing   true when a shooting command is actively firing
     * @param timeLeft unused
     */
    public void setRobotState(boolean firing, double timeLeft) {
        this.isFiring = firing;
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();

        // Disabled: turn LEDs off once, then return
        if (!DriverStation.isEnabled()) {
            if (currentAnimationState != -1) {
                setAll(BLACK);
                currentAnimationState = -1;
            }
            return;
        }

        if (isFiring) {
            // Firing: fast Yellow/Black chase (~40 Hz)
            if (currentAnimationState != 1) {
                currentAnimationState = 1;
                chaseOffset = 0;
            }

            if (now - lastAnimationUpdate > FIRING_CHASE_PERIOD_S) {
                chaseOffset = (chaseOffset + 1) % 20;
                lastAnimationUpdate = now;
                updateChase(chaseOffset, 20, 10, YELLOW, BLACK);
            }

        } else {
            // Passive: Blue/White chase (~20 Hz, default when enabled)
            if (currentAnimationState != 0) {
                currentAnimationState = 0;
                chaseOffset = 0;
            }

            if (now - lastAnimationUpdate > PASSIVE_CHASE_PERIOD_S) {
                chaseOffset = (chaseOffset + 1) % 20;
                lastAnimationUpdate = now;
                updateChase(chaseOffset, 20, 10, BLUE, WHITE);
            }
        }
    }

    // =========================================================================
    // Private helpers
    // =========================================================================

    /**
     * Sets the entire external strip to one solid color.
     */
    private void setAll(RGBWColor color) {
        candle.setControl(
            new SolidColor(STRIP_OFFSET, STRIP_OFFSET + NUM_LEDS - 1)
                .withColor(color));
    }

    /**
     * Renders a two-color chase pattern onto the external strip.
     * Consecutive LEDs of the same color are batched into a single
     * SolidColor request to minimise CAN bus traffic.
     *
     * @param offset   animation phase (incremented each frame)
     * @param period   pattern repeat length (e.g. 6 for blue/white, 4 for yellow/black)
     * @param onCount  number of color-A LEDs per period
     * @param colorA   first color
     * @param colorB   second color
     */
    private void updateChase(int offset, int period, int onCount,
                             RGBWColor colorA, RGBWColor colorB) {
        int i = 0;
        while (i < NUM_LEDS) {
            boolean isColorA = ((i + offset) % period) < onCount;

            // Find the length of this contiguous same-color run
            int runLen = 0;
            while (i + runLen < NUM_LEDS
                    && (((i + runLen + offset) % period) < onCount) == isColorA) {
                runLen++;
            }

            int startIdx = STRIP_OFFSET + i;
            int endIdx   = STRIP_OFFSET + i + runLen - 1;
            candle.setControl(
                new SolidColor(startIdx, endIdx)
                    .withColor(isColorA ? colorA : colorB));

            i += runLen;
        }
    }
}
