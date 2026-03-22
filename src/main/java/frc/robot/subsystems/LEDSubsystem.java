    package frc.robot.subsystems;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * LED subsystem — controls the CANdle (CAN ID 0) via Phoenix 6.
 *
 * <p>The CANdle has 8 onboard LEDs (indices 0-7); the external strip
 * starts at index 8. All animations target the external strip only.
 *
 * <p>State machine:
 * <ul>
 *   <li>Disabled: Static blue.</li>
 *   <li>Enabled, passive: Blue/White chase (default).</li>
 *   <li>Enabled, firing: Rainbow cycle.</li>
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
    private static final RGBWColor RED    = new RGBWColor(255, 0,   0);
    private static final RGBWColor GREEN  = new RGBWColor(0,   255, 0);
    private static final RGBWColor WHITE  = new RGBWColor(255, 255, 255);
    private static final RGBWColor BLACK  = new RGBWColor(0,   0,   0);

    // ── State ─────────────────────────────────────────────────────────────────
    private boolean isFiring = false;

    // ── Animation trackers ────────────────────────────────────────────────────
    private int    chaseOffset           = 0;
    private double lastAnimationUpdate   = 0.0;
    /** -2 = dashboard-forced OFF, -1 = robot disabled, 0 = default/passive, 1 = firing, 2 = active red/blue solid, 4 = shift warning flash */
    private int    currentAnimationState = -1;

    // ── Timing constants ──────────────────────────────────────────────────────
    // Strip density: 144 LEDs/m — sections are 10 LEDs each (period = 20).
    private static final double PASSIVE_CHASE_PERIOD_S       = 0.05;   // ~20 Hz (blue/white)
    private static final double FIRING_RAINBOW_PERIOD_S      = 0.020;  // ~50 Hz rainbow cycle updates

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

        // Dashboard/Elastic master enable:
        // false => force LEDs fully OFF regardless of robot state.
        boolean ledsEnabled = SmartDashboard.getBoolean("LED/Enabled", true);
        if (!ledsEnabled) {
            if (currentAnimationState != -2) {
                setAll(BLACK);
                currentAnimationState = -2;
            }
            return;
        }

        // Disabled: static blue once, then return
        if (!DriverStation.isEnabled()) {
            if (currentAnimationState != -1) {
                setAll(BLUE);
                currentAnimationState = -1;
            }
            return;
        }

        if (isFiring) {
            // Firing: rainbow cycle
            if (currentAnimationState != 1) {
                currentAnimationState = 1;
                chaseOffset = 0;
                lastAnimationUpdate = now;
            }

            if (now - lastAnimationUpdate > FIRING_RAINBOW_PERIOD_S) {
                chaseOffset = (chaseOffset + 1) % 180;
                lastAnimationUpdate = now;
                updateRainbowCycle(chaseOffset);
            }
            return;
        }

        // Enabled/not firing:
        // Use computed display hub from RobotContainer (supports practice mode timing logic).
        // Rules requested:
        // - MY hub active => GREEN
        // - MY hub not active => RED
        // - BOTH active => GREEN
        // - Alliance unknown => blue/white flashing
        String displayHub = SmartDashboard.getString("FMS/Active Hub", "Unknown");

        // Prefer dashboard alliance flags (published by RobotContainer.updateFmsInfo)
        // to avoid transient/stale DriverStation alliance reads during practice.
        boolean dashIsRed = SmartDashboard.getBoolean("FMS/Alliance Is Red", false);
        boolean dashIsBlue = SmartDashboard.getBoolean("FMS/Alliance Is Blue", false);

        DriverStation.Alliance effectiveAlliance = null;
        String allianceSource = "Unknown";
        if (dashIsRed && !dashIsBlue) {
            effectiveAlliance = DriverStation.Alliance.Red;
            allianceSource = "Dashboard";
        } else if (dashIsBlue && !dashIsRed) {
            effectiveAlliance = DriverStation.Alliance.Blue;
            allianceSource = "Dashboard";
        } else {
            // Fallback to DriverStation only if dashboard flags are not decisive.
            var allianceOpt = DriverStation.getAlliance();
            if (allianceOpt.isPresent()) {
                effectiveAlliance = allianceOpt.get();
                allianceSource = "DriverStation";
            }
        }

        boolean allianceKnown = effectiveAlliance != null;
        RGBWColor primaryColor = null;

        if (allianceKnown) {
            boolean myHubActive = SmartDashboard.getBoolean("FMS/My Hub Active", false);

            if ("Both".equalsIgnoreCase(displayHub)) {
                primaryColor = GREEN;
            } else if ("Red".equalsIgnoreCase(displayHub) || "Blue".equalsIgnoreCase(displayHub)) {
                primaryColor = myHubActive ? GREEN : RED;
            }
        }

        boolean shiftEndingSoon = SmartDashboard.getBoolean("FMS/Shift Ending Soon", false);

        // LED debug telemetry for Elastic troubleshooting
        SmartDashboard.putString("LED/Debug Display Hub", displayHub);
        SmartDashboard.putString("LED/Debug Alliance Source", allianceSource);
        SmartDashboard.putString("LED/Debug Alliance Effective",
                effectiveAlliance == null ? "Unknown" : effectiveAlliance.name());
        SmartDashboard.putBoolean("LED/Debug Shift Ending Soon", shiftEndingSoon);

        String primaryColorName = "Unknown";
        if (primaryColor == GREEN) primaryColorName = "Green";
        else if (primaryColor == RED) primaryColorName = "Red";
        else if (primaryColor == BLUE) primaryColorName = "Blue";
        SmartDashboard.putString("LED/Debug Primary Color", primaryColorName);

        if (primaryColor != null) {
            if (shiftEndingSoon) {
                // 5-second countdown bar to next shift:
                // current color shrinks while opposite color grows.
                if (currentAnimationState != 4) {
                    currentAnimationState = 4;
                }

                double matchTime = SmartDashboard.getNumber("FMS/Match Time (s)", -1.0);
                double progress = getShiftWarningProgress(matchTime); // 0.0 -> 1.0 over warning window

                RGBWColor oppositeColor = (primaryColor == GREEN) ? RED : GREEN;
                updateShiftCountdownBar(progress, primaryColor, oppositeColor);

                SmartDashboard.putNumber("LED/Debug Shift Warning Progress", progress);
            } else {
                // Solid active color when not in warning window.
                if (currentAnimationState != 2) {
                    currentAnimationState = 2;
                    setAll(primaryColor);
                }
            }
        } else {
            // Alliance unknown or display hub unknown: blue/white chase
            if (currentAnimationState != 0) {
                currentAnimationState = 0;
                chaseOffset = 0;
                lastAnimationUpdate = now;
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
     * @param offset  animation phase (incremented each frame)
     * @param period  pattern repeat length (e.g. 6 for blue/white, 4 for yellow/black)
     * @param onCount number of color-A LEDs per period
     * @param colorA  first color
     * @param colorB  second color
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
            int endIdx = STRIP_OFFSET + i + runLen - 1;
            candle.setControl(
                    new SolidColor(startIdx, endIdx)
                            .withColor(isColorA ? colorA : colorB));

            i += runLen;
        }
    }

    /**
     * Renders a shift warning countdown bar over the strip.
     *
     * <p>progress = 0.0 => all currentColor
     * <p>progress = 1.0 => all oppositeColor
     */
    private void updateShiftCountdownBar(double progress, RGBWColor currentColor, RGBWColor oppositeColor) {
        double clamped = Math.max(0.0, Math.min(1.0, progress));
        int currentCount = (int) Math.round(NUM_LEDS * (1.0 - clamped));
        currentCount = Math.max(0, Math.min(NUM_LEDS, currentCount));

        int oppositeCount = NUM_LEDS - currentCount;

        if (currentCount > 0) {
            int startIdx = STRIP_OFFSET;
            int endIdx = STRIP_OFFSET + currentCount - 1;
            candle.setControl(new SolidColor(startIdx, endIdx).withColor(currentColor));
        }

        if (oppositeCount > 0) {
            int startIdx = STRIP_OFFSET + currentCount;
            int endIdx = STRIP_OFFSET + NUM_LEDS - 1;
            candle.setControl(new SolidColor(startIdx, endIdx).withColor(oppositeColor));
        }
    }

    /**
     * Calculates 0..1 progress inside each 5-second warning window.
     * Windows: 135→130, 110→105, 85→80, 60→55, 35→30.
     */
    private double getShiftWarningProgress(double matchTime) {
        if (matchTime < 0.0) return 0.0;

        if (matchTime <= 135.0 && matchTime > 130.0) return (135.0 - matchTime) / 5.0;
        if (matchTime <= 110.0 && matchTime > 105.0) return (110.0 - matchTime) / 5.0;
        if (matchTime <= 85.0  && matchTime > 80.0)  return (85.0  - matchTime) / 5.0;
        if (matchTime <= 60.0  && matchTime > 55.0)  return (60.0  - matchTime) / 5.0;
        if (matchTime <= 35.0  && matchTime > 30.0)  return (35.0  - matchTime) / 5.0;

        return 0.0;
    }

    /**
     * Renders a rainbow across the strip and cycles it over time.
     *
     * @param baseHueDeg base hue offset [0, 179]
     */
    private void updateRainbowCycle(int baseHueDeg) {
        for (int i = 0; i < NUM_LEDS; i++) {
            int hue = (baseHueDeg + (i * 180 / NUM_LEDS)) % 180;
            RGBWColor color = hsvToRgbw(hue, 255, 160);
            int idx = STRIP_OFFSET + i;
            candle.setControl(new SolidColor(idx, idx).withColor(color));
        }
    }

    /**
     * Convert HSV (WPILib style: H=[0,179], S/V=[0,255]) to RGBWColor.
     */
    private RGBWColor hsvToRgbw(int h, int s, int v) {
        if (s <= 0) {
            return new RGBWColor(v, v, v);
        }

        int region = h / 30;
        int remainder = (h - (region * 30)) * 255 / 30;

        int p = (v * (255 - s)) / 255;
        int q = (v * (255 - ((s * remainder) / 255))) / 255;
        int t = (v * (255 - ((s * (255 - remainder)) / 255))) / 255;

        switch (region) {
            case 0:
                return new RGBWColor(v, t, p);
            case 1:
                return new RGBWColor(q, v, p);
            case 2:
                return new RGBWColor(p, v, t);
            case 3:
                return new RGBWColor(p, q, v);
            case 4:
                return new RGBWColor(t, p, v);
            default:
                return new RGBWColor(v, p, q);
        }
    }
}
