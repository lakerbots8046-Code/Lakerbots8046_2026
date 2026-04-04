package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class ShootOnMoveMathTest {

    private static double estimateShooterMps(double launcherRps, double wheelCircumferenceMeters, double muzzleSpeedScale) {
        return Math.max(
                0.1,
                Math.abs(launcherRps) * Math.max(0.01, wheelCircumferenceMeters) * Math.max(0.05, muzzleSpeedScale));
    }

    private static double solveCompensatedDistance(
            double toTowerX,
            double toTowerY,
            double verticalDelta,
            double robotVxField,
            double robotVyField,
            double launcherRps,
            int iterations) {

        double leadGain = Constants.ShootingArc.kShootOnMoveLeadCompGain;
        double wheelCirc = Constants.ShootingArc.kShootOnMoveWheelCircumferenceMeters;
        double muzzleScale = Constants.ShootingArc.kShootOnMoveMuzzleSpeedScale;
        double extraLatencySec = Math.max(0.0, Constants.ShootingArc.kShootOnMoveExtraLatencySec);

        double distance = Math.hypot(Math.hypot(toTowerX, toTowerY), verticalDelta);
        int iters = Math.max(1, iterations);

        for (int i = 0; i < iters; i++) {
            double shooterMps = estimateShooterMps(launcherRps, wheelCirc, muzzleScale);
            double flightTimeSec = (distance / shooterMps) + extraLatencySec;

            // Corrected sign convention for current command implementation:
            // positive robot field velocity yields a positive lead vector, then command
            // applies unaryMinus() by default to aim opposite robot motion.
            double compX = toTowerX + robotVxField * flightTimeSec * leadGain;
            double compY = toTowerY + robotVyField * flightTimeSec * leadGain;

            distance = Math.hypot(Math.hypot(compX, compY), verticalDelta);
        }

        return distance;
    }

    @Test
    void lpfAlphaBoundsAreValid() {
        assertTrue(Constants.ShootingArc.kShootOnMoveVelocityLpfAlpha >= 0.0);
        assertTrue(Constants.ShootingArc.kShootOnMoveVelocityLpfAlpha <= 1.0);
    }

    @Test
    void leadIterationsIsAtLeastOne() {
        assertTrue(Constants.ShootingArc.kShootOnMoveLeadSolveIterations >= 1);
    }

    @Test
    void zeroVelocityKeepsDistanceSame() {
        double toX = 2.0;
        double toY = 1.0;
        double dz = 0.6;
        double direct = Math.hypot(Math.hypot(toX, toY), dz);

        double solved = solveCompensatedDistance(toX, toY, dz, 0.0, 0.0, -40.0, 3);
        assertEquals(direct, solved, 1e-9);
    }

    @Test
    void withRobotMotionCompensatedDistanceChanges() {
        double toX = 2.0;
        double toY = 0.0;
        double dz = 0.5;
        double direct = Math.hypot(Math.hypot(toX, toY), dz);

        double solved = solveCompensatedDistance(toX, toY, dz, 2.0, 0.0, -40.0, 3);
        assertTrue(Math.abs(solved - direct) > 1e-6);
    }

    @Test
    void forwardRobotMotionRequiresOppositeAimOffset() {
        double toX = 2.0;
        double toY = 0.0;
        double dz = 0.5;

        double leadGain = Constants.ShootingArc.kShootOnMoveLeadCompGain;
        double wheelCirc = Constants.ShootingArc.kShootOnMoveWheelCircumferenceMeters;
        double muzzleScale = Constants.ShootingArc.kShootOnMoveMuzzleSpeedScale;
        double extraLatencySec = Math.max(0.0, Constants.ShootingArc.kShootOnMoveExtraLatencySec);

        double launcherRps = -40.0;
        double shooterMps = estimateShooterMps(launcherRps, wheelCirc, muzzleScale);
        double directDistance = Math.hypot(Math.hypot(toX, toY), dz);
        double flightTimeSec = (directDistance / shooterMps) + extraLatencySec;

        // Positive field X robot velocity creates a positive candidate lead vector.
        // Command default then applies unaryMinus(), resulting in opposite-direction aim.
        double candidateLeadX = 2.0 * flightTimeSec * leadGain;
        assertTrue(candidateLeadX > 0.0);

        double appliedCompX = toX - candidateLeadX;
        assertTrue(appliedCompX < toX);
    }

    @Test
    void additionalIterationsRemainBoundedAndReasonable() {
        double toX = 2.2;
        double toY = -0.7;
        double dz = 0.5;

        double d1 = solveCompensatedDistance(toX, toY, dz, 1.5, -0.8, -42.0, 1);
        double d3 = solveCompensatedDistance(toX, toY, dz, 1.5, -0.8, -42.0, 3);
        double d5 = solveCompensatedDistance(toX, toY, dz, 1.5, -0.8, -42.0, 5);

        double direct = Math.hypot(Math.hypot(toX, toY), dz);

        assertTrue(d1 > 0.0 && d3 > 0.0 && d5 > 0.0);
        assertTrue(Math.abs(d5 - d3) < 0.25);
        assertTrue(Math.abs(d3 - direct) < 2.0);
    }
}
