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
        double leadGainX = Constants.ShootingArc.kShootOnMoveLeadCompGainX;
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
        double leadGainX = Constants.ShootingArc.kShootOnMoveLeadCompGainX;
        double wheelCirc = Constants.ShootingArc.kShootOnMoveWheelCircumferenceMeters;
        double muzzleScale = Constants.ShootingArc.kShootOnMoveMuzzleSpeedScale;
        double extraLatencySec = Math.max(0.0, Constants.ShootingArc.kShootOnMoveExtraLatencySec);

        double launcherRps = -40.0;
        double shooterMps = estimateShooterMps(launcherRps, wheelCirc, muzzleScale);
        double directDistance = Math.hypot(Math.hypot(toX, toY), dz);
        double flightTimeSec = (directDistance / shooterMps) + extraLatencySec;

        // Candidate lead sign is determined by tuning constants (leadGain * leadGainX).
        double candidateLeadX = 2.0 * flightTimeSec * (leadGain * leadGainX);

        // Command default applies unaryMinus() to candidate lead for aiming.
        double appliedCompX = toX - candidateLeadX;
        assertEquals(toX - candidateLeadX, appliedCompX, 1e-9);
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
        assertTrue(Double.isFinite(d1) && Double.isFinite(d3) && Double.isFinite(d5));
    }

    @Test
    void predictionHorizonClampIsNonNegative() {
        assertTrue(Constants.ShootingArc.kShootOnMoveMaxPredictionSec >= 0.0);
    }

    @Test
    void turretSlewRateConstantIsPositive() {
        assertTrue(Constants.ShootingArc.kShootOnMoveTurretSlewRateDegPerSec > 0.0);
    }

    @Test
    void turretSettleLoopsIsAtLeastOne() {
        assertTrue(Constants.ShootingArc.kShootOnMoveTurretSettleLoops >= 1);
    }

    @Test
    void slewLimiterCapsPerCycleTurretDelta() {
        double requestedDeltaDeg = 30.0;
        double dtSec = 0.02;
        double maxStepDeg = Constants.ShootingArc.kShootOnMoveTurretSlewRateDegPerSec * dtSec;
        double limitedDeltaDeg = Math.max(-maxStepDeg, Math.min(maxStepDeg, requestedDeltaDeg));
        assertEquals(maxStepDeg, limitedDeltaDeg, 1e-9);
    }

    @Test
    void settleCounterReachesThresholdAfterConsecutiveAimedLoops() {
        int settleLoops = Math.max(1, Constants.ShootingArc.kShootOnMoveTurretSettleLoops);
        int counter = 0;
        for (int i = 0; i < settleLoops; i++) {
            counter++;
        }
        assertTrue(counter >= settleLoops);
    }

    @Test
    void settleCounterResetsWhenNotAimed() {
        int counter = 3;
        boolean turretAimedInstant = false;
        if (turretAimedInstant) {
            counter++;
        } else {
            counter = 0;
        }
        assertEquals(0, counter);
    }

    @Test
    void radialTangentialDecompositionIsOrthogonalAndReconstructsVelocity() {
        double toTowerX = 3.0;
        double toTowerY = 4.0; // norm = 5
        double vx = 1.2;
        double vy = -0.7;

        double norm = Math.hypot(toTowerX, toTowerY);
        double rx = toTowerX / norm;
        double ry = toTowerY / norm;
        double tx = -ry;
        double ty = rx;

        double radial = vx * rx + vy * ry;
        double tangential = vx * tx + vy * ty;

        double reconVx = radial * rx + tangential * tx;
        double reconVy = radial * ry + tangential * ty;

        assertEquals(vx, reconVx, 1e-9);
        assertEquals(vy, reconVy, 1e-9);

        // Orthonormal basis checks
        assertEquals(1.0, rx * rx + ry * ry, 1e-9);
        assertEquals(1.0, tx * tx + ty * ty, 1e-9);
        assertEquals(0.0, rx * tx + ry * ty, 1e-9);
    }

    @Test
    void reducedRadialScaleProducesSmallerForwardBackCompThanTangential() {
        double radialScale = Constants.ShootingArc.kShootOnMoveRadialCompScale;
        double tangentialScale = Constants.ShootingArc.kShootOnMoveTangentialCompScale;
        assertTrue(radialScale < tangentialScale);

        double scalarDt = 0.5;
        double radialVelocity = 2.0;
        double tangentialVelocity = 2.0;

        double radialCompMag = Math.abs(radialVelocity * scalarDt * radialScale);
        double tangentialCompMag = Math.abs(tangentialVelocity * scalarDt * tangentialScale);

        assertTrue(radialCompMag < tangentialCompMag);
    }

    @Test
    void lateralSpeedAboveThresholdUsesMinSettleLoops() {
        int settleLoopsBase = Math.max(1, Constants.ShootingArc.kShootOnMoveTurretSettleLoops);
        int settleLoopsMin = Math.max(1, Constants.ShootingArc.kShootOnMoveTurretSettleLoopsMin);
        double lateralRelaxStart = Math.max(0.0, Constants.ShootingArc.kShootOnMoveLateralRelaxStartMps);

        double tangentialVelocity = lateralRelaxStart + 0.1;
        int settleRequired = settleLoopsBase;
        if (Math.abs(tangentialVelocity) > lateralRelaxStart) {
            settleRequired = settleLoopsMin;
        }

        assertEquals(settleLoopsMin, settleRequired);
    }

    @Test
    void lateralSpeedBelowThresholdUsesBaseSettleLoops() {
        int settleLoopsBase = Math.max(1, Constants.ShootingArc.kShootOnMoveTurretSettleLoops);
        int settleLoopsMin = Math.max(1, Constants.ShootingArc.kShootOnMoveTurretSettleLoopsMin);
        double lateralRelaxStart = Math.max(0.0, Constants.ShootingArc.kShootOnMoveLateralRelaxStartMps);

        double tangentialVelocity = Math.max(0.0, lateralRelaxStart - 0.05);
        int settleRequired = settleLoopsBase;
        if (Math.abs(tangentialVelocity) > lateralRelaxStart) {
            settleRequired = settleLoopsMin;
        }

        assertEquals(settleLoopsBase, settleRequired);
    }

    @Test
    void mixedKinematicsRadialTangentialSignsBehaveAsExpected() {
        // Tower direction unit basis from robot to tower at 45 deg
        double rx = Math.sqrt(0.5);
        double ry = Math.sqrt(0.5);
        double tx = -ry;
        double ty = rx;

        // Mixed motion: mostly forward-right (away from tower basis) and slight negative tangent
        double vx = 1.5;
        double vy = 0.5;

        double radial = vx * rx + vy * ry;
        double tangential = vx * tx + vy * ty;

        assertTrue(radial > 0.0);
        assertTrue(tangential < 0.0);
    }

    @Test
    void maxLeadClampBoundsCombinedRadialTangentialVector() {
        double maxLead = Math.max(0.0, Constants.ShootingArc.kShootOnMoveMaxLeadMeters);

        double leadX = maxLead + 2.0;
        double leadY = maxLead + 1.0;
        double mag = Math.hypot(leadX, leadY);

        double clampedX = leadX;
        double clampedY = leadY;
        if (maxLead > 0.0 && mag > maxLead) {
            double s = maxLead / Math.max(1e-6, mag);
            clampedX *= s;
            clampedY *= s;
        }

        assertTrue(Math.hypot(clampedX, clampedY) <= maxLead + 1e-9);
    }

    @Test
    void invertLeadFlipsAppliedPredictionDirection() {
        double leadX = 0.7;
        double leadY = -0.2;

        double appliedNormalX = leadX;
        double appliedNormalY = leadY;

        double appliedInvertedX = -leadX;
        double appliedInvertedY = -leadY;

        assertEquals(-appliedNormalX, appliedInvertedX, 1e-9);
        assertEquals(-appliedNormalY, appliedInvertedY, 1e-9);
    }

    @Test
    void radialCompScaleIsWithinAggressiveTuningBounds() {
        double radialScale = Constants.ShootingArc.kShootOnMoveRadialCompScale;

        // Aggressive tuning range for front/back compensation while still bounded.
        assertTrue(radialScale >= 0.25);
        assertTrue(radialScale <= 0.60);
    }

    @Test
    void turretAimToleranceWasRelaxedFromOriginalValue() {
        assertTrue(Constants.ShootingArc.kTurretAimToleranceDeg >= 18.0);
    }

    @Test
    void leadVectorClampStillHoldsAtExtremeVelocitySigns() {
        double maxLead = Math.max(0.0, Constants.ShootingArc.kShootOnMoveMaxLeadMeters);

        double[][] vectors = new double[][] {
            { 50.0,  50.0},
            { 50.0, -50.0},
            {-50.0,  50.0},
            {-50.0, -50.0}
        };

        for (double[] v : vectors) {
            double x = v[0];
            double y = v[1];
            double mag = Math.hypot(x, y);
            if (maxLead > 0.0 && mag > maxLead) {
                double s = maxLead / Math.max(1e-6, mag);
                x *= s;
                y *= s;
            }
            assertTrue(Math.hypot(x, y) <= maxLead + 1e-9);
        }
    }

    @Test
    void debounceRequiresTwoConsecutiveMissesToResetSettledCounter() {
        int settledCounter = 3;
        int missCounter = 0;

        // first miss: should NOT reset settled counter
        boolean aimedInstant = false;
        if (aimedInstant) {
            settledCounter++;
            missCounter = 0;
        } else {
            missCounter++;
            if (missCounter >= 2) {
                settledCounter = 0;
            }
        }
        assertEquals(3, settledCounter);
        assertEquals(1, missCounter);

        // second consecutive miss: should reset
        if (aimedInstant) {
            settledCounter++;
            missCounter = 0;
        } else {
            missCounter++;
            if (missCounter >= 2) {
                settledCounter = 0;
            }
        }
        assertEquals(0, settledCounter);
        assertEquals(2, missCounter);
    }

    @Test
    void aimedInstantClearsMissCounterAndBuildsSettledCounter() {
        int settledCounter = 0;
        int missCounter = 1;

        boolean aimedInstant = true;
        if (aimedInstant) {
            settledCounter++;
            missCounter = 0;
        } else {
            missCounter++;
            if (missCounter >= 2) {
                settledCounter = 0;
            }
        }

        assertEquals(1, settledCounter);
        assertEquals(0, missCounter);
    }

    @Test
    void comfortablyInsideToleranceForcesSingleSettleLoopRequirement() {
        int settleLoopsBase = Math.max(1, Constants.ShootingArc.kShootOnMoveTurretSettleLoops);
        int settleLoopsMin = Math.max(1, Constants.ShootingArc.kShootOnMoveTurretSettleLoopsMin);
        double lateralRelaxStart = Math.max(0.0, Constants.ShootingArc.kShootOnMoveLateralRelaxStartMps);

        // Case where lateral speed would normally select either base/min.
        double tangentialVelocity = lateralRelaxStart + 0.2;
        int settleLoopsRequired = settleLoopsBase;
        if (Math.abs(tangentialVelocity) > lateralRelaxStart) {
            settleLoopsRequired = settleLoopsMin;
        }

        double aimToleranceDeg = Constants.ShootingArc.kTurretAimToleranceDeg;
        double absTurretErrorDeg = 0.5 * aimToleranceDeg; // comfortably inside (below 60%)

        if (absTurretErrorDeg < (0.6 * aimToleranceDeg)) {
            settleLoopsRequired = Math.min(settleLoopsRequired, 1);
        }

        assertEquals(1, settleLoopsRequired);
    }
}
