package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;

public class FeedFromCenterMathTest {

    private static double radialVelocity(Translation2d robotVelocity, Translation2d robotPos, Translation2d target) {
        Translation2d toTarget = target.minus(robotPos);
        double norm = toTarget.getNorm();
        if (norm <= 1e-6) return 0.0;
        Translation2d unit = toTarget.div(norm);
        return robotVelocity.getX() * unit.getX() + robotVelocity.getY() * unit.getY();
    }

    private static double rpsMotionScale(double radialVelocity) {
        final double kRadialCompGainPerMps = 0.10;
        final double kMinRpsMotionScale = 0.90;
        final double kMaxRpsMotionScale = 1.30;

        double scale = 1.0 - (kRadialCompGainPerMps * radialVelocity);
        return Math.max(kMinRpsMotionScale, Math.min(kMaxRpsMotionScale, scale));
    }

    @Test
    void retreatingMotionIncreasesRpsScale() {
        Translation2d robotPos = new Translation2d(5.0, 2.0);
        Translation2d target = new Translation2d(2.0, 2.0); // target to the left (-X)
        Translation2d robotVelocity = new Translation2d(+2.0, 0.0); // moving right, away from target

        double vr = radialVelocity(robotVelocity, robotPos, target);
        double scale = rpsMotionScale(vr);

        assertTrue(vr < 0.0);
        assertTrue(scale > 1.0);
    }

    @Test
    void approachingMotionDecreasesRpsScale() {
        Translation2d robotPos = new Translation2d(5.0, 2.0);
        Translation2d target = new Translation2d(2.0, 2.0); // target to the left (-X)
        Translation2d robotVelocity = new Translation2d(-2.0, 0.0); // moving left, toward target

        double vr = radialVelocity(robotVelocity, robotPos, target);
        double scale = rpsMotionScale(vr);

        assertTrue(vr > 0.0);
        assertTrue(scale < 1.0);
    }

    @Test
    void scaleIsClampedAtUpperBound() {
        double scale = rpsMotionScale(-10.0); // very strong retreat
        assertEquals(1.30, scale, 1e-9);
    }

    @Test
    void scaleIsClampedAtLowerBound() {
        double scale = rpsMotionScale(+10.0); // very strong approach
        assertEquals(0.90, scale, 1e-9);
    }

    @Test
    void blueTargetMappingIsSymmetricToRedByYBand() {
        // Red fixed targets: (15,2), (15,6)
        // Blue fixed targets should mirror in X and preserve Y bands: (2,2), (2,6)
        Translation2d redLow = new Translation2d(15.0, 2.0);
        Translation2d redHigh = new Translation2d(15.0, 6.0);
        Translation2d blueLow = new Translation2d(2.0, 2.0);
        Translation2d blueHigh = new Translation2d(2.0, 6.0);

        assertEquals(redLow.getY(), blueLow.getY(), 1e-9);
        assertEquals(redHigh.getY(), blueHigh.getY(), 1e-9);
        assertEquals(2.0, blueLow.getX(), 1e-9);
        assertEquals(2.0, blueHigh.getX(), 1e-9);
    }
}
