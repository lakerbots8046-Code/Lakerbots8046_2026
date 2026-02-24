package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

/**
 * Static utility class for shoot-on-arc calculations.
 *
 * <p>The "shooting arc" is a circular arc of valid shooting positions around a tower.
 * At any point on this arc the robot is at a known field position, the turret can
 * aim at the tower, and the launcher can shoot the ball in.
 *
 * <p>All calculations use field-relative coordinates from the robot's pose estimator,
 * which fuses odometry with AprilTag vision measurements for accurate positioning
 * even while the robot is moving.
 *
 * <h3>Tower tag layout (2026 Rebuilt field)</h3>
 * <ul>
 *   <li>Red  tower: tags 9, 10, 11 — center ≈ (12.42, 4.02)</li>
 *   <li>Blue tower: tags 19, 20, 21 — center ≈ (5.01, 4.02)</li>
 * </ul>
 */
public class ShootingArcManager {

    // ── Tower tag ID arrays ───────────────────────────────────────────────────
    public static final int[] RED_TOWER_TAGS  = {9, 10, 11};
    public static final int[] BLUE_TOWER_TAGS = {19, 20, 21};

    // ── Fallback tower center positions (used if tag not in field layout) ─────
    // Red tower: average of tags 9 (12.505, 3.666), 10 (12.505, 4.021), 11 (12.257, 4.625)
    private static final Translation2d RED_TOWER_CENTER_FALLBACK  = new Translation2d(12.422, 4.021);
    // Blue tower: average of tags 19 (5.215, 3.666), 20 (5.215, 4.021), 21 (4.612, 4.625)
    private static final Translation2d BLUE_TOWER_CENTER_FALLBACK = new Translation2d(5.014, 4.021);

    // Private constructor — static utility class only
    private ShootingArcManager() {}

    // =========================================================================
    // Tower position helpers
    // =========================================================================

    /**
     * Gets the tower center position from the field layout for a given tag ID.
     * Falls back to hardcoded positions if the tag is not found in the layout.
     *
     * @param tagId The AprilTag ID of any tower tag (9–11 for red, 19–21 for blue)
     * @return Translation2d of the tower tag's position on the field
     */
    public static Translation2d getTowerCenter(int tagId) {
        try {
            var tagPose = Constants.Vision.kTagLayout.getTagPose(tagId);
            if (tagPose.isPresent()) {
                return new Translation2d(tagPose.get().getX(), tagPose.get().getY());
            }
        } catch (Exception e) {
            System.err.println("ShootingArcManager: Could not get tag " + tagId
                    + " pose: " + e.getMessage());
        }
        // Fallback
        return isRedTowerTag(tagId) ? RED_TOWER_CENTER_FALLBACK : BLUE_TOWER_CENTER_FALLBACK;
    }

    /**
     * Returns {@code true} if the tag ID belongs to the red alliance tower.
     */
    public static boolean isRedTowerTag(int tagId) {
        for (int t : RED_TOWER_TAGS) {
            if (t == tagId) return true;
        }
        return false;
    }

    /**
     * Returns {@code true} if the tag ID belongs to the blue alliance tower.
     */
    public static boolean isBlueTowerTag(int tagId) {
        for (int t : BLUE_TOWER_TAGS) {
            if (t == tagId) return true;
        }
        return false;
    }

    // =========================================================================
    // Arc geometry
    // =========================================================================

    /**
     * Finds the nearest valid shooting pose on the arc around the tower.
     *
     * <p>The robot will be placed at {@code kPreferredShootingDistance} from the tower,
     * in the direction of the robot's current position from the tower.
     * The returned heading faces the tower.
     *
     * @param robotPose   Current robot pose on the field
     * @param towerTagId  The tower's AprilTag ID (any of 9–11 or 19–21)
     * @return Pose2d of the nearest shooting position (preferred distance, facing tower)
     */
    public static Pose2d getNearestShootingPose(Pose2d robotPose, int towerTagId) {
        Translation2d towerCenter = getTowerCenter(towerTagId);
        double preferredDist = Constants.ShootingArc.kPreferredShootingDistance;

        // Direction from tower to robot
        Translation2d toRobot = robotPose.getTranslation().minus(towerCenter);
        double currentDist = toRobot.getNorm();

        if (currentDist < 0.01) {
            // Robot is at tower center — use a safe default approach direction
            // Red tower: approach from the left (-X), Blue tower: from the right (+X)
            double defaultAngle = isRedTowerTag(towerTagId) ? Math.PI : 0.0;
            toRobot = new Translation2d(Math.cos(defaultAngle), Math.sin(defaultAngle));
            currentDist = 1.0;
        }

        // Normalize direction and scale to preferred distance
        Translation2d normalizedDir = toRobot.div(currentDist);
        Translation2d shootingPos   = towerCenter.plus(normalizedDir.times(preferredDist));

        // Robot heading: face the tower
        double angleToTower = Math.atan2(
                towerCenter.getY() - shootingPos.getY(),
                towerCenter.getX() - shootingPos.getX());

        return new Pose2d(shootingPos, new Rotation2d(angleToTower));
    }

    /**
     * Calculates the tangential velocity components for sliding along the arc.
     *
     * <p>Positive {@code lateralInput} moves the robot clockwise around the tower
     * (when viewed from above), which feels like "moving right" to the driver.
     *
     * @param robotPose    Current robot pose
     * @param towerPos     Tower center position
     * @param lateralInput Joystick input in the range [-1, 1]
     * @return Translation2d with field-relative X and Y velocity components (m/s)
     */
    public static Translation2d calculateArcVelocity(
            Pose2d robotPose, Translation2d towerPos, double lateralInput) {

        Translation2d toRobot    = robotPose.getTranslation().minus(towerPos);
        double        currentAngle = Math.atan2(toRobot.getY(), toRobot.getX());

        // Clockwise tangent at angle θ: (sin θ, −cos θ)
        double tangentX = Math.sin(currentAngle);
        double tangentY = -Math.cos(currentAngle);

        double speed = lateralInput * Constants.ShootingArc.kArcSlideSpeed;
        return new Translation2d(tangentX * speed, tangentY * speed);
    }

    /**
     * Calculates the target heading (radians) for the robot to face the tower.
     *
     * @param robotPose Current robot pose
     * @param towerPos  Tower center position
     * @return Target heading in radians (field-relative)
     */
    public static double calculateTargetHeading(Pose2d robotPose, Translation2d towerPos) {
        return Math.atan2(
                towerPos.getY() - robotPose.getY(),
                towerPos.getX() - robotPose.getX());
    }

    // =========================================================================
    // Turret aiming
    // =========================================================================

    /**
     * Calculates the turret angle (robot-relative, degrees) needed to aim at the tower.
     *
     * <p>Uses field-relative robot pose for accurate calculation while moving.
     * 0° = robot forward, positive = counterclockwise (left), negative = clockwise (right).
     *
     * @param robotPose Current robot pose on the field
     * @param towerPos  Tower center position on the field
     * @return Turret angle in degrees, normalized to [−180, 180]
     */
    public static double calculateTurretAngle(Pose2d robotPose, Translation2d towerPos) {
        // Angle from robot to tower in field coordinates (radians)
        double fieldAngle = Math.atan2(
                towerPos.getY() - robotPose.getY(),
                towerPos.getX() - robotPose.getX());

        // Subtract robot heading to get robot-relative turret angle
        double robotHeading   = robotPose.getRotation().getRadians();
        double turretAngleDeg = Math.toDegrees(fieldAngle - robotHeading);

        // Normalize to [−180, 180]
        while (turretAngleDeg >  180.0) turretAngleDeg -= 360.0;
        while (turretAngleDeg < -180.0) turretAngleDeg += 360.0;

        return turretAngleDeg;
    }

    // =========================================================================
    // Distance
    // =========================================================================

    /**
     * Calculates the Euclidean distance from the robot to the tower center.
     *
     * @param robotPose Current robot pose
     * @param towerPos  Tower center position
     * @return Distance in meters
     */
    public static double calculateDistance(Pose2d robotPose, Translation2d towerPos) {
        return robotPose.getTranslation().getDistance(towerPos);
    }

    /**
     * Returns {@code true} if the robot is within the valid shooting distance range.
     *
     * @param robotPose  Current robot pose
     * @param towerTagId Tower AprilTag ID
     */
    public static boolean isInShootingZone(Pose2d robotPose, int towerTagId) {
        double dist = calculateDistance(robotPose, getTowerCenter(towerTagId));
        return dist >= Constants.ShootingArc.kMinShootingDistance
                && dist <= Constants.ShootingArc.kMaxShootingDistance;
    }

    // =========================================================================
    // Launcher / hood lookup
    // =========================================================================

    /**
     * Calculates the launcher flywheel velocity in RPS based on distance to tower.
     * Uses linear interpolation from {@link Constants.ShootingArc#kLauncherRPSLookup}.
     *
     * @param distanceMeters Distance from robot to tower in meters
     * @return Target launcher velocity in rotations per second (RPS)
     */
    public static double calculateLauncherRPS(double distanceMeters) {
        return interpolate(Constants.ShootingArc.kLauncherRPSLookup, distanceMeters);
    }

    /**
     * Calculates the hood angle in mechanism rotations based on distance to tower.
     * Uses linear interpolation from {@link Constants.ShootingArc#kHoodAngleLookup}.
     *
     * @param distanceMeters Distance from robot to tower in meters
     * @return Target hood position in mechanism rotations
     */
    public static double calculateHoodAngle(double distanceMeters) {
        return interpolate(Constants.ShootingArc.kHoodAngleLookup, distanceMeters);
    }

    // =========================================================================
    // Internal helpers
    // =========================================================================

    /**
     * Linear interpolation from a 2-column lookup table.
     *
     * <p>Table format: {@code {{x0, y0}, {x1, y1}, ...}} sorted ascending by x.
     * Clamps to the first/last y value when x is out of range.
     */
    private static double interpolate(double[][] table, double x) {
        if (table == null || table.length == 0) return 0.0;
        if (x <= table[0][0])                   return table[0][1];
        if (x >= table[table.length - 1][0])    return table[table.length - 1][1];

        for (int i = 0; i < table.length - 1; i++) {
            if (x >= table[i][0] && x <= table[i + 1][0]) {
                double t = (x - table[i][0]) / (table[i + 1][0] - table[i][0]);
                return table[i][1] + t * (table[i + 1][1] - table[i][1]);
            }
        }
        return table[table.length - 1][1];
    }
}
