package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;

/**
 * Static utility class for shoot-on-arc calculations.
 *
 * <p>The "shooting arc" is a circular arc of valid shooting positions around a tower.
 * At any point on this arc the robot is at a known field position, the turret can
 * aim at the tower, and the launcher can shoot the ball in.
 *
 * <p>All calculations use field-relative coordinates from the robot's pose estimator,
 * which fuses odometry with ApriloTag vision measurements for accurate positioning
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
    // All tags physically mounted on or around each tower (from Constants.ShootingArc).
    // Expanded from {9,10,11} / {19,20,21} so any visible tower tag can be used
    // for pose estimation and alliance identification.
    public static final int[] RED_TOWER_TAGS  = Constants.ShootingArc.kRedTowerTagIds;   // {2,3,4,5,8,9,10,11}
    public static final int[] BLUE_TOWER_TAGS = Constants.ShootingArc.kBlueTowerTagIds;  // {18,19,20,21,24,25,26,27}

    // ── Primary tower tag IDs (center of tower opening) ──────────────────────
    // getTowerCenter() always returns the position of these tags so the turret
    // aims at the center of the goal regardless of which side tag is visible.
    private static final int RED_PRIMARY_TAG  = Constants.ShootingArc.kRedPrimaryTagId;  // 10
    private static final int BLUE_PRIMARY_TAG = Constants.ShootingArc.kBluePrimaryTagId; // 20

    // ── Goal center depth offset ──────────────────────────────────────────────
    //
    // The primary tags (10 for red, 20 for blue) are mounted on the FACE of the
    // tower. Both primary tags have identity rotation (W=1, X=Y=Z=0), meaning
    // their normal points in the +X direction. The goal center is this many
    // meters BEHIND the tag face in the −X direction.
    //
    // 2 ft = 0.6096 m — matches the fallback constants below.
    // TUNE: increase if shots miss outside edge; decrease if shots miss inside edge.
    private static final double GOAL_CENTER_DEPTH_METERS = 0.6096; // 2 ft behind tag face

    // ── Fallback tower center positions (used if primary tag not in field layout) ─
    // Red  tag 10 face: x=12.505156 → goal center x=12.505156−0.6096=11.895556
    // Blue tag 20 face: x=5.215153  → goal center x=5.215153−0.6096=4.605553
    // Z = 1.12395 m — tower tag height from 2026-rebuilt.json (tags 10 & 20)
    private static final Translation3d RED_TOWER_CENTER_FALLBACK  = new Translation3d(11.8956, 4.0214, 1.12395);
    private static final Translation3d BLUE_TOWER_CENTER_FALLBACK = new Translation3d(4.6056, 4.0214, 1.12395);

    // Private constructor — static utility class only
    private ShootingArcManager() {}

    // =========================================================================
    // Tower position helpers
    // =========================================================================

    /**
     * Gets the tower CENTER position (center of the goal opening) for a given tag ID.
     *
     * <p>The goal center is a <em>fixed field point</em> — it does not depend on the
     * robot's position. It is computed once from the primary tag's field position
     * (tag 10 for red, tag 20 for blue) by subtracting {@link #GOAL_CENTER_DEPTH_METERS}
     * in the −X direction (both primary tags have identity rotation, so their face
     * normal points in +X and the goal interior is in the −X direction).
     *
     * <p>Using a fixed goal center ensures the turret aims at the same point regardless
     * of where the robot is on the field, eliminating the "goal center drifts with robot
     * position" bug that caused outside-edge misses when approaching from an angle.
     *
     * @param robotPose Current robot pose (used only to determine alliance via tagId;
     *                  the returned position does NOT depend on robotPose)
     * @param tagId     Any tower tag ID — used only to determine red vs blue alliance
     * @return Translation3d of the fixed goal center position (X, Y, Z)
     */
    public static Translation3d getTowerCenter(Pose2d robotPose, int tagId) {
        int primaryTagId = isRedTowerTag(tagId) ? RED_PRIMARY_TAG : BLUE_PRIMARY_TAG;

        try {
            var tagPose = Constants.Vision.kTagLayout.getTagPose(primaryTagId);
            if (tagPose.isPresent()) {
                double tagX = tagPose.get().getX();
                double tagY = tagPose.get().getY();
                double tagZ = tagPose.get().getZ();
                // Both primary tags (10 & 20) have identity rotation → face +X.
                // Goal center is GOAL_CENTER_DEPTH_METERS behind the face in the −X direction.
                return new Translation3d(tagX - GOAL_CENTER_DEPTH_METERS, tagY, tagZ);
            }
        } catch (Exception e) {
            System.err.println("ShootingArcManager: Error getting primary tag "
                    + primaryTagId + ": " + e.getMessage());
        }

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
        Translation3d towerCenter   = getTowerCenter(robotPose, towerTagId);
        Translation2d towerCenter2d = towerCenter.toTranslation2d(); // 2D for path planning
        double preferredDist = Constants.ShootingArc.kPreferredShootingDistance;

        // Direction from tower to robot (2D — robot drives on a flat field)
        Translation2d toRobot = robotPose.getTranslation().minus(towerCenter2d);
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
        Translation2d shootingPos   = towerCenter2d.plus(normalizedDir.times(preferredDist));

        // Robot heading: face the tower
        double angleToTower = Math.atan2(
                towerCenter2d.getY() - shootingPos.getY(),
                towerCenter2d.getX() - shootingPos.getX());

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
            Pose2d robotPose, Translation3d towerPos, double lateralInput) {

        Translation2d toRobot    = robotPose.getTranslation().minus(towerPos.toTranslation2d());
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
    public static double calculateTargetHeading(Pose2d robotPose, Translation3d towerPos) {
        return Math.atan2(
                towerPos.getY() - robotPose.getY(),
                towerPos.getX() - robotPose.getX());
    }

    // =========================================================================
    // Turret pivot position
    // =========================================================================

    /**
     * Computes the turret pivot's actual field position by applying the
     * robot-relative offset ({@link Constants.TurretConstants#kTurretOffsetX},
     * {@link Constants.TurretConstants#kTurretOffsetY}) rotated by the robot's
     * current heading.
     *
     * <p>The turret pivot is 5 inches behind the robot center (kTurretOffsetX = −0.127 m,
     * kTurretOffsetY = 0). At side angles this offset shifts the turret laterally in
     * field coordinates, causing a real aiming error if the robot center is used instead.
     *
     * @param robotPose Current robot pose on the field
     * @return Field-relative position of the turret rotation axis
     */
    public static Translation3d getTurretFieldPosition(Pose2d robotPose) {
        double heading  = robotPose.getRotation().getRadians();
        double offsetX  = Constants.TurretConstants.kTurretOffsetX; // −0.127 m (5 in back)
        double offsetY  = Constants.TurretConstants.kTurretOffsetY; // 0.0 m (centered)

        // Rotate the robot-relative offset by the robot's heading to get field-relative offset
        double fieldX = robotPose.getX() + offsetX * Math.cos(heading) - offsetY * Math.sin(heading);
        double fieldY = robotPose.getY() + offsetX * Math.sin(heading) + offsetY * Math.cos(heading);
        double fieldZ = Constants.TurretConstants.kTurretOffsetZ; // 14 in = 0.3556 m above floor

        return new Translation3d(fieldX, fieldY, fieldZ);
    }

    // =========================================================================
    // Turret aiming
    // =========================================================================

    /**
     * Calculates the turret angle (robot-relative, degrees) needed to aim at the tower,
     * WITHOUT applying the {@code kTurretZeroOffsetDegrees} calibration constant.
     *
     * <p>Uses the turret pivot's actual field position (not the robot center) so that
     * the 5-inch rearward offset is correctly accounted for at all robot headings.
     * At side angles the offset shifts the turret laterally, which would cause a
     * measurable aiming error if the robot center were used instead.
     *
     * <p>0° = robot forward, positive = CCW (left), negative = CW (right).
     *
     * @param robotPose Current robot pose on the field
     * @param towerPos  Tower center position on the field
     * @return Turret angle in degrees, normalized to [−180, 180], no offset applied
     */
    public static double calculateTurretAngleRaw(Pose2d robotPose, Translation3d towerPos) {
        // Use the turret pivot's actual field position, not the robot center
        Translation3d turretPos = getTurretFieldPosition(robotPose);

        // Angle from turret pivot to tower in field coordinates (radians)
        double fieldAngle = Math.atan2(
                towerPos.getY() - turretPos.getY(),
                towerPos.getX() - turretPos.getX());

        // Subtract robot heading to get robot-relative turret angle
        double robotHeading   = robotPose.getRotation().getRadians();
        double turretAngleDeg = Math.toDegrees(fieldAngle - robotHeading);

        // Normalize to [−180, 180]
        while (turretAngleDeg >  180.0) turretAngleDeg -= 360.0;
        while (turretAngleDeg < -180.0) turretAngleDeg += 360.0;

        return turretAngleDeg;
    }

    /**
     * Calculates the turret angle (robot-relative, degrees) needed to aim at the tower,
     * applying the compiled {@link frc.robot.Constants.TurretConstants#kTurretZeroOffsetDegrees}
     * calibration constant.
     *
     * <p>Prefer {@link #calculateTurretAngleRaw} + a dashboard-adjustable offset when
     * tuning on the robot, so the offset can be changed without redeploying code.
     *
     * @param robotPose Current robot pose on the field
     * @param towerPos  Tower center position on the field
     * @return Turret angle in degrees, normalized to [−180, 180]
     */
    public static double calculateTurretAngle(Pose2d robotPose, Translation3d towerPos) {
        double turretAngleDeg = calculateTurretAngleRaw(robotPose, towerPos);

        // Apply physical zero-offset calibration.
        turretAngleDeg -= frc.robot.Constants.TurretConstants.kTurretZeroOffsetDegrees;

        // Normalize to [−180, 180]
        while (turretAngleDeg >  180.0) turretAngleDeg -= 360.0;
        while (turretAngleDeg < -180.0) turretAngleDeg += 360.0;

        return turretAngleDeg;
    }

    // =========================================================================
    // Distance
    // =========================================================================

    /**
     * Calculates the Euclidean distance from the turret pivot to the tower center.
     *
     * <p>Uses the turret pivot's actual field position (not the robot center) so that
     * the flywheel RPS and hood angle lookup tables receive the correct launch distance.
     *
     * @param robotPose Current robot pose
     * @param towerPos  Tower center position
     * @return Distance in meters from turret pivot to tower center
     */
    public static double calculateDistance(Pose2d robotPose, Translation3d towerPos) {
        // True 3D Euclidean distance: accounts for height difference between
        // turret pivot (kTurretOffsetZ = 0.3556 m) and tower center (1.12395 m).
        return getTurretFieldPosition(robotPose).getDistance(towerPos);
    }

    /**
     * Returns {@code true} if the robot is within the valid shooting distance range.
     *
     * @param robotPose  Current robot pose
     * @param towerTagId Tower AprilTag ID
     */
    public static boolean isInShootingZone(Pose2d robotPose, int towerTagId) {
        double dist = calculateDistance(robotPose, getTowerCenter(robotPose, towerTagId));
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
