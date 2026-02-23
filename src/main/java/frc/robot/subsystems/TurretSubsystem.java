package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

//===================================== NOT OFFICIAL TURRET CODE!! TURRET CODE IS IN LAUNCHER.JAVA =====================================//

/**
 * Turret subsystem for aiming and positioning.
 * Provides dashboard controls to start/stop spinning the turret motor.
 * Also supports position control for autonomous aiming.
 */
public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turretMotor;
    private double targetSpeed = 0.0;
    private boolean isSpinning = false;
    
    // Position control
    private final PositionVoltage positionControl = new PositionVoltage(0);
    
    public TurretSubsystem() {
        // Initialize TalonFX turret motor using constants
        turretMotor = new TalonFX(TurretConstants.kTurretMotorID, TurretConstants.kCANBusName);
        
        // Configure motor to brake mode for safety
        turretMotor.setNeutralMode(NeutralModeValue.Brake);
        
        // Initialize dashboard controls
        initializeDashboard();
        
    }
    
    /**
     * Initialize SmartDashboard controls and displays
     */
    private void initializeDashboard() {
        String prefix = TurretConstants.kSmartDashboardPrefix;
        
        // Control buttons (these will appear in SmartDashboard/Shuffleboard)
        SmartDashboard.putBoolean(prefix + TurretConstants.kStartSpinKey, false);
        SmartDashboard.putBoolean(prefix + TurretConstants.kStopSpinKey, false);
        
        // Speed control
        SmartDashboard.putNumber(prefix + TurretConstants.kSpinSpeedKey, TurretConstants.kDefaultSpinSpeed);
        
        // Status displays
        SmartDashboard.putBoolean(prefix + TurretConstants.kIsSpinningKey, false);
        SmartDashboard.putNumber(prefix + TurretConstants.kCurrentSpeedKey, 0.0);
        SmartDashboard.putNumber(prefix + TurretConstants.kMotorCurrentKey, 0.0);
        SmartDashboard.putNumber(prefix + TurretConstants.kMotorTempKey, 0.0);
        SmartDashboard.putString(prefix + TurretConstants.kStatusKey, "Stopped");
    }
    
    // Loop counter for throttling SmartDashboard updates
    private int periodicCounter = 0;

    @Override
    public void periodic() {
        // Check dashboard buttons and apply motor speed every loop
        checkDashboardControls();
        turretMotor.set(targetSpeed);

        // Throttle SmartDashboard updates to every 5 loops (~100ms)
        periodicCounter++;
        if (periodicCounter < 5) return;
        periodicCounter = 0;
        updateDashboard();
    }
    
    /**
     * Check dashboard buttons and respond to user input
     */
    private void checkDashboardControls() {
        String prefix = TurretConstants.kSmartDashboardPrefix;
        
        // Check Start Spin button
        boolean startPressed = SmartDashboard.getBoolean(prefix + TurretConstants.kStartSpinKey, false);
        if (startPressed && !isSpinning) {
            startSpin();
            // Reset button
            SmartDashboard.putBoolean(prefix + TurretConstants.kStartSpinKey, false);
        }
        
        // Check Stop Spin button
        boolean stopPressed = SmartDashboard.getBoolean(prefix + TurretConstants.kStopSpinKey, false);
        if (stopPressed && isSpinning) {
            stopSpin();
            // Reset button
            SmartDashboard.putBoolean(prefix + TurretConstants.kStopSpinKey, false);
        }
        
        // Update speed from dashboard if motor is spinning
        if (isSpinning) {
            double dashboardSpeed = SmartDashboard.getNumber(prefix + TurretConstants.kSpinSpeedKey, TurretConstants.kDefaultSpinSpeed);
            // Clamp speed between min and max
            targetSpeed = Math.max(TurretConstants.kMinSpinSpeed, Math.min(TurretConstants.kMaxSpinSpeed, dashboardSpeed));
        }
    }
    
    /**
     * Update dashboard with current motor status
     */
    private void updateDashboard() {
        String prefix = TurretConstants.kSmartDashboardPrefix;
        
        SmartDashboard.putBoolean(prefix + TurretConstants.kIsSpinningKey, isSpinning);
        SmartDashboard.putNumber(prefix + TurretConstants.kCurrentSpeedKey, targetSpeed);
        
        // Get motor telemetry
        double current = turretMotor.getSupplyCurrent().getValueAsDouble();
        double temp = turretMotor.getDeviceTemp().getValueAsDouble();
        
        SmartDashboard.putNumber(prefix + TurretConstants.kMotorCurrentKey, current);
        SmartDashboard.putNumber(prefix + TurretConstants.kMotorTempKey, temp);
        
        // Update status string
        String status = isSpinning ? 
            String.format("Spinning at %.1f%%", targetSpeed * 100) : 
            "Stopped";
        SmartDashboard.putString(prefix + TurretConstants.kStatusKey, status);
    }
    
    /**
     * Start spinning the turret motor at the dashboard-specified speed
     */
    public void startSpin() {
        String prefix = TurretConstants.kSmartDashboardPrefix;
        double speed = SmartDashboard.getNumber(prefix + TurretConstants.kSpinSpeedKey, TurretConstants.kDefaultSpinSpeed);
        targetSpeed = Math.max(TurretConstants.kMinSpinSpeed, Math.min(TurretConstants.kMaxSpinSpeed, speed));
        isSpinning = true;
    }
    
    /**
     * Stop spinning the turret motor
     */
    public void stopSpin() {
        targetSpeed = 0.0;
        isSpinning = false;
    }
    
    /**
     * Set motor speed directly (for command-based control)
     * 
     * @param speed Motor speed from -1.0 to 1.0
     */
    public void setSpeed(double speed) {
        targetSpeed = Math.max(TurretConstants.kMinSpinSpeed, Math.min(TurretConstants.kMaxSpinSpeed, speed));
        isSpinning = (Math.abs(targetSpeed) > 0.01);
    }
    
    /**
     * Get current motor speed
     * 
     * @return Current speed from -1.0 to 1.0
     */
    public double getSpeed() {
        return targetSpeed;
    }
    
    /**
     * Check if motor is currently spinning
     * 
     * @return true if motor is spinning
     */
    public boolean isSpinning() {
        return isSpinning;
    }
    
    /**
     * Emergency stop - immediately stops the turret motor
     */
    public void emergencyStop() {
        targetSpeed = 0.0;
        isSpinning = false;
        turretMotor.set(0.0);
    }
    
    /**
     * Set turret to a specific angle using position control
     * 
     * @param degrees Target angle in degrees (0-360)
     */
    public void setAngle(double degrees) {
        double rotations = degrees * TurretConstants.kRotationsPerDegree;
        turretMotor.setControl(positionControl.withPosition(rotations));
        SmartDashboard.putNumber(TurretConstants.kSmartDashboardPrefix + TurretConstants.kTargetAngleKey, degrees);
    }
    
    /**
     * Get current turret angle in degrees
     * 
     * @return Current angle in degrees
     */
    public double getAngle() {
        double rotations = turretMotor.getPosition().getValueAsDouble();
        return rotations / TurretConstants.kRotationsPerDegree;
    }
    
    /**
     * Reset turret encoder to zero at current position
     */
    public void resetEncoder() {
        turretMotor.setPosition(0);
    }
    
    /**
     * Check if turret is at target angle (within tolerance)
     * 
     * @param targetDegrees Target angle in degrees
     * @param toleranceDegrees Tolerance in degrees
     * @return true if within tolerance
     */
    public boolean atAngle(double targetDegrees, double toleranceDegrees) {
        double currentAngle = getAngle();
        return Math.abs(currentAngle - targetDegrees) <= toleranceDegrees;
    }
    
    /**
     * Check if turret is at target angle with default tolerance
     * 
     * @param targetDegrees Target angle in degrees
     * @return true if within default tolerance
     */
    public boolean atAngle(double targetDegrees) {
        return atAngle(targetDegrees, TurretConstants.kPositionToleranceDegrees);
    }
    
    /**
     * Check if current angle is within rotation limits
     * 
     * @return true if within limits
     */
    public boolean isWithinLimits() {
        double currentAngle = getAngle();
        return currentAngle >= TurretConstants.kMinRotationDegrees && 
               currentAngle <= TurretConstants.kMaxRotationDegrees;
    }
    
    /**
     * Check if current angle is near the minimum limit
     * 
     * @return true if near minimum limit
     */
    public boolean isNearMinLimit() {
        double currentAngle = getAngle();
        return currentAngle <= (TurretConstants.kMinRotationDegrees + 
                               (180.0 - TurretConstants.kWrapAroundThreshold));
    }
    
    /**
     * Check if current angle is near the maximum limit
     * 
     * @return true if near maximum limit
     */
    public boolean isNearMaxLimit() {
        double currentAngle = getAngle();
        return currentAngle >= (TurretConstants.kMaxRotationDegrees - 
                               (180.0 - TurretConstants.kWrapAroundThreshold));
    }
    
    /**
     * Check if turret needs to wrap around to continue tracking
     * This happens when the turret is near a limit and the target is on the opposite side
     * 
     * @param targetAngle The desired target angle in degrees
     * @return true if wrap-around is needed
     */
    public boolean needsWrapAround(double targetAngle) {
        double currentAngle = getAngle();
        
        // If we're near the max limit and target is on the negative side
        if (isNearMaxLimit() && targetAngle < 0) {
            return true;
        }
        
        // If we're near the min limit and target is on the positive side
        if (isNearMinLimit() && targetAngle > 0) {
            return true;
        }
        
        return false;
    }
    
    /**
     * Calculate the shortest path to target angle, considering wrap-around
     * Returns the target angle adjusted for wrap-around if needed
     * 
     * @param targetAngle The desired target angle in degrees
     * @return Adjusted target angle that accounts for wrap-around
     */
    public double calculateShortestPath(double targetAngle) {
        double currentAngle = getAngle();
        
        // If we need to wrap around, return the opposite side target
        if (needsWrapAround(targetAngle)) {
            // If near max limit, wrap to negative side
            if (isNearMaxLimit()) {
                return -TurretConstants.kWrapAroundTargetOffset;
            }
            // If near min limit, wrap to positive side
            else {
                return TurretConstants.kWrapAroundTargetOffset;
            }
        }
        
        // Otherwise, return the target as-is
        return targetAngle;
    }
    
    /**
     * Normalize angle to be within -180 to +180 degrees
     * 
     * @param angle Angle in degrees
     * @return Normalized angle
     */
    public double normalizeAngle(double angle) {
        while (angle > 180.0) {
            angle -= 360.0;
        }
        while (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
    }
    
    /**
     * Clamp angle to be within rotation limits
     * 
     * @param angle Angle in degrees
     * @return Clamped angle
     */
    public double clampToLimits(double angle) {
        return Math.max(TurretConstants.kMinRotationDegrees, 
                       Math.min(TurretConstants.kMaxRotationDegrees, angle));
    }
    
    /**
     * Set turret angle with limit checking
     * This will clamp the angle to be within limits
     * 
     * @param degrees Target angle in degrees
     */
    public void setAngleSafe(double degrees) {
        double clampedAngle = clampToLimits(degrees);
        setAngle(clampedAngle);
        
        // Update dashboard
        SmartDashboard.putBoolean(TurretConstants.kSmartDashboardPrefix + TurretConstants.kAtLimitKey, 
                                 clampedAngle != degrees);
    }
}
