package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Turret subsystem for CAN device ID 6 on RIO bus.
 * Provides dashboard controls to start/stop spinning the turret motor.
 * Also supports position control for autonomous aiming.
 */
public class TestMotorSubsystem extends SubsystemBase {
    private final TalonFX turretMotor;
    private double targetSpeed = 0.0;
    private boolean isSpinning = false;
    
    // Position control
    private final PositionVoltage positionControl = new PositionVoltage(0);
    private static final double GEAR_RATIO = 1.0; // Adjust based on your turret gearing
    private static final double ROTATIONS_PER_DEGREE = GEAR_RATIO / 360.0;
    
    // Default test speed (can be adjusted from dashboard)
    private static final double DEFAULT_SPIN_SPEED = 0.3; // 30% power
    
    public TestMotorSubsystem() {
        // Initialize TalonFX turret motor on CAN ID 6 (RIO bus)
        turretMotor = new TalonFX(6, "rio");
        
        // Configure motor to brake mode for safety
        turretMotor.setNeutralMode(NeutralModeValue.Brake);
        
        // Initialize dashboard controls
        initializeDashboard();
        
        System.out.println("Turret subsystem initialized on CAN ID 6 (RIO bus)");
    }
    
    /**
     * Initialize SmartDashboard controls and displays
     */
    private void initializeDashboard() {
        // Control buttons (these will appear in SmartDashboard/Shuffleboard)
        SmartDashboard.putBoolean("Turret/Start Spin", false);
        SmartDashboard.putBoolean("Turret/Stop Spin", false);
        
        // Speed control
        SmartDashboard.putNumber("Turret/Spin Speed", DEFAULT_SPIN_SPEED);
        
        // Status displays
        SmartDashboard.putBoolean("Turret/Is Spinning", false);
        SmartDashboard.putNumber("Turret/Current Speed", 0.0);
        SmartDashboard.putNumber("Turret/Motor Current (A)", 0.0);
        SmartDashboard.putNumber("Turret/Motor Temp (C)", 0.0);
        SmartDashboard.putString("Turret/Status", "Stopped");
    }
    
    @Override
    public void periodic() {
        // Check dashboard buttons
        checkDashboardControls();
        
        // Apply motor speed
        turretMotor.set(targetSpeed);
        
        // Update dashboard status
        updateDashboard();
    }
    
    /**
     * Check dashboard buttons and respond to user input
     */
    private void checkDashboardControls() {
        // Check Start Spin button
        boolean startPressed = SmartDashboard.getBoolean("Turret/Start Spin", false);
        if (startPressed && !isSpinning) {
            startSpin();
            // Reset button
            SmartDashboard.putBoolean("Turret/Start Spin", false);
        }
        
        // Check Stop Spin button
        boolean stopPressed = SmartDashboard.getBoolean("Turret/Stop Spin", false);
        if (stopPressed && isSpinning) {
            stopSpin();
            // Reset button
            SmartDashboard.putBoolean("Turret/Stop Spin", false);
        }
        
        // Update speed from dashboard if motor is spinning
        if (isSpinning) {
            double dashboardSpeed = SmartDashboard.getNumber("Turret/Spin Speed", DEFAULT_SPIN_SPEED);
            // Clamp speed between -1.0 and 1.0
            targetSpeed = Math.max(-1.0, Math.min(1.0, dashboardSpeed));
        }
    }
    
    /**
     * Update dashboard with current motor status
     */
    private void updateDashboard() {
        SmartDashboard.putBoolean("Turret/Is Spinning", isSpinning);
        SmartDashboard.putNumber("Turret/Current Speed", targetSpeed);
        
        // Get motor telemetry
        double current = turretMotor.getSupplyCurrent().getValueAsDouble();
        double temp = turretMotor.getDeviceTemp().getValueAsDouble();
        
        SmartDashboard.putNumber("Turret/Motor Current (A)", current);
        SmartDashboard.putNumber("Turret/Motor Temp (C)", temp);
        
        // Update status string
        String status = isSpinning ? 
            String.format("Spinning at %.1f%%", targetSpeed * 100) : 
            "Stopped";
        SmartDashboard.putString("Turret/Status", status);
    }
    
    /**
     * Start spinning the turret motor at the dashboard-specified speed
     */
    public void startSpin() {
        double speed = SmartDashboard.getNumber("Turret/Spin Speed", DEFAULT_SPIN_SPEED);
        targetSpeed = Math.max(-1.0, Math.min(1.0, speed));
        isSpinning = true;
        System.out.println("Turret: Started spinning at " + (targetSpeed * 100) + "%");
    }
    
    /**
     * Stop spinning the turret motor
     */
    public void stopSpin() {
        targetSpeed = 0.0;
        isSpinning = false;
        System.out.println("Turret: Stopped spinning");
    }
    
    /**
     * Set motor speed directly (for command-based control)
     * 
     * @param speed Motor speed from -1.0 to 1.0
     */
    public void setSpeed(double speed) {
        targetSpeed = Math.max(-1.0, Math.min(1.0, speed));
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
        System.out.println("Turret: EMERGENCY STOP");
    }
    
    /**
     * Set turret to a specific angle using position control
     * 
     * @param degrees Target angle in degrees (0-360)
     */
    public void setAngle(double degrees) {
        double rotations = degrees * ROTATIONS_PER_DEGREE;
        turretMotor.setControl(positionControl.withPosition(rotations));
        SmartDashboard.putNumber("Turret/Target Angle", degrees);
    }
    
    /**
     * Get current turret angle in degrees
     * 
     * @return Current angle in degrees
     */
    public double getAngle() {
        double rotations = turretMotor.getPosition().getValueAsDouble();
        return rotations / ROTATIONS_PER_DEGREE;
    }
    
    /**
     * Reset turret encoder to zero at current position
     */
    public void resetEncoder() {
        turretMotor.setPosition(0);
        System.out.println("Turret: Encoder reset to 0");
    }
    
    /**
     * Check if turret is at target angle (within tolerance)
     * 
     * @param targetDegrees Target angle in degrees
     * @param toleranceDegrees Tolerance in degrees (default 2.0)
     * @return true if within tolerance
     */
    public boolean atAngle(double targetDegrees, double toleranceDegrees) {
        double currentAngle = getAngle();
        return Math.abs(currentAngle - targetDegrees) <= toleranceDegrees;
    }
    
    /**
     * Check if turret is at target angle with default tolerance (2 degrees)
     * 
     * @param targetDegrees Target angle in degrees
     * @return true if within 2 degrees of target
     */
    public boolean atAngle(double targetDegrees) {
        return atAngle(targetDegrees, 2.0);
    }
}
