// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Mechanisms;

/**
 * Subsystem for the Launcher mechanism, which includes:
 * - Turret (Position - Motion Magic) for aiming left and right
 * - Launcher (Velocity - Velocity Closed Loop) for shooting balls at certain speed
 * - Hood (Position - Motion Magic) for adjusting angle of launcher
 */

//launch = velo  hood = pos

public class Launcher extends SubsystemBase {
  //Motors
    private final TalonFX turretMotor;
    private final TalonFX launcherMotor; 
    private final TalonFX hoodMotor; 
    
  //Controls
    // Separate MotionMagicVoltage objects for turret and hood.
    // CTRE Phoenix 6 holds a reference to the request object — sharing one object
    // between two motors means the last withPosition() call overwrites the previous
    // one, causing the hood to receive the turret's target (and vice versa) every loop.
    // Slot 0 — original MotionMagicVoltage (retained for reference / easy rollback)
    // Slot 1 — DynamicMotionMagicTorqueCurrentFOC (active control mode)
    // Constructor: (Position rots, CruiseVelocity rps, Acceleration rps/s) — Jerk defaults to 0.0 (disabled).
    // Starting very slow: 2 rps cruise, 2 rps/s accel. Raise once motion is confirmed safe on the robot.
    // raised to 10 rps, 10 rps/s on 3/14/26
    // Previous (rollback): new DynamicMotionMagicTorqueCurrentFOC(0.0, 10.0, 10.0).withSlot(1)
    private final DynamicMotionMagicTorqueCurrentFOC m_dynMMTorqueTurret =
        new DynamicMotionMagicTorqueCurrentFOC(0.0, 90.0, 90.0).withSlot(1); // Rebalanced for smoother tracking (reduced from 150.0)
    private final MotionMagicVoltage m_mmreqHood   = new MotionMagicVoltage(0);

    private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut m_brake = new NeutralOut();
    private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);

  //Mechanisms visualization  
    private final Mechanisms m_mechanisms = new Mechanisms();

  // State tracking
    private double lastHoodAngle = 0;
    private double lastLauncherVelocity = 0;
    private double lastTurretAngle = 0;
    private int periodicCounter = 0; // Throttle SmartDashboard updates to reduce NT load
  
  public Launcher() {
    // Initialize Motors using constants
    launcherMotor = new TalonFX(LauncherConstants.kLauncherMotorID);
    hoodMotor = new TalonFX(LauncherConstants.kHoodMotorID);
    turretMotor = new TalonFX(LauncherConstants.kTurretMotorID);
    
    //Config
    TalonFXConfiguration cfgLaunch = new TalonFXConfiguration();
    TalonFXConfiguration cfgHood = new TalonFXConfiguration();
    TalonFXConfiguration cfgTurret = new TalonFXConfiguration();

    /* ------------------------------LAUNCHER : VelocityClosedLoop -> --------------------------------- */
     /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    cfgLaunch.Slot0.kS = 0;    // Static friction feedforward (V) — set to ~0.25 if motor stalls at low speeds
    cfgLaunch.Slot0.kV = 0.12; // Kraken X60: 500 kV → 8.333 rps/V → kV = 1/8.333 = 0.12 V/rps
                                // FIXED: was incorrectly set to 0.0085 (14× too low), causing
                                // feedforward to produce ~0.5V instead of the ~7–10V needed.
    // kP raised from 0.01 → 0.3 to enable active speed recovery when a ball loads the flywheel.
    // kV handles steady-state (feedforward); kP handles transient drops.
    // At kP=0.3: a 10 RPS drop (ball impact) produces 3 V of corrective output — enough to
    // recover speed quickly without causing oscillation at steady state.
    // Tune upward (0.4–0.6) if recovery is still too slow; downward if flywheel oscillates.
    cfgLaunch.Slot0.kP = 0.3;  // 1 RPS error → 0.3 V correction
    cfgLaunch.Slot0.kI = 0;    // No integral
    cfgLaunch.Slot0.kD = 0;    // No derivative
    // Peak output raised to 12 V so the motor can reach the highest lookup-table target
    // (-85 RPS × 0.12 V/rps = 10.2 V feedforward — would have been clipped at the old 8 V cap).
    cfgLaunch.Voltage.withPeakForwardVoltage(Volts.of(12))
      .withPeakReverseVoltage(Volts.of(-12));

    /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    cfgLaunch.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
    cfgLaunch.Slot1.kP = 10; // An error of 1 rotation per second results in 5 A output
    cfgLaunch.Slot1.kI = 0; // No output for integrated error
    cfgLaunch.Slot1.kD = 0; // No output for error derivative
    // Peak output of 40 A
    cfgLaunch.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
      .withPeakReverseTorqueCurrent(Amps.of(-40));
     /* Retry config apply up to 5 times, report if failure */

     /* -------------------------------------- TURRET & HOOD : Motion Magic -> ----------------------------------*/
    // ============ CONFIGURE TURRET POSITION CONTROL ============ //
    //
    // SensorToMechanismRatio = 1.0 → getTurretPosition() returns raw motor rotations.
    // LauncherConstants.kSensorToMechanismRatio is declared but never initialized
    // (Java defaults it to 0.0), which causes CTRE to divide by zero internally,
    // producing NaN/Infinity positions and severe motor oscillation. DO NOT use it here.
    cfgTurret.Feedback.SensorToMechanismRatio = 1.0;
    
    cfgTurret.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.kPhysicalLimitRotations;
    cfgTurret.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfgTurret.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -TurretConstants.kPhysicalLimitRotations;
    cfgTurret.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    // Configure Motion Magic cruise/accel/jerk in raw motor rotations per second
    MotionMagicConfigs mmTurret = cfgTurret.MotionMagic;
    //MotionMagicConfigs mmTurret = cfgTurret
    mmTurret.withMotionMagicCruiseVelocity(RotationsPerSecond.of(95)) // 40.0, 60, 80
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(380)) // 20.0, 60, 80
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0));

    Slot0Configs slot0Turret = cfgTurret.Slot0;
    // kS removed (set to 0.0): static friction feedforward was causing a 0.1 V kick
    // every time the motor was commanded, which caused oscillation near the target.
    // Same fix that resolved hood oscillation.
    slot0Turret.kS = 0.0;
    slot0Turret.kV = 8.8; // Velocity feedforward: 1 rps → 0.12 V was 0.12
    slot0Turret.kA = 0.0; // Acceleration feedforward
    // kP is calibrated for RAW MOTOR ROTATIONS (SensorToMechanismRatio = 1.0).
    // Gear ratio = 115/3 ≈ 38.33 → 1 raw rotation ≈ 9.4°.
    // At kP=4: a 1-rotation error (~9.4°) produces 4 V output — conservative for first test.
    slot0Turret.kP = 50.0;
    slot0Turret.kI = 0;    // No integral
    // kD increased from 1.5 → 2.0 for more damping near the target position.
    slot0Turret.kD = 2.0;

    // Slot 1 uses DynamicMotionMagicTorqueCurrentFOC gains in amps.
    Slot1Configs slot1Turret = cfgTurret.Slot1;
    slot1Turret.kS = 2.0;   // Static friction feedforward (A) — small kick to overcome stiction
    slot1Turret.kV = 0.0;   // Velocity feedforward (A per rps) — start at 0; add if motor lags
    slot1Turret.kA = 0.0;   // Acceleration feedforward (A per rps/s) — not needed initially
    // Previous (rollback): kP=5.0, kD=0.1
    slot1Turret.kP = 9.0;   // Reduced to limit overshoot/hunting while tracking
    slot1Turret.kI = 0.0;   // No integral
    slot1Turret.kD = 0.3;   // Slightly increased damping for smoother settle

    // Peak torque current limits for turret — conservative for initial testing.
    // Raise toward 40–60 A once motion is confirmed safe and gains are tuned.
    // Previous (rollback): ±20 A
    cfgTurret.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(35))
        .withPeakReverseTorqueCurrent(Amps.of(-35));

    // ============ CONFIGURE HOOD POSITION CONTROL ============ //
    // SensorToMechanismRatio = 1.0 → positions are in raw motor rotations.
    // Physical range: 0.0 (22° launch) to 11.5 (62° launch) motor rotations.
    cfgHood.Feedback.SensorToMechanismRatio = 1.0;

    // Software limits — prevent the hood from travelling past its physical endpoints.
    cfgHood.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
    cfgHood.SoftwareLimitSwitch.ForwardSoftLimitThreshold = LauncherConstants.kHoodMaxRotations; // 11.5 rot → 62°
    cfgHood.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
    cfgHood.SoftwareLimitSwitch.ReverseSoftLimitThreshold = LauncherConstants.kHoodMinRotations; // 0.0 rot → 22°

    // Configure Motion Magic for hood position control.
    MotionMagicConfigs mmHood = cfgHood.MotionMagic;
    mmHood.withMotionMagicCruiseVelocity(RotationsPerSecond.of(30)) // 8.0, 20
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(15)) // 4.0, 10
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0.0)); // 0 = jerk limiting disabled

    Slot0Configs slot0Hood = cfgHood.Slot0;
    slot0Hood.kS = 0.0;  // Remove static kick to reduce on-move oscillation
    slot0Hood.kV = 8.4;  // Velocity feedforward: tuned for hood mechanism
    slot0Hood.kA = 0.0;  // Acceleration feedforward (not used)
    slot0Hood.kP = 22.0; // Lower proportional gain for smoother response
    slot0Hood.kI = 0;    // No integral
    slot0Hood.kD = 0.25; // Add damping near target to reduce ringing


    // Status Code for ALL
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = turretMotor.getConfigurator().apply(cfgTurret);
      status = hoodMotor.getConfigurator().apply(cfgHood);
      status = launcherMotor.getConfigurator().apply(cfgLaunch);
      if (status.isOK()) break;
    }
  }

   // ========================= TURRET POSITION CONTROL METHODS ========================= //
   
  public Command setTurretVoltage(double speed){
    return Commands.run(() -> turretMotor.set(speed), this);
  }
  
  /**
   * Sets the turret to a specific position using DynamicMotionMagicTorqueCurrentFOC (Slot 1).
   *
   * <p>Profile: CruiseVelocity=2 rps, Acceleration=2 rps/s — intentionally very slow.
   * Increase these values in {@code m_dynMMTorqueTurret} once motion is confirmed safe.
   *
   * @param mechRotations Target position in raw motor rotations
   */
  public void setTurretPosition(double mechRotations) {
    lastTurretAngle = mechRotations;
    // Slot 1 — DynamicMotionMagicTorqueCurrentFOC (active)
    turretMotor.setControl(m_dynMMTorqueTurret.withPosition(mechRotations));
  }
  
  /**
   * Gets the current pivot position in mechanism rotations
   * @return Current position in rotations
   */
  public double getTurretPosition() {
    return turretMotor.getPosition().getValueAsDouble();
  }

  /**
   * Gets the current pivot velocity in rotations per second
   * @return Current velocity in rotations/sec
   */
  public double getTurretVelocity() {
    return turretMotor.getVelocity().getValueAsDouble();
  }
  
  /**
   * Gets the error between target and current pivot position
   * @return Position error in rotations
   */
  public double getTurretError() {
    return getTurretPosition() - lastTurretAngle;
  }
  
  /**
   * Checks if pivot is at the target position within tolerance
   * @return True if at target position
   */
  public boolean isTurretAtTarget() {
    return Math.abs(getTurretError()) < LauncherConstants.kTurretPositionTolerance;
  }
  
  // ========================= HOOD POSITION CONTROL METHODS ========================= //
   
  /**
   * Sets the pivot to a specific position in rotations
   * @param mechRotations Target position in mechanism rotations
   */
  public void setHoodPosition(double mechRotations) {
    lastHoodAngle = mechRotations;
    hoodMotor.setControl(m_mmreqHood.withPosition(mechRotations).withSlot(0));
  }
  
  /**
   * Gets the current pivot position in mechanism rotations
   * @return Current position in rotations
   */
  public double getHoodPosition() {
    return hoodMotor.getPosition().getValueAsDouble();
  }

  /**
   * Gets the current pivot velocity in rotations per second
   * @return Current velocity in rotations/sec
   */
  public double getHoodVelocity() {
    return hoodMotor.getVelocity().getValueAsDouble();
  }
  
  /**
   * Gets the error between target and current pivot position
   * @return Position error in rotations
   */
  public double getHoodError() {
    return getHoodPosition() - lastHoodAngle;
  }
  
  /**
   * Checks if hood is at the target position within tolerance.
   * Uses {@link LauncherConstants#kHoodPositionToleranceRotations} (0.2 motor rotations).
   * @return True if at target position
   */
  public boolean isHoodAtTarget() {
    return Math.abs(getHoodError()) < LauncherConstants.kHoodPositionToleranceRotations;
  }

  // ==================== COLLECTION VELOCITY CONTROL METHODS ====================
  
  /**
   * Sets the collection motor velocity using closed-loop VelocityVoltage control.
   * Requires a working encoder. If the flywheel does not reach speed, use
   * {@link #setCollectDutyCycle(double)} instead (open-loop, always works).
   *
   * @param velocityRPS Velocity in rotations per second (negative = shooting direction)
   */
  public void setCollectVelocity(double velocityRPS) {
    if (!isFlywheelEnabled()) {
      lastLauncherVelocity = 0.0;
      launcherMotor.setControl(m_brake);
      return;
    }
    lastLauncherVelocity = velocityRPS;
    launcherMotor.setControl(m_velocityVoltage.withVelocity(velocityRPS));
  }

  /**
   * Drives the flywheel open-loop using DutyCycleOut — the same control mode used by
   * {@link #launchFromTowerLauncher()}, which is known to work on this robot.
   *
   * <p>The duty cycle is calculated from the target RPS assuming Kraken X60 free speed
   * of 100 RPS at 12 V:  {@code dutyCycle = velocityRPS / 100.0}
   *
   * <p>Use this instead of {@link #setCollectVelocity(double)} when closed-loop velocity
   * control is unreliable (e.g., encoder feedback issues).
   *
   * @param velocityRPS Target velocity in RPS (negative = shooting direction).
   *                    Converted to duty cycle and clamped to [-1, 1].
   */
  public void setCollectDutyCycle(double velocityRPS) {
    if (!isFlywheelEnabled()) {
      lastLauncherVelocity = 0.0;
      launcherMotor.setControl(m_brake);
      return;
    }
    lastLauncherVelocity = velocityRPS;
    // Kraken X60 free speed ≈ 100 RPS at 12 V → duty cycle = RPS / 100
    double dutyCycle = velocityRPS / 100.0;
    dutyCycle = Math.max(-1.0, Math.min(1.0, dutyCycle));
    launcherMotor.setControl(m_dutyCycleOut.withOutput(dutyCycle));
  }

  /**
   * Dashboard master toggle for allowing flywheel commands.
   * When false, velocity/duty requests are ignored and the launcher motor is braked.
   */
  private boolean isFlywheelEnabled() {
    return SmartDashboard.getBoolean("Launcher/Flywheel Enabled", true);
  }

  /**
   * Returns {@code true} if the flywheel is within ±5 RPS of its last commanded velocity.
   * Uses absolute values so the check is sign-agnostic (works regardless of sensor polarity).
   * Published to SmartDashboard as {@code "Launcher/Flywheel At Speed"} every periodic loop.
   */
  public boolean isFlywheelAtSpeed() {
    if (Math.abs(lastLauncherVelocity) < 1.0) return false; // not commanded — treat as not at speed
    double actual = Math.abs(getCollectVelocity());
    double target = Math.abs(lastLauncherVelocity);
    return Math.abs(actual - target) < frc.robot.Constants.ShootingArc.kLauncherVelocityTolerance;
  }
  
  /**
   * Stops the collection motor
   */
  public void stopLauncher() {
    launcherMotor.setControl(m_brake);
  }

  // ==================== TURRET HELPERS FOR SHOOT-ON-ARC ====================

  /**
   * Gets the turret position converted to degrees.
   * Since SensorToMechanismRatio is not configured, getTurretPosition() returns
   * raw motor rotations. This converts using the known gear ratio (115/3).
   *
   * @return Turret angle in degrees (0 = center, positive = CCW)
   */
  public double getTurretPositionDegrees() {
      return getTurretPosition() / TurretConstants.kRotationsPerDegree;
  }

  /**
   * Checks whether the turret is within its physical hard-stop limits.
   * Physical limits: ±kPhysicalLimitRotations raw motor rotations.
   *
   * @return true if within limits, false if past a hard stop
   */
  public boolean isTurretWithinLimits() {
      return Math.abs(getTurretPosition()) <= TurretConstants.kPhysicalLimitRotations;
  }

  /**
   * Stops the turret motor immediately (brake mode).
   * For use inside command execute()/end() methods where a Command cannot be returned.
   */
  public void stopTurretDirect() {
      turretMotor.setControl(m_brake);
  }

  /**
   * Nudges the turret position by the given degrees using Motion Magic position control.
   * Safe to call without subsystem ownership (like stopTurretDirect).
   *
   * <p>Sign convention (matches ShootingArcManager):
   *   Positive degrees = CCW (left) = increase raw motor rotations.
   *   Negative degrees = CW  (right) = decrease raw motor rotations.
   *
   * @param degrees Degrees to nudge (positive = CCW/left, negative = CW/right)
   */
  public void nudgeTurretDirect(double degrees) {
      double currentRot = getTurretPosition();
      double deltaRot   = degrees * TurretConstants.kRotationsPerDegree;
      double newRot     = currentRot + deltaRot;
      // Clamp to physical hard-stop limits
      newRot = Math.max(-TurretConstants.kPhysicalLimitRotations,
               Math.min( TurretConstants.kPhysicalLimitRotations, newRot));
      setTurretPosition(newRot);
  }
  
  /**
   * Gets the current collection motor velocity
   * @return Velocity in rotations per second
   */
  public double getCollectVelocity() {
    return launcherMotor.getVelocity().getValueAsDouble();
  }

  // ==================== MONITORING METHODS ====================
  
   /**
   * Gets the launcher motor current draw
   * @return Current in amps
   */
  public double getTurretCurrent() {
    return turretMotor.getSupplyCurrent().getValueAsDouble();
  }
  

  /**
   * Gets the launcher motor current draw
   * @return Current in amps
   */
  public double getHoodCurrent() {
    return hoodMotor.getSupplyCurrent().getValueAsDouble();
  }
  
  /**
   * Gets the collection motor current draw
   * @return Current in amps
   */
  public double getCollectCurrent() {
    return launcherMotor.getSupplyCurrent().getValueAsDouble();
  }
  
  /**
   * Gets the pivot motor temperature
   * @return Temperature in Celsius
   */
  public double getTurretTemperature() {
    return turretMotor.getDeviceTemp().getValueAsDouble();
  }
  
  /**
   * Gets the pivot motor temperature
   * @return Temperature in Celsius
   */
  public double getHoodTemperature() {
    return hoodMotor.getDeviceTemp().getValueAsDouble();
  }
  
  /**
   * Gets the collection motor temperature
   * @return Temperature in Celsius
   */
  public double getCollectTemperature() {
    return launcherMotor.getDeviceTemp().getValueAsDouble();
  }

  // ==================== COMMAND FACTORY METHODS ====================
  
  public Command setHoodVoltage(double speed){
    return Commands.run(() -> hoodMotor.set(speed), this);
  }

  public Command turretGoHome(){
    return Commands.sequence(
        Commands.runOnce(() -> setTurretPosition(0), this),
        Commands.waitUntil(this::isTurretAtTarget).withTimeout(1.5)
    );
  }
  
  

  // ==================== LAUNCH FROM TOWER COMMAND ====================

  /**
   * LaunchFromTower launcher command.
   * Runs the launcher flywheel at a fixed duty cycle and moves the hood to the
   * configured tower-launch position while the button is held.
   *
   *   - Launcher motor (CAN 8): {@link TurretConstants#flywheelDutyCycleOut}
   *   - Hood motor     (CAN 9): {@link TurretConstants#hoodTowerPosition} (motor rotations,
   *                             Motion Magic position control)
   *
   * Designed to be used in parallel with Spindexer.launchFromTower().
   * Runs while the button is held; flywheel stops on release and the default
   * command (retractHood) automatically retracts the hood afterward.
   *
   * <p>To tune the hood angle, adjust {@link TurretConstants#hoodTowerPosition}:
   * <ul>
   *   <li>0.0  rot = 68° from horizontal (steepest — high arc)</li>
   *   <li>11.5 rot = 28° from horizontal (flattest — low arc)</li>
   * </ul>
   *
   * @return Command that drives the flywheel and positions the hood for tower shooting
   */
  public Command launchFromTowerLauncher() {
    return Commands.run(() -> {
      // Turret: center to zero (robot-forward) for tower shots
      setTurretPosition(0);
      // Hood: move to tower launch position (Motion Magic position control)
      setHoodPosition(TurretConstants.hoodTowerPosition);
      // Launcher flywheel: velocity closed-loop (VelocityVoltage) — same control mode
      // as ShootFromPointCommand. kV handles steady-state; kP (0.3) recovers speed
      // when a ball loads the flywheel. kFlywheelTowerRPS = -65 RPS (≡ -0.65 duty cycle).
      setCollectVelocity(TurretConstants.kFlywheelTowerRPS);
    }, this).finallyDo(() -> {
      // Stop flywheel when command ends (button released).
      // Hood retraction is handled automatically by the default command (retractHood()).
      launcherMotor.setControl(m_brake);
    });
  }

  /**
   * Command to run intake collection at intake speed
   * @return Command that runs collection motor
   */
  public Command runLauncherIntake() {
    return Commands.run(() -> {
      setCollectVelocity(LauncherConstants.kLauncherIntakeVelocity);
    }, this);
  }
  
  public Command stopLauncherSpin() {
    return Commands.runOnce(this::stopLauncher, this);
  }

  /**
   * Command to run intake collection at outtake speed
   * @return Command that ejects game pieces
   */
  public Command runLauncherOuttake() {
    return Commands.run(() -> {
      setCollectVelocity(LauncherConstants.kCollectOuttakeVelocity);
    }, this);
  }
  
  /**
   * Command to hold game piece with low speed
   * @return Command that holds game piece
   */
  public Command holdGamePiece() {
    return Commands.run(() -> {
      setCollectVelocity(LauncherConstants.kCollectHoldVelocity);
    }, this);
  }
  
  /**
   * Command to stop collection motor
   * @return Command that stops collection
   */
  public Command stopCollection() {
    return Commands.runOnce(() -> {
      stopLauncher();
    }, this);
  }
 

  /** Retracts hood to stow, brakes, then idles flywheel logic in default state. */
  public Command retractHood() {
    return retractHood(() -> false);
  }

  /**
   * Default command variant: retracts hood, then conditionally idles flywheel based on intake home state.
   *
   * <p>When {@code intakeAtHomeSupplier.getAsBoolean()} is true, flywheel is forced off even if
   * {@link LauncherConstants#kFlywheelIdleEnabled} is enabled. This prevents idling while intake
   * is near/at the home position.
   *
   * @param intakeAtHomeSupplier true when intake pivot is at/near home
   * @return Command that retracts hood then runs conditional flywheel idle logic
   */
  public Command retractHood(BooleanSupplier intakeAtHomeSupplier) {
    return Commands.sequence(
        // Phase 1: command retract position once
        Commands.runOnce(() -> setHoodPosition(0.5), this),
        // Phase 2: wait until hood arrives at 0.5 rot
        Commands.waitUntil(this::isHoodAtTarget),
        // Phase 3: brake hood — no active voltage, no oscillation, no heating
        Commands.runOnce(() -> hoodMotor.setControl(m_brake), this),
        // Phase 4: idle forever.
        // Toggle: set LauncherConstants.kFlywheelIdleEnabled = true to spin the flywheel
        // at kFlywheelIdleRPS between shots (reduces spool-up time).
        // Additional gate: if intake is at home, force flywheel off.
        Commands.run(() -> {
            boolean intakeAtHome = intakeAtHomeSupplier.getAsBoolean();

            if (!intakeAtHome && LauncherConstants.kFlywheelIdleEnabled) {
                setCollectVelocity(LauncherConstants.kFlywheelIdleRPS);
            } else {
                stopLauncher();
            }

            // Turret is controlled elsewhere (RobotContainer zone tracking / shooting commands).
            // Do not force hold-at-zero here.
        }, this)
    ).withName("RetractHood");
  }


  /**
   * Command to go to a specific pivot position and wait until reached
   * @param position Target position in rotations
   * @return Command that moves to position and finishes when reached
   */
  public Command goToPivotPosition(double position) {
    return Commands.runOnce(() -> {
      setHoodPosition(position);
    }, this).andThen(Commands.waitUntil(this::isHoodAtTarget));
  }

  @Override
  public void periodic() {
    // Throttle SmartDashboard updates to every 5 loops (~100ms) to reduce NT memory pressure.
    // Mechanism visualization is also throttled here — it only drives a Mechanism2d widget
    // on the dashboard and does not need to update at 50 Hz.
    periodicCounter++;
    if (periodicCounter < 5) return;
    periodicCounter = 0;

    // Update mechanism visualization (~10 Hz is plenty for dashboard display)
    m_mechanisms.update(hoodMotor.getPosition(), hoodMotor.getVelocity());
    m_mechanisms.update(turretMotor.getPosition(), turretMotor.getVelocity());

    // Keep flywheel toggle/status visible in Elastic.
    boolean flywheelEnabled = isFlywheelEnabled();
    SmartDashboard.putBoolean("Launcher/Flywheel Enabled", flywheelEnabled);
    SmartDashboard.putString("Launcher/Flywheel State", flywheelEnabled ? "Enabled" : "Disabled");

    // Always-on flywheel status indicator for Elastic dashboard.
    // Green = flywheel is within tolerance of its commanded target.
    // False when no velocity has been commanded (lastLauncherVelocity ≈ 0).
    SmartDashboard.putBoolean("Launcher/Flywheel At Speed", flywheelEnabled && isFlywheelAtSpeed());

  }
}
