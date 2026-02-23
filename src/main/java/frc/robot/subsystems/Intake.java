package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Mechanisms;

/**
 * Intake subsystem with dual motors:
 * - intakeRollers (velocity control) for spinning intake rollers
 * - intakePivot (position control) for pivoting intake mechanism
 */
public class Intake extends SubsystemBase {
  // Motors
  private final TalonFX intakeRollers;
  private final TalonFX intakePivot;
  
  // Control requests
  private final MotionMagicVoltage m_mmreq = new MotionMagicVoltage(0);
  private final VelocityVoltage m_VelocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(0);
  private final NeutralOut m_brake = new NeutralOut();

  // Mechanisms visualization
  private final Mechanisms m_mechanisms = new Mechanisms();

  // State tracking
  private double pivotTargetPosition = 0.0;
  private boolean rollersEnabled = false; // Tracks whether rollers should be spinning
  private int periodicCounter = 0;        // Throttle SmartDashboard updates to reduce NT load
    
  public Intake() {
    // Initialize motors using constants
    intakeRollers = new TalonFX(IntakeConstants.kIntakeRollersMotorID, IntakeConstants.kCANBusName);
    intakePivot = new TalonFX(IntakeConstants.kIntakePivotMotorID, IntakeConstants.kCANBusName);
    
    // Config
    TalonFXConfiguration cfgRollers = new TalonFXConfiguration();
    TalonFXConfiguration cfgPivot = new TalonFXConfiguration();

    // Need to resolve two different configurations for each motor, so we'll apply them sequentially with different slot configs?? magic AI stuff

    /*  ----------------------INTAKE ROLLERS : VelocityClosedLoop ->------------------------------------ */
    // ============ CONFIGURE COLLECTOR VELOCITY CONTROL ============ //
    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    cfgRollers.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    cfgRollers.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    cfgRollers.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    cfgRollers.Slot0.kI = 0; // No output for integrated error
    cfgRollers.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    cfgRollers.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    cfgRollers.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
    cfgRollers.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
    cfgRollers.Slot1.kI = 0; // No output for integrated error
    cfgRollers.Slot1.kD = 0; // No output for error derivative
    // Peak output of 40 A
    cfgRollers.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
      .withPeakReverseTorqueCurrent(Amps.of(-40));
     /* Retry config apply up to 5 times, report if failure */
    
 
    /* ---------------------------------- INTAKE PIVOT : Motion Magic -> ----------------------------------*/
    // ============ CONFIGURE PIVOT POSITION CONTROL ============ //

    // Configure Gear Ratio using constant
    FeedbackConfigs fdb = cfgPivot.Feedback;
    fdb.SensorToMechanismRatio = IntakeConstants.kSensorToMechanismRatio; //12.8 rotor rotations per mechanism rotation
    cfgPivot.Feedback.SensorToMechanismRatio = IntakeConstants.kSensorToMechanismRatio;
    

    //Configure Motion Magic
    MotionMagicConfigs mm = cfgPivot.MotionMagic;
    mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(.6))
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(.9))
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
 

    Slot0Configs slot0 = cfgPivot.Slot0;
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0.kV = 13.0; // A velocity target of 1 rps results in 13 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 20; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

    // Software limits — prevent pivot from travelling outside safe range
    // Upper limit: kPivotHomePosition (0.0) — pivot cannot raise above home/stowed
    // Lower limit: kPivotDeployCollectPosition (-1.36) — pivot cannot lower past deploy position
    cfgPivot.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfgPivot.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.kPivotHomePosition; // hard stop at 0.0 (home)
    cfgPivot.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfgPivot.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.kPivotDeployCollectPosition; // hard stop at -1.36


    // Apply configs to correct motors (rollers config → rollers motor, pivot config → pivot motor)
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = intakeRollers.getConfigurator().apply(cfgRollers);
      status = intakePivot.getConfigurator().apply(cfgPivot);
      if (status.isOK()) break;
    }
  }

  // ==================== PIVOT POSITION CONTROL METHODS ====================
  
  /**
   * Sets the pivot to a specific position in rotations
   * @param mechRotations Target position in mechanism rotations
   */
  public void setPivotPosition(double mechRotations) {
    pivotTargetPosition = mechRotations;
    intakePivot.setControl(m_mmreq.withPosition(mechRotations).withSlot(0));
  }
  
  /**
   * Gets the current pivot position in mechanism rotations
   * @return Current position in rotations
   */
  public double getPivotPosition() {
    return intakePivot.getPosition().getValueAsDouble();
  }

  /**
   * Gets the current pivot velocity in rotations per second
   * @return Current velocity in rotations/sec
   */
  public double getPivotVelocity() {
    return intakePivot.getVelocity().getValueAsDouble();
  }
  
  /**
   * Gets the error between target and current pivot position
   * @return Position error in rotations
   */
  public double getPivotError() {
    return getPivotPosition() - pivotTargetPosition;
  }
  
  /**
   * Checks if pivot is at the target position within tolerance
   * @return True if at target position
   */
  public boolean isPivotAtTarget() {
    return Math.abs(getPivotError()) < IntakeConstants.kPivotPositionTolerance;
  }

  // ==================== COLLECTION VELOCITY CONTROL METHODS ====================
  
  /**
   * Sets the collection motor velocity
   * @param velocityRPS Velocity in rotations per second
   */
  public void setRollersVelocity(double velocityRPS) {
    intakeRollers.setControl(m_VelocityVoltage.withVelocity(velocityRPS));
  }
  
  /**
   * Stops the collection motor
   */
  public void stopRollers() {
    intakeRollers.setControl(m_brake);
  }
  
  /**
   * Gets the current collection motor velocity
   * @return Velocity in rotations per second
   */
  public double getRollersVelocity() {
    return intakeRollers.getVelocity().getValueAsDouble();
  }

  // ==================== MONITORING METHODS ====================
  
  /**
   * Gets the pivot motor current draw
   * @return Current in amps
   */
  public double getPivotCurrent() {
    return intakePivot.getSupplyCurrent().getValueAsDouble();
  }
  
  /**
   * Gets the collection motor current draw
   * @return Current in amps
   */
  public double getRollersCurrent() {
    return intakeRollers.getSupplyCurrent().getValueAsDouble();
  }
  
  /**
   * Gets the pivot motor temperature
   * @return Temperature in Celsius
   */
  public double getPivotTemperature() {
    return intakePivot.getDeviceTemp().getValueAsDouble();
  }
  
  /**
   * Gets the collection motor temperature
   * @return Temperature in Celsius
   */
  public double getRollersTemperature() {
    return intakeRollers.getDeviceTemp().getValueAsDouble();
  }

  // ==================== COMMAND FACTORY METHODS ====================
  
  /**
   * Command to move pivot to stowed position
   * @return Command that stows the intake
   */
  public Command stowIntake() {
    return Commands.runOnce(() -> {
      setPivotPosition(IntakeConstants.kPivotStowedPosition);
    }, this);
  }
  
  /**
   * Command to move pivot to collect position
   * @return Command that extends intake for collecting
   */
  public Command extendForCollect() {
    return Commands.runOnce(() -> {
      setPivotPosition(IntakeConstants.kPivotCollectPosition);
    }, this);
  }
  
  /**
   * Command to move pivot to high scoring position
   * @return Command that positions intake for high scoring
   */
  public Command positionForScoreHigh() {
    return Commands.runOnce(() -> {
      setPivotPosition(IntakeConstants.kPivotScoreHighPosition);
    }, this);
  }
  
  /**
   * Command to move pivot to low scoring position
   * @return Command that positions intake for low scoring
   */
  public Command positionForScoreLow() {
    return Commands.runOnce(() -> {
      setPivotPosition(IntakeConstants.kPivotScoreLowPosition);
    }, this);
  }
  
  public Command setIntakeRollersVoltage(double speed){
    return Commands.run(() -> {
      intakeRollers.set(speed);
    }, this);
  }

  public Command setIntakePivotVoltage(double speed){
    return Commands.run(() -> {
      intakePivot.set(speed);
    }, this);
  }

  /**
   * Command to run intake collection at intake speed
   * @return Command that runs collection motor
   */
  public Command runIntake() {
    return Commands.run(() -> {
      setRollersVelocity(IntakeConstants.kRollersIntakeVelocity);
    }, this);
  }
  
  /**
   * Command to run intake collection at outtake speed
   * @return Command that ejects game pieces
   */
  public Command runOuttake() {
    return Commands.run(() -> {
      setRollersVelocity(IntakeConstants.kRollersOuttakeVelocity);
    }, this);
  }
  
  /**
   * Command to hold game piece with low speed
   * @return Command that holds game piece
   */
  public Command holdGamePiece() {
    return Commands.run(() -> {
      setRollersVelocity(IntakeConstants.kRollersHoldVelocity);
    }, this);
  }
  
  /**
   * Command to stop collection motor
   * @return Command that stops collection
   */
  public Command stopRollersSpin() {
    return Commands.runOnce(() -> {
      stopRollers();
    }, this);
  }
  
  /**
   * Command to go to a specific pivot position and wait until reached
   * @param position Target position in rotations
   * @return Command that moves to position and finishes when reached
   */
  public Command goToPivotPosition(double position) {
    return Commands.runOnce(() -> {
      setPivotPosition(position);
    }, this).andThen(Commands.waitUntil(this::isPivotAtTarget));
  }

  /**
   * Command to deploy the intake pivot to -1.2 rotations (Motion Magic) and start
   * the intake rollers. Designed for toggleOnTrue():
   *   - First press: deploys pivot to -1.2 and starts rollers
   *   - Second press (cancel): stops rollers and retracts pivot to home (-0.1)
   * Rollers can also be independently toggled via toggleRollers() without interrupting pivot.
   * @return Command that deploys pivot and spins rollers, retracts on cancel
   */
  public Command intakeDeployCollect() {
    return Commands.runOnce(() -> rollersEnabled = true, this)
      .andThen(Commands.run(() -> {
        setPivotPosition(IntakeConstants.kPivotDeployCollectPosition);
        if (rollersEnabled) {
          setRollersVelocity(IntakeConstants.kRollersIntakeVelocity);
        } else {
          stopRollers();
        }
      }, this))
      .finallyDo(() -> {
        // On cancel (second press): stop rollers and retract pivot to home position
        rollersEnabled = false;
        stopRollers();
        setPivotPosition(IntakeConstants.kPivotHomePosition);
      });
  }

  /**
   * Toggles the intake rollers on or off without interrupting the pivot control command.
   * This command intentionally has NO subsystem requirement so it does not cancel
   * intakeDeployCollect(). The rollersEnabled flag is read each loop by intakeDeployCollect().
   * @return Command that toggles roller state
   */
  public Command toggleRollers() {
    return Commands.runOnce(() -> {
      rollersEnabled = !rollersEnabled;
      if (!rollersEnabled) stopRollers();
    }); // No subsystem requirement — will NOT interrupt intakeDeployCollect()
  }

  /**
   * Manually lowers the intake pivot while the button is held (X button).
   * Runs the pivot motor at -kPivotManualSpeed (downward) continuously.
   * On release: stops the motor and holds the current position via Motion Magic.
   * The TalonFX reverse soft limit enforces the floor at kPivotDeployCollectPosition (-1.36).
   * @return Command that lowers pivot while held, holds on release
   */
  public Command lowerIntakeManual() {
    return Commands.run(() -> intakePivot.set(-IntakeConstants.kPivotManualSpeed), this)
        .finallyDo(() -> setPivotPosition(getPivotPosition())); // hold wherever it stopped
  }

  /**
   * Manually raises the intake pivot while the button is held (Y button).
   * Runs the pivot motor at +kPivotManualSpeed (upward) continuously.
   * On release: stops the motor and holds the current position via Motion Magic.
   * The TalonFX forward soft limit enforces the ceiling at kPivotHomePosition (0.0).
   * @return Command that raises pivot while held, holds on release
   */
  public Command raiseIntakeManual() {
    return Commands.run(() -> intakePivot.set(IntakeConstants.kPivotManualSpeed), this)
        .finallyDo(() -> setPivotPosition(getPivotPosition())); // hold wherever it stopped
  }

  /**
   * Command to stow the intake pivot back to the stowed position (0 rotations)
   * and stop the rollers. Designed for use with onFalse() after intakeDeployCollect().
   * @return Command that stows pivot and stops rollers
   */
  public Command stowAndStop() {
    return Commands.runOnce(() -> {
      setPivotPosition(IntakeConstants.kPivotStowedPosition);
      stopRollers();
    }, this);
  }

  @Override
  public void periodic() {
    // Throttle all NT/SmartDashboard updates to every 5 loops (~100ms) to reduce NT memory pressure
    periodicCounter++;
    if (periodicCounter < 5) return;
    periodicCounter = 0;

    // Update mechanism visualization at throttled rate (100ms) — avoids flooding NT queue
    m_mechanisms.update(intakePivot.getPosition(), intakePivot.getVelocity());
/*
    String prefix = IntakeConstants.kSmartDashboardPrefix;
    SmartDashboard.putNumber(prefix + IntakeConstants.kPivotPositionKey, getPivotPosition());
    SmartDashboard.putNumber(prefix + IntakeConstants.kPivotTargetKey, pivotTargetPosition);
    SmartDashboard.putNumber(prefix + IntakeConstants.kPivotErrorKey, getPivotError());
    SmartDashboard.putBoolean(prefix + IntakeConstants.kPivotAtTargetKey, isPivotAtTarget());
    SmartDashboard.putNumber(prefix + IntakeConstants.kRollersVelocityKey, getRollersVelocity());
    SmartDashboard.putNumber(prefix + IntakeConstants.kRollersCurrentKey, getRollersCurrent());
    SmartDashboard.putNumber(prefix + IntakeConstants.kPivotCurrentKey, getPivotCurrent());
    SmartDashboard.putNumber(prefix + IntakeConstants.kPivotTempKey, getPivotTemperature());
    SmartDashboard.putNumber(prefix + IntakeConstants.kRollersTempKey, getRollersTemperature());
    SmartDashboard.putString(prefix + IntakeConstants.kStatusKey,
        String.format("Pivot: %.2f | Rollers: %.1f RPS", getPivotPosition(), getRollersVelocity()));
  
  */
      }

}
