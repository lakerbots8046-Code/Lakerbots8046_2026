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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Mechanisms;

/**
 * Intake subsystem with dual motors:
 * - intakeCollect (velocity control) for spinning intake rollers
 * - intakePivot (position control) for pivoting intake mechanism
 */
public class Intake extends SubsystemBase {
  // Motors
  private final TalonFX intakeCollect;
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
    
  public Intake() {
    // Initialize motors using constants
    intakeCollect = new TalonFX(IntakeConstants.kIntakeCollectMotorID, IntakeConstants.kCANBusName);
    intakePivot = new TalonFX(IntakeConstants.kIntakePivotMotorID, IntakeConstants.kCANBusName);
    
    // Config
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    /*  ----------------------INTAKE COLLECT : VelocityClosedLoop ->------------------------------------ */
    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    cfg.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    cfg.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    cfg.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    cfg.Slot0.kI = 0; // No output for integrated error
    cfg.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    cfg.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    cfg.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
    cfg.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
    cfg.Slot1.kI = 0; // No output for integrated error
    cfg.Slot1.kD = 0; // No output for error derivative
    // Peak output of 40 A
    cfg.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
      .withPeakReverseTorqueCurrent(Amps.of(-40));
     /* Retry config apply up to 5 times, report if failure */
    
    
    /* ---------------------------------- INTAKE PIVOT : Motion Magic -> ----------------------------------*/

    // Configure Gear Ratio using constant
    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = IntakeConstants.kSensorToMechanismRatio; //12.8 rotor rotations per mechanism rotation
    cfg.Feedback.SensorToMechanismRatio = IntakeConstants.kSensorToMechanismRatio;
    

    //Configure Motion Magic
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(15))
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(15))
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
 

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output


    //Status code for all
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = intakePivot.getConfigurator().apply(cfg);
      status = intakeCollect.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
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
  public void setCollectVelocity(double velocityRPS) {
    intakeCollect.setControl(m_VelocityVoltage.withVelocity(velocityRPS));
  }
  
  /**
   * Stops the collection motor
   */
  public void stopCollect() {
    intakeCollect.setControl(m_brake);
  }
  
  /**
   * Gets the current collection motor velocity
   * @return Velocity in rotations per second
   */
  public double getCollectVelocity() {
    return intakeCollect.getVelocity().getValueAsDouble();
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
  public double getCollectCurrent() {
    return intakeCollect.getSupplyCurrent().getValueAsDouble();
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
  public double getCollectTemperature() {
    return intakeCollect.getDeviceTemp().getValueAsDouble();
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
  
  /**
   * Command to run intake collection at intake speed
   * @return Command that runs collection motor
   */
  public Command runIntake() {
    return Commands.run(() -> {
      setCollectVelocity(IntakeConstants.kCollectIntakeVelocity);
    }, this);
  }
  
  /**
   * Command to run intake collection at outtake speed
   * @return Command that ejects game pieces
   */
  public Command runOuttake() {
    return Commands.run(() -> {
      setCollectVelocity(IntakeConstants.kCollectOuttakeVelocity);
    }, this);
  }
  
  /**
   * Command to hold game piece with low speed
   * @return Command that holds game piece
   */
  public Command holdGamePiece() {
    return Commands.run(() -> {
      setCollectVelocity(IntakeConstants.kCollectHoldVelocity);
    }, this);
  }
  
  /**
   * Command to stop collection motor
   * @return Command that stops collection
   */
  public Command stopCollection() {
    return Commands.runOnce(() -> {
      stopCollect();
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

  @Override
  public void periodic() {
    // Update SmartDashboard with telemetry
    String prefix = IntakeConstants.kSmartDashboardPrefix;
    
    SmartDashboard.putNumber(prefix + IntakeConstants.kPivotPositionKey, getPivotPosition());
    SmartDashboard.putNumber(prefix + IntakeConstants.kPivotTargetKey, pivotTargetPosition);
    SmartDashboard.putNumber(prefix + IntakeConstants.kPivotErrorKey, getPivotError());
    SmartDashboard.putBoolean(prefix + IntakeConstants.kPivotAtTargetKey, isPivotAtTarget());
    
    SmartDashboard.putNumber(prefix + IntakeConstants.kCollectVelocityKey, getCollectVelocity());
    SmartDashboard.putNumber(prefix + IntakeConstants.kCollectCurrentKey, getCollectCurrent());
    SmartDashboard.putNumber(prefix + IntakeConstants.kPivotCurrentKey, getPivotCurrent());
    
    SmartDashboard.putNumber(prefix + IntakeConstants.kPivotTempKey, getPivotTemperature());
    SmartDashboard.putNumber(prefix + IntakeConstants.kCollectTempKey, getCollectTemperature());
    
    // Status string
    String status = String.format("Pivot: %.2f | Collect: %.1f RPS", getPivotPosition(), getCollectVelocity());
    SmartDashboard.putString(prefix + IntakeConstants.kStatusKey, status);
    
    // Update mechanism visualization
    m_mechanisms.update(intakePivot.getPosition(), intakePivot.getVelocity());
  }

}
