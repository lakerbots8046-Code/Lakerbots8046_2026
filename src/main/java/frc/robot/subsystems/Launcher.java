// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.Constants.LauncherConstants;
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
    private final MotionMagicVoltage m_mmreq = new MotionMagicVoltage(0);
    private final VelocityVoltage m_VelocityVoltage = new VelocityVoltage(0).withSlot(0);
    private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(0);
    private final NeutralOut m_brake = new NeutralOut();

  //Mechanisms visualization  
    private final Mechanisms m_mechanisms = new Mechanisms();

  // State tracking
    private double lastHoodAngle = 0;
    private double lastLauncherVelocity = 0;  
    private double lastTurretAngle = 0;
  
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
    cfgLaunch.Slot0.kS = 0; // To account for friction, add 0.1 V of static feedforward
    cfgLaunch.Slot0.kV = 0.0085; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    cfgLaunch.Slot0.kP = 0.01; // An error of 1 rotation per second results in 0.11 V output
    cfgLaunch.Slot0.kI = 0; // No output for integrated error
    cfgLaunch.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    cfgLaunch.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    cfgLaunch.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
    cfgLaunch.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
    cfgLaunch.Slot1.kI = 0; // No output for integrated error
    cfgLaunch.Slot1.kD = 0; // No output for error derivative
    // Peak output of 40 A
    cfgLaunch.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
      .withPeakReverseTorqueCurrent(Amps.of(-40));
     /* Retry config apply up to 5 times, report if failure */

     /* -------------------------------------- TURRET & HOOD : Motion Magic -> ----------------------------------*/
    // ============ CONFIGURE TURRET POSITION CONTROL ============ //
    // Configure Gear Ratio using constant
    FeedbackConfigs fdbTurret = cfgTurret.Feedback;
    fdbTurret.SensorToMechanismRatio = 1; //12.8 rotor rotations per mechanism rotation
    fdbTurret.SensorToMechanismRatio = LauncherConstants.kSensorToMechanismRatio; //12.8 rotor rotations per mechanism rotation
    cfgTurret.Feedback.SensorToMechanismRatio = LauncherConstants.kSensorToMechanismRatio;

    //Configure Motion Magic
    MotionMagicConfigs mmTurret = cfgTurret.MotionMagic;
    mmTurret.withMotionMagicCruiseVelocity(RotationsPerSecond.of(15))
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(15))
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
 

    Slot0Configs slot0Turret = cfgTurret.Slot0;
    slot0Turret.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Turret.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Turret.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Turret.kP = 60; // A position error of 0.2 rotations results in 12 V output
    slot0Turret.kI = 0; // No output for integrated error
    slot0Turret.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

    // ============ CONFIGURE HOOD POSITION CONTROL ============ //
    // Configure Gear Ratio using constant
    FeedbackConfigs fdbHood = cfgHood.Feedback;
    fdbHood.SensorToMechanismRatio = 1; //12.8 rotor rotations per mechanism rotation
    fdbHood.SensorToMechanismRatio = LauncherConstants.kSensorToMechanismRatio; //12.8 rotor rotations per mechanism rotation
    cfgTurret.Feedback.SensorToMechanismRatio = LauncherConstants.kSensorToMechanismRatio;

    //Configure Motion Magic
    MotionMagicConfigs mmHood = cfgHood.MotionMagic;
    mmHood.withMotionMagicCruiseVelocity(RotationsPerSecond.of(15))
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(15))
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
 

    Slot0Configs slot0Hood = cfgHood.Slot0;
    slot0Hood.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Hood.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Hood.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Hood.kP = 60; // A position error of 0.2 rotations results in 12 V output
    slot0Hood.kI = 0; // No output for integrated error
    slot0Hood.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output


    // Status Code for ALL
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = turretMotor.getConfigurator().apply(cfgTurret);
      status = hoodMotor.getConfigurator().apply(cfgHood);
      status = launcherMotor.getConfigurator().apply(cfgLaunch);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }

   // ========================= TURRET POSITION CONTROL METHODS ========================= //
   
  /**
   * Sets the pivot to a specific position in rotations
   * @param mechRotations Target position in mechanism rotations
   */
  public void setTurretPosition(double mechRotations) {
    lastTurretAngle = mechRotations;
    turretMotor.setControl(m_mmreq.withPosition(mechRotations).withSlot(0));
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
    hoodMotor.setControl(m_mmreq.withPosition(mechRotations).withSlot(0));
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
   * Checks if pivot is at the target position within tolerance
   * @return True if at target position
   */
  public boolean isHoodAtTarget() {
    return Math.abs(getHoodError()) < LauncherConstants.kHoodPositionTolerance;
  }

  // ==================== COLLECTION VELOCITY CONTROL METHODS ====================
  
  /**
   * Sets the collection motor velocity
   * @param velocityRPS Velocity in rotations per second
   */
  public void setCollectVelocity(double velocityRPS) {
    launcherMotor.setControl(m_VelocityVoltage.withVelocity(velocityRPS));
  }
  
  /**
   * Stops the collection motor
   */
  public void stopLauncher() {
    launcherMotor.setControl(m_brake);
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
  
  /**
   * Command to move pivot to stowed position
   * @return Command that stows the intake
   */
  public Command stowIntake() {
    return Commands.runOnce(() -> {
      setHoodPosition(LauncherConstants.kHoodStowedPosition);
    }, this);
  }
  
  /**
   * Command to move pivot to collect position
   * @return Command that extends intake for collecting
   */
  public Command extendForCollect() {
    return Commands.runOnce(() -> {
      setHoodPosition(LauncherConstants.kHoodCollectPosition);
    }, this);
  }
  
  /**
   * Command to move pivot to high scoring position
   * @return Command that positions intake for high scoring
   */
  public Command positionForScoreHigh() {
    return Commands.runOnce(() -> {
      setHoodPosition(LauncherConstants.kHoodScoreHighPosition);
    }, this);
  }
  
  /**
   * Command to move pivot to low scoring position
   * @return Command that positions intake for low scoring
   */
  public Command positionForScoreLow() {
    return Commands.runOnce(() -> {
      setHoodPosition(LauncherConstants.kHoodScoreLowPosition);
    }, this);
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
    return Commands.run(() -> {
      stopLauncher();
    }, this);
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
 
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
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
    // Update SmartDashboard with telemetry
    String prefix = LauncherConstants.kSmartDashboardPrefix;
    
    SmartDashboard.putNumber(prefix + LauncherConstants.kTurretPositionKey, getTurretPosition());
    SmartDashboard.putNumber(prefix + LauncherConstants.kTurretTargetKey, lastTurretAngle);
    SmartDashboard.putNumber(prefix + LauncherConstants.kTurretErrorKey, getTurretError());
    SmartDashboard.putBoolean(prefix + LauncherConstants.kTurretAtTargetKey, isTurretAtTarget());
    
    SmartDashboard.putNumber(prefix + LauncherConstants.kHoodPositionKey, getHoodPosition());
    SmartDashboard.putNumber(prefix + LauncherConstants.kHoodTargetKey, lastHoodAngle);
    SmartDashboard.putNumber(prefix + LauncherConstants.kHoodErrorKey, getHoodError());
    SmartDashboard.putBoolean(prefix + LauncherConstants.kHoodAtTargetKey, isHoodAtTarget());
    
    SmartDashboard.putNumber(prefix + LauncherConstants.kCollectVelocityKey, getCollectVelocity());
    SmartDashboard.putNumber(prefix + LauncherConstants.kCollectCurrentKey, getCollectCurrent());
    SmartDashboard.putNumber(prefix + LauncherConstants.kPivotCurrentKey, getHoodCurrent());
    
    SmartDashboard.putNumber(prefix + LauncherConstants.kTurretTempKey, getTurretTemperature());
    SmartDashboard.putNumber(prefix + LauncherConstants.kHoodTempKey, getHoodTemperature());
    SmartDashboard.putNumber(prefix + LauncherConstants.kCollectTempKey, getCollectTemperature());
    
    // Status string
    String status = String.format("Hood: %.2f | Turret: %.2f | Launcher: %.1f RPS", getHoodPosition(), getTurretPosition(), getCollectVelocity());
    SmartDashboard.putString(prefix + LauncherConstants.kStatusKey, status);
    
    // Update mechanism visualization
    m_mechanisms.update(hoodMotor.getPosition(), hoodMotor.getVelocity());
    m_mechanisms.update(turretMotor.getPosition(), turretMotor.getVelocity());
  }
}