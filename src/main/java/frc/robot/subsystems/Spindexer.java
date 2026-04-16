// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;

/**
 * Subsystem for the Launcher mechanism, which includes:
 * - Spindexer motor for feeding game pieces (Velocity control)
 * - FlappyWheelFeeder motor for additional feeding control (Velocity control)
 * - Feeder motor for final stage of feeding into the launcher (Velocity control)
 */


public class Spindexer extends SubsystemBase {
  //Motors
    private final TalonFX spindexerMotor; 
    private final TalonFX flappyWheelFeederMotor;
    private final TalonFX feederMotor; 
    
  //Controls
    // Separate VelocityVoltage objects per motor — sharing one object between motors causes
    // the last withVelocity() call to overwrite the previous one in the same loop iteration.
    private final VelocityVoltage m_spindexerVelocity    = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage m_flappyWheelVelocity  = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage m_feederVelocity        = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut m_brake = new NeutralOut();

  // State tracking
    private int periodicCounter = 0; // Throttle SmartDashboard updates to reduce NT load
  
  public Spindexer() {
    // Initialize Motors using constants
    spindexerMotor = new TalonFX(SpindexerConstants.kSpindexerMotorID);
    flappyWheelFeederMotor = new TalonFX(SpindexerConstants.kFlappyWheelFeederMotorID);
    feederMotor = new TalonFX(SpindexerConstants.kFeederMotorID);
    
    //Config
    TalonFXConfiguration cfgSpindexer = new TalonFXConfiguration();
    TalonFXConfiguration cfgFlappyWheel = new TalonFXConfiguration();
    TalonFXConfiguration cfgFeeder = new TalonFXConfiguration();

    /* ------------------------------SPINDEXER/FLAPPYWHEEL/FEEDER : VelocityClosedLoop -> --------------------------------- */
    //======== SPINDEXER CONFIG ========// 
    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    // kS increased: better overcomes sudden load friction when a ball enters the spindexer.
    cfgSpindexer.Slot0.kS = 0.25; // Increased from 0.1 — matches feeder, overcomes ball-load friction
    cfgSpindexer.Slot0.kV = 0.12; // Kraken X60: 1/8.333 rps per V = 0.12 V/RPS (unchanged)
    // kP increased: at 0.11 a 20 RPS drop only added 2.2 V correction.
    // At 0.5, a 20 RPS drop adds 10 V correction — pushes motor to near-full voltage immediately.
    cfgSpindexer.Slot0.kP = 0.5;  // Increased from 0.11 — fast recovery under ball load
    // kI added: eliminates steady-state velocity droop under sustained ball load.
    cfgSpindexer.Slot0.kI = 0.02; // Added — eliminates steady-state error under sustained load
    cfgSpindexer.Slot0.kD = 0;    // No derivative — load changes are gradual, D not needed
    // Peak output raised to 12 V — at 90 RPS the feedforward alone needs ~10.9 V.
    // The previous 8 V cap physically prevented the motor from exceeding ~64 RPS.
    cfgSpindexer.Voltage.withPeakForwardVoltage(Volts.of(12))
      .withPeakReverseVoltage(Volts.of(-12));

    /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    cfgSpindexer.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
    cfgSpindexer.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
    cfgSpindexer.Slot1.kI = 0; // No output for integrated error
    cfgSpindexer.Slot1.kD = 0; // No output for error derivative
    // Peak output of 40 A
    cfgSpindexer.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
      .withPeakReverseTorqueCurrent(Amps.of(-40));
     /* Retry config apply up to 5 times, report if failure */

    // ========= FLAPPY WHEEL CONFIG ========== //
    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    cfgFlappyWheel.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    cfgFlappyWheel.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    cfgFlappyWheel.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    cfgFlappyWheel.Slot0.kI = 0; // No output for integrated error
    cfgFlappyWheel.Slot0.kD = 0; // No output for error derivative
    // Peak output raised to 12 V (was incorrectly setting cfgSpindexer.Voltage — bug fixed).
    cfgFlappyWheel.Voltage.withPeakForwardVoltage(Volts.of(12))
      .withPeakReverseVoltage(Volts.of(-12));

    /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    cfgFlappyWheel.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
    cfgFlappyWheel.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
    cfgFlappyWheel.Slot1.kI = 0; // No output for integrated error
    cfgFlappyWheel.Slot1.kD = 0; // No output for error derivative
    // Peak output of 40 A
    cfgFlappyWheel.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
      .withPeakReverseTorqueCurrent(Amps.of(-40));
     /* Retry config apply up to 5 times, report if failure */

    // ========= FEEDER CONFIG ========== //
    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    // kS increased further: feeder sees the highest load of all three motors.
    cfgFeeder.Slot0.kS = 0.35; // Increased from 0.25 — better overcomes peak ball-load friction
    cfgFeeder.Slot0.kV = 0.12; // Kraken X60: 1/8.333 rps per V = 0.12 V/RPS (unchanged)
    // kP doubled: with 12 V peak, kP=1.0 pushes to full voltage for any drop >1.1 RPS.
    // This is the maximum useful kP before the motor is always at voltage saturation.
    cfgFeeder.Slot0.kP = 1.0;  // Increased from 0.5 — maximum-speed recovery under ball load
    // kI increased slightly: faster elimination of steady-state droop under sustained load.
    cfgFeeder.Slot0.kI = 0.05; // Increased from 0.02 — faster steady-state error correction
    cfgFeeder.Slot0.kD = 0;    // No derivative — feeder load changes are gradual, D not needed
    // Peak output raised to 12 V — feeder needs ~11.05 V feedforward at 90 RPS.
    // The previous 8 V cap was the reason the feeder stalled at ~60 RPS under load.
    cfgFeeder.Voltage.withPeakForwardVoltage(Volts.of(12))
      .withPeakReverseVoltage(Volts.of(-12));

    /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    cfgFeeder.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
    cfgFeeder.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
    cfgFeeder.Slot1.kI = 0; // No output for integrated error
    cfgFeeder.Slot1.kD = 0; // No output for error derivative
    // Peak output of 40 A
    cfgFeeder.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
      .withPeakReverseTorqueCurrent(Amps.of(-40));
     /* Retry config apply up to 5 times, report if failure */


     // Status Code for ALL
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = spindexerMotor.getConfigurator().apply(cfgSpindexer);
      status = flappyWheelFeederMotor.getConfigurator().apply(cfgFlappyWheel);
      status = feederMotor.getConfigurator().apply(cfgFeeder);
      if (status.isOK()) break;
    }
  }


  // ==================== SPINDEXER VELOCITY CONTROL METHODS ====================
  
  /**
   * Sets the collection motor velocity
   * @param velocityRPS Velocity in rotations per second
   */
  public void setSpindexerVelocity(double velocityRPS) {
    spindexerMotor.setControl(m_spindexerVelocity.withVelocity(velocityRPS));
  }
  
  /**
   * Stops the spindexer motor
   */
  public void stopSpindexer() {
    spindexerMotor.setControl(m_brake);
  }
  
  /**
   * Gets the current collection motor velocity
   * @return Velocity in rotations per second
   */
  public double getSpindexerVelocity() {
    return spindexerMotor.getVelocity().getValueAsDouble();
  }
  // ==================== COMMAND FACTORY METHODS ====================

  public Command setSpindexerVoltage(double speed){
    return Commands.run(() -> {
      spindexerMotor.set(speed);
    }, this);
  }
  
  /**
   * Command to run spindexer at intake velocity (velocity closed-loop).
   * @return Command that runs spindexer at kSpindexerIntakeVelocity RPS
   */
  public Command runSpindexer() {
    return Commands.run(() -> {
      setSpindexerVelocity(SpindexerConstants.kSpindexerIntakeVelocity);
    }, this);
  }

  public Command stopSpindexerSpin() {
    return Commands.runOnce(this::stopSpindexer, this);
  }
  
  /**
   * Command to run spindexer at outtake velocity (velocity closed-loop).
   * @return Command that ejects game pieces
   */
  public Command runSpindexerOuttake() {
    return Commands.run(() -> setSpindexerVelocity(60), this)
        .finallyDo((interrupted) -> setSpindexerVelocity(0));
  }
  
  // ==================== FLAPPY WHEEL FEEDER VELOCITY CONTROL METHODS ====================
  
  /**
   * Sets the collection motor velocity
   * @param velocityRPS Velocity in rotations per second
   */
  public void setFlappyWheelVelocity(double velocityRPS) {
    flappyWheelFeederMotor.setControl(m_flappyWheelVelocity.withVelocity(velocityRPS));
  }
  
  /**
   * Stops the spindexer motor
   */
  public void stopFlappyWheel() {
    flappyWheelFeederMotor.setControl(m_brake);
  }
  
  /**
   * Gets the current collection motor velocity
   * @return Velocity in rotations per second
   */
  public double getFlappyWheelVelocity() {
    return flappyWheelFeederMotor.getVelocity().getValueAsDouble();
  }

  public Command setFlappyWheelVoltage(double speed){
    return Commands.run(() -> {
      flappyWheelFeederMotor.set(speed);
    }, this);
  }



  /**
   * Command to run flappy wheel (Stars) at intake velocity (velocity closed-loop).
   * @return Command that runs flappy wheel at kFlappyWheelIntakeVelocity RPS
   */
  public Command runFlappyWheel() {
    return Commands.run(() -> {
      setFlappyWheelVelocity(SpindexerConstants.kFlappyWheelIntakeVelocity);
    }, this);
  }
  
  public Command stopFlappyWheelSpin() {
    return Commands.runOnce(this::stopFlappyWheel, this);
  }

  /**
   * Command to run flappy wheel at outtake velocity (velocity closed-loop).
   * @return Command that ejects game pieces
   */
  public Command runFlappyWheelOuttake() {
    return Commands.run(() -> {
      setFlappyWheelVelocity(SpindexerConstants.kFlappyWheelOuttakeVelocity);
    }, this);
  }

  // ==================== FEEDER VELOCITY CONTROL METHODS ====================
  
  /**
   * Sets the collection motor velocity
   * @param velocityRPS Velocity in rotations per second
   */
  public void setFeederVelocity(double velocityRPS) {
    feederMotor.setControl(m_feederVelocity.withVelocity(velocityRPS));
  }
  
  /**
   * Stops the spindexer motor
   */
  public void stopFeeder() {
    feederMotor.setControl(m_brake);
  }
  
  /**
   * Gets the current collection motor velocity
   * @return Velocity in rotations per second
   */
  public double getFeederVelocity() {
    return feederMotor.getVelocity().getValueAsDouble();
  }

  public Command setFeederVoltage(double speed){
    return Commands.run(() -> {
      feederMotor.set(speed);
    }, this);
  }

  /**
   * Command to run feeder at intake velocity (velocity closed-loop).
   * @return Command that runs feeder at kFeederIntakeVelocity RPS
   */
  public Command runFeeder() {
    return Commands.run(() -> {
      setFeederVelocity(SpindexerConstants.kFeederIntakeVelocity);
    }, this);
  }

  public Command stopFeederSpin() {
    return Commands.runOnce(this::stopFeeder, this);
  }
  
  /**
   * Command to run feeder at outtake velocity (velocity closed-loop).
   * @return Command that ejects game pieces
   */
  public Command runFeederOuttake() {
    return Commands.run(() -> {
      setFeederVelocity(SpindexerConstants.kFeederOuttakeVelocity);
    }, this);
  }

  // ==================== MONITORING METHODS ====================
  
  /**
   * Gets the launcher motor current draw
   * @return Current in amps
   */
  public double getFeederCurrent() {
    return feederMotor.getSupplyCurrent().getValueAsDouble();
  }

   /**
   * Gets the launcher motor current draw
   * @return Current in amps
   */
  public double getFlappyWheelCurrent() {
    return flappyWheelFeederMotor.getSupplyCurrent().getValueAsDouble();
  }
  
  /**
   * Gets the collection motor current draw
   * @return Current in amps
   */
  public double getSpindexerCurrent() {
    return spindexerMotor.getSupplyCurrent().getValueAsDouble();
  }
  
  /**
   * Gets the feeder motor temperature
   * @return Temperature in Celsius
   */
  public double getFeederTemperature() {
    return feederMotor.getDeviceTemp().getValueAsDouble();
  }

  /**
   * Gets the feeder motor temperature
   * @return Temperature in Celsius
   */
  public double getFlappyWheelTemperature() {
    return flappyWheelFeederMotor.getDeviceTemp().getValueAsDouble();
  }
  
  /**
   * Gets the spindexer motor temperature
   * @return Temperature in Celsius
   */
  public double getSpindexerTemperature() {
    return spindexerMotor.getDeviceTemp().getValueAsDouble();
  }

  

  // ==================== DIRECT MOTOR CONTROL (for use inside other commands) ====================

  /**
   * Directly runs all three feed-side motors at the specified velocities (RPS).
   *
   * <p>Uses VelocityVoltage closed-loop control for each motor independently.
   * Intended for use inside another command's {@code execute()} method where
   * returning a {@link Command} object is not possible.
   *
   * @param spindexerRPS    Target velocity for the spindexer motor    (CAN 4), RPS
   * @param flappyWheelRPS  Target velocity for the flappy-wheel motor (CAN 5), RPS
   * @param feederRPS       Target velocity for the feeder motor        (CAN 6), RPS
   */
  public void runFeedMotorsDirect(double spindexerRPS, double flappyWheelRPS, double feederRPS) {
    setSpindexerVelocity(spindexerRPS);
    setFlappyWheelVelocity(flappyWheelRPS);
    setFeederVelocity(feederRPS);
  }

  /**
   * Directly stops all three feed-side motors (coast/brake).
   *
   * <p>Intended for use inside another command's {@code execute()} or {@code end()} method.
   */
  public void stopFeedMotorsDirect() {
    spindexerMotor.setControl(m_brake);
    flappyWheelFeederMotor.setControl(m_brake);
    feederMotor.setControl(m_brake);
  }

  // ==================== LAUNCH FROM TOWER COMMAND ====================

  /**
   * LaunchFromTower shoot command.
   * Runs all three Spindexer-side motors at fixed RPS using VelocityVoltage closed-loop control.
   *   - Spindexer motor    (CAN 4): {@link SpindexerConstants#kSpindexerIntakeVelocity} RPS
   *   - FlappyWheel (Stars)(CAN 5): {@link SpindexerConstants#kFlappyWheelIntakeVelocity} RPS
   *   - Feeder motor       (CAN 6): {@link SpindexerConstants#kFeederIntakeVelocity} RPS
   *
   * Designed to be used in parallel with Launcher.launchFromTowerLauncher().
   * Runs while the button is held; all motors stop when the command ends.
   *
   * @return Command that drives the three feed-side motors for tower shooting
   */
  public Command launchFromTower() {
    return Commands.run(() -> {
      // All three motors use independent VelocityVoltage closed-loop control.
      // Tune each constant independently in SpindexerConstants.
      setSpindexerVelocity(SpindexerConstants.kSpindexerIntakeVelocity);
      setFlappyWheelVelocity(SpindexerConstants.kFlappyWheelIntakeVelocity);
      setFeederVelocity(SpindexerConstants.kFeederIntakeVelocity);
    }, this).finallyDo(() -> {
      // Stop all three motors when command ends (button released)
      spindexerMotor.setControl(m_brake);
      flappyWheelFeederMotor.setControl(m_brake);
      feederMotor.setControl(m_brake);
    });
  }


  @Override
  public void periodic() {
    // Throttle periodic work to every ~100ms.
    periodicCounter++;
    if (periodicCounter < 5) return;
    periodicCounter = 0;
  }

}

