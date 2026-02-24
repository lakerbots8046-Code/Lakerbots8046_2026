// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.Constants.TurretConstants;

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
    private final MotionMagicVoltage m_mmreq = new MotionMagicVoltage(0);
    private final VelocityVoltage m_VelocityVoltage = new VelocityVoltage(0).withSlot(0);
    private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(0);
    private final NeutralOut m_brake = new NeutralOut();
    private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);

  // State tracking
    private double lastSpindexerVelocity = 0; 
    private double lastFlappyWheelVelocity = 0;
    private double lastFeederVelocity = 0;
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
    cfgSpindexer.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    cfgSpindexer.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    cfgSpindexer.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    cfgSpindexer.Slot0.kI = 0; // No output for integrated error
    cfgSpindexer.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    cfgSpindexer.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

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
    // Peak output of 8 volts
    cfgSpindexer.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

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
    cfgFeeder.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    cfgFeeder.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    cfgFeeder.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    cfgFeeder.Slot0.kI = 0; // No output for integrated error
    cfgFeeder.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    cfgFeeder.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

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
    spindexerMotor.setControl(m_VelocityVoltage.withVelocity(velocityRPS));
  }
  
  /**
   * Stops the spindexer motor
   */
  public void stopSpindexer() {
    spindexerMotor.setControl(m_brake);
    // this is not correct this sets the control mode to brake but we need to set the voltage or Speed to Zero
  }
  
  /**
   * Gets the current collection motor velocity
   * @return Velocity in rotations per second
   */
  public double getSpindexerVelocity() {
    return spindexerMotor.getVelocity().getValueAsDouble();
  }
//TO DO --- create open loop voltage contgrol methods and commands for initial testing
// - 
//public void setSpindexerVoltage(m_VelocityVoltage.w


  // ==================== COMMAND FACTORY METHODS ====================
  
  public Command setSpindexerVoltage(double speed){
    return Commands.run(() -> {
      spindexerMotor.set(speed);
    }, this);
  }
  
  /**
   * Command to run intake collection at intake speed
   * @return Command that runs collection motor
   */
  public Command runSpindexer() {
    return Commands.run(() -> {
      setSpindexerVelocity(SpindexerConstants.kFeederIntakeVelocity);
    }, this);
  }

  public Command stopSpindexerSpin() {
    return Commands.run(() -> {
      stopSpindexer();
    }, this);
  }
  
  /**
   * Command to run intake collection at outtake speed
   * @return Command that ejects game pieces
   */
  public Command runSpindexerOuttake() {
    return Commands.run(() -> {
      setSpindexerVelocity(SpindexerConstants.kFeederOuttakeVelocity);
    }, this);
  }
  
  // ==================== FLAPPY WHEEL FEEDER VELOCITY CONTROL METHODS ====================
  
  /**
   * Sets the collection motor velocity
   * @param velocityRPS Velocity in rotations per second
   */
  public void setFlappyWheelVelocity(double velocityRPS) {
    flappyWheelFeederMotor.setControl(m_VelocityVoltage.withVelocity(velocityRPS));
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

  // ==================== COMMAND FACTORY METHODS ====================
  
  public Command setFlappyWheelVoltage(double speed){
    return Commands.run(() -> {
      flappyWheelFeederMotor.set(speed);
    }, this);
  }

  /**
   * Command to run intake collection at intake speed
   * @return Command that runs collection motor
   */
  public Command runFlappyWheel() {
    return Commands.run(() -> {
      setFlappyWheelVelocity(SpindexerConstants.kFeederIntakeVelocity);
    }, this);
  }
  
  public Command stopFlappyWheelSpin() {
    return Commands.run(() -> {
      stopFlappyWheel();
    }, this);
  }

  /**
   * Command to run intake collection at outtake speed
   * @return Command that ejects game pieces
   */
  public Command runFlappyWheelOuttake() {
    return Commands.run(() -> {
      setFlappyWheelVelocity(SpindexerConstants.kFeederOuttakeVelocity);
    }, this);
  }

  // ==================== FEEDER VELOCITY CONTROL METHODS ====================
  
  /**
   * Sets the collection motor velocity
   * @param velocityRPS Velocity in rotations per second
   */
  public void setFeederVelocity(double velocityRPS) {
    feederMotor.setControl(m_VelocityVoltage.withVelocity(velocityRPS));
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

  // ==================== COMMAND FACTORY METHODS ====================
  
  public Command setFeederVoltage(double speed){
    return Commands.run(() -> {
      feederMotor.set(speed);
    }, this);
  }

  /**
   * Command to run intake collection at intake speed
   * @return Command that runs collection motor
   */
  public Command runFeeder() {
    return Commands.run(() -> {
      setFeederVelocity(SpindexerConstants.kFeederIntakeVelocity);
    }, this);
  }

  public Command stopFeederSpin() {
    return Commands.run(() -> {
      stopFeeder();
    }, this);
  }
  
  /**
   * Command to run intake collection at outtake speed
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
   * Directly runs all three feed-side motors at the specified duty cycles.
   *
   * <p>Intended for use inside another command's {@code execute()} method where
   * returning a {@link Command} object is not possible.  Uses the same duty-cycle
   * output mode as {@link #launchFromTower()}.
   *
   * @param spindexerDC    Duty cycle for the spindexer motor   (CAN 4), range [-1, 1]
   * @param flappyWheelDC  Duty cycle for the flappy-wheel motor (CAN 5), range [-1, 1]
   * @param feederDC       Duty cycle for the feeder motor       (CAN 6), range [-1, 1]
   */
  public void runFeedMotorsDirect(double spindexerDC, double flappyWheelDC, double feederDC) {
    spindexerMotor.set(spindexerDC);
    flappyWheelFeederMotor.set(flappyWheelDC);
    feederMotor.set(feederDC);
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
   * Runs all three Spindexer-side motors at fixed voltages for tower shooting.
   *   - Spindexer motor (CAN 4):        -0.2 V  (temp test value)
   *   - StarFeeder / FlappyWheel (CAN 5): +0.2 V  (temp test value)
   *   - Feeder motor (CAN 6):            +0.2 V  (temp test value)
   *
   * Designed to be used in parallel with Launcher.launchFromTowerLauncher().
   * Runs while the button is held; all motors stop when the command ends.
   *
   * @return Command that drives the three feed-side motors for tower shooting
   */
  public Command launchFromTower() {
    return Commands.run(() -> {
      // Spindexer: TurretConstants.spindexerDutyCycleOut (temp: -0.2 | final: -1.0)
      spindexerMotor.setControl(m_dutyCycleOut.withOutput(TurretConstants.spindexerDutyCycleOut));
      // StarFeeder (FlappyWheel): TurretConstants.starFeederDutyCycleOut (temp: +0.2 | final: +1.0)
      flappyWheelFeederMotor.setControl(m_dutyCycleOut.withOutput(TurretConstants.starFeederDutyCycleOut));
      // Feeder: TurretConstants.feederDutyCycleOut (temp: +0.2 | final: +0.5)
      feederMotor.setControl(m_dutyCycleOut.withOutput(TurretConstants.feederDutyCycleOut));
    }, this).finallyDo(() -> {
      // Stop all three motors when command ends (button released)
      spindexerMotor.setControl(m_brake);
      flappyWheelFeederMotor.setControl(m_brake);
      feederMotor.setControl(m_brake);
    });
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
  /*public Command goToPivotPosition(double position) {
    return Commands.runOnce(() -> {
      setHoodPosition(position);
    }, this).andThen(Commands.waitUntil(this::isHoodAtTarget));
  }*/

  @Override
  public void periodic() {
    // Throttle SmartDashboard updates to every 5 loops (~100ms) to reduce NT memory pressure
    periodicCounter++;
    if (periodicCounter < 5) return;
    periodicCounter = 0;

    /*
    String prefix = SpindexerConstants.kSmartDashboardPrefix;
    SmartDashboard.putNumber(prefix + SpindexerConstants.kSpindexerVelocityKey, getSpindexerVelocity());
    SmartDashboard.putNumber(prefix + SpindexerConstants.kSpindexerCurrentKey, getSpindexerCurrent());
    SmartDashboard.putNumber(prefix + SpindexerConstants.kFlappyWheelVelocityKey, getFlappyWheelVelocity());
    SmartDashboard.putNumber(prefix + SpindexerConstants.kFlappyWheelCurrentKey, getFlappyWheelCurrent());
    SmartDashboard.putNumber(prefix + SpindexerConstants.kFeederVelocityKey, getFeederVelocity());
    SmartDashboard.putNumber(prefix + SpindexerConstants.kFeederCurrentKey, getFeederCurrent());
    SmartDashboard.putNumber(prefix + SpindexerConstants.kFeederTempKey, getFeederTemperature());
    SmartDashboard.putNumber(prefix + SpindexerConstants.kFlappyWheelTempKey, getFlappyWheelTemperature());
    SmartDashboard.putNumber(prefix + SpindexerConstants.kSpindexerTempKey, getSpindexerTemperature());
    SmartDashboard.putString(prefix + SpindexerConstants.kStatusKey,
        String.format("Spindexer: %.2f | FlappyWheel: %.1f | Feeder: %.1f RPS",
            getSpindexerVelocity(), getFlappyWheelVelocity(), getFeederVelocity()));
  
  */
          }

}

