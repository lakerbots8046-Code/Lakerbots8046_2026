package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  /**
   * Adjustable pivot target used by intakeDeployCollect().
   * Modified by bumpPivotUp() / bumpPivotDown() WITHOUT requiring the subsystem,
   * so those commands never interrupt intakeDeployCollect() and the rollers keep spinning.
   */
  private double currentPivotTarget = IntakeConstants.kPivotDeployCollectPosition;
    
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
    cfgRollers.Voltage.withPeakForwardVoltage(Volts.of(12))
      .withPeakReverseVoltage(Volts.of(-12));
    //    cfgRollers.CurrentLimits.withStatorCurrentLimit(Amps.of(60)); // Limit current to 40 A to prevent breaker trips and motor damage

    /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    cfgRollers.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
    cfgRollers.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
    cfgRollers.Slot1.kI = 0; // No output for integrated error
    cfgRollers.Slot1.kD = 0; // No output for error derivative
    // Peak output of 40 A
    cfgRollers.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(60))
      .withPeakReverseTorqueCurrent(Amps.of(-60));
     /* Retry config apply up to 5 times, report if failure */
    
 
    /* ---------------------------------- INTAKE PIVOT : Motion Magic -> ----------------------------------*/
    // ============ CONFIGURE PIVOT POSITION CONTROL ============ //

    // Configure Gear Ratio using constant
    FeedbackConfigs fdb = cfgPivot.Feedback;
    fdb.SensorToMechanismRatio = IntakeConstants.kSensorToMechanismRatio; //12.8 rotor rotations per mechanism rotation
    cfgPivot.Feedback.SensorToMechanismRatio = IntakeConstants.kSensorToMechanismRatio;
    

    //Configure Motion Magic
    MotionMagicConfigs mm = cfgPivot.MotionMagic;
    mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(4.8))
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(4.8))
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0));
 

    Slot0Configs slot0 = cfgPivot.Slot0;
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0.kV = 13.0; // A velocity target of 1 rps results in 13 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 70; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output

    // Software limits prevent pivot from travelling outside safe range
    // Upper limit: kPivotHomePosition (0.0) — pivot cannot raise above home/stowed
    // Lower limit: kPivotDeployCollectPosition (-1.36) — pivot cannot lower past deploy position
    cfgPivot.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfgPivot.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.kPivotHomePosition; // hard stop at 0.0 (home)
    cfgPivot.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfgPivot.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.kPivotHardDownLimit; // physical hard stop at -1.4 (DO NOT EXCEED)


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
public Command AutoIntakeDeployCollect() {
    return Commands.runOnce(() -> rollersEnabled = true, this)
      .andThen(Commands.run(() -> {
        setPivotPosition(IntakeConstants.kPivotDeployCollectPosition);
        if (rollersEnabled) {
          //setIntakeRollersVoltage(IntakeConstants.kIntakeVoltage);
          setRollersVelocity(IntakeConstants.kRollersIntakeVelocity);
        } else {
          stopRollers();
        }
          rollersEnabled = true;
        }, this)).withTimeout(3);
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
    return Commands.runOnce(() -> {
        rollersEnabled = true;
        currentPivotTarget = IntakeConstants.kPivotDeployCollectPosition; // reset on each deploy
      }, this)
      .andThen(Commands.run(() -> {
        // Use currentPivotTarget so bumpPivotUp/Down can adjust it without interrupting this command
        setPivotPosition(currentPivotTarget);
        if (rollersEnabled) {
          setRollersVelocity(IntakeConstants.kRollersIntakeVelocity);
        } else {
          stopRollers();
        }
      }, this))
      .finallyDo((interrupted) -> {
        rollersEnabled = false;
        stopRollers();
        if (DriverStation.isDisabled()) {
          // Robot was disabled — hold wherever the pivot currently is.
          // If it's deployed, stay deployed. If it's stowed, stay stowed.
          // This prevents the intake from automatically retracting on re-enable.
          setPivotPosition(getPivotPosition());
        } else {
          // User intentionally cancelled (second button press while enabled) — retract to home.
          setPivotPosition(IntakeConstants.kPivotHomePosition);
        }
        currentPivotTarget = IntakeConstants.kPivotDeployCollectPosition; // reset for next deploy
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
   * Immediately stops the intake rollers and prevents {@link #intakeDeployCollect()}
   * from restarting them. Safe to call from outside a command (no subsystem requirement).
   *
   * <p>Call this when another mechanism (e.g. shoot-on-arc) needs the rollers off.
   * Pair with {@link #enableRollers()} to restore normal roller behaviour afterward.
   */
  public void stopRollersDirect() {
    rollersEnabled = false;
    intakeRollers.setControl(m_brake);
  }

  /**
   * Allows {@link #intakeDeployCollect()} to resume spinning the rollers on its next loop.
   * Has no effect if {@link #intakeDeployCollect()} is not currently running.
   *
   * <p>Call this when the mechanism that called {@link #stopRollersDirect()} has finished.
   */
  public void enableRollers() {
    rollersEnabled = true;
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
        .finallyDo((interrupted) -> setPivotPosition(getPivotPosition())); // hold wherever it stopped
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
        .finallyDo((interrupted) -> setPivotPosition(getPivotPosition())); // hold wherever it stopped
  }

  /**
   * Manually raises the intake pivot at a reduced speed while held.
   * Intended for operator B: move opposite deploy direction (upward) gently.
   * On release: holds current position via Motion Magic.
   *
   * @return Command that raises pivot slowly while held, then holds on release
   */
  public Command raiseIntakeManualLowSpeed() {
    // Use a stronger low-speed output so the pivot can reliably overcome gravity/static friction.
    // This is intentionally higher than 0.3 * kPivotManualSpeed based on on-robot feedback.
    return Commands.run(() -> intakePivot.set(IntakeConstants.kPivotManualSpeed * 1.8), this)
        .finallyDo((interrupted) -> setPivotPosition(getPivotPosition()));
  }

  /**
   * Resets intake pivot encoder position to the home reference.
   * This sets both the integrated sensor and command target to home (0 rotations).
   */
  public void resetPivotEncoderToHome() {
    intakePivot.setPosition(IntakeConstants.kPivotHomePosition);
    pivotTargetPosition = IntakeConstants.kPivotHomePosition;
    currentPivotTarget = IntakeConstants.kPivotDeployCollectPosition;
    setPivotPosition(IntakeConstants.kPivotHomePosition);
  }

  /**
   * Command wrapper for resetting intake pivot encoder to home.
   *
   * @return Command that resets pivot encoder reference
   */
  public Command resetPivotEncoderCommand() {
    return Commands.runOnce(this::resetPivotEncoderToHome, this);
  }

  /**
   * Bumps the intake pivot position up by kPivotBumpFactor.
   * Used with POV Up (0°) button to incrementally raise the intake.
   *
   * <p>Intentionally has NO subsystem requirement so it does NOT interrupt
   * {@link #intakeDeployCollect()}. It updates {@code currentPivotTarget}, which
   * intakeDeployCollect() reads every loop — the pivot moves to the new position
   * while the rollers keep spinning uninterrupted.
   *
   * @return Command that bumps pivot position upward without stopping rollers
   */
  public Command bumpPivotUp() {
    // InstantCommand with no requirements — will NOT interrupt intakeDeployCollect().
    // Updates currentPivotTarget; intakeDeployCollect() reads it every loop so the
    // pivot moves to the new position while the rollers keep spinning uninterrupted.
    return new InstantCommand(() -> {
      currentPivotTarget = Math.min(
          currentPivotTarget + IntakeConstants.kPivotBumpFactor,
          IntakeConstants.kPivotStowedPosition);
      // Also command the pivot directly in case intakeDeployCollect() is not running
      setPivotPosition(currentPivotTarget);
    }); // no subsystem requirement
  }

  /**
   * Bumps the intake pivot position down by kPivotBumpFactor.
   * Used with POV Down (180°) button to incrementally lower the intake.
   *
   * <p>Intentionally has NO subsystem requirement so it does NOT interrupt
   * {@link #intakeDeployCollect()}. It updates {@code currentPivotTarget}, which
   * intakeDeployCollect() reads every loop — the pivot moves to the new position
   * while the rollers keep spinning uninterrupted.
   *
   * @return Command that bumps pivot position downward without stopping rollers
   */
  public Command bumpPivotDown() {
    // InstantCommand with no requirements — will NOT interrupt intakeDeployCollect().
    return new InstantCommand(() -> {
      currentPivotTarget = Math.max(
          currentPivotTarget - IntakeConstants.kPivotBumpFactor,
          IntakeConstants.kPivotHardDownLimit); // never go past the physical hard stop (-1.4)
      // Also command the pivot directly in case intakeDeployCollect() is not running
      setPivotPosition(currentPivotTarget);
    }); // no subsystem requirement
  }

  /**
   * Lifts the intake pivot to the dump position (-0.75 rotations), holds it there
   * for 0.5 seconds, then returns to the deploy/collect position.
   *
   * <p>Intended to be bound to a button with {@code onTrue()} so it runs once per press.
   * Requires the intake subsystem — will interrupt {@link #intakeDeployCollect()} if running.
   * After this command finishes, re-press the deploy/collect button to resume normal intake.
   *
   * @return Command that performs the dump-and-return sequence
   */
  public Command dumpAndReturn() {
    return Commands.sequence(
        goToPivotPosition(-1),                                                                      // lift to dump position and wait until there // -0.75                        // run rollers for 0.5 seconds to dump game piece //0.25
        Commands.waitSeconds(0.25),                                                         // hold at dump position for 0.5 s //0.5
        Commands.runOnce(() -> setPivotPosition(IntakeConstants.kPivotDeployCollectPosition), this) // return to collect position
    );
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
