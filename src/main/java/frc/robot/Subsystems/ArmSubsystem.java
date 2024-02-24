// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;

// Static imports for units
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

public class ArmSubsystem extends SubsystemBase {
  // Initializing the TalonSRX motorcontrollers
  private final WPI_TalonSRX leftMotor = new WPI_TalonSRX(ArmConstants.leftID);
  private final WPI_TalonSRX rightMotor = new WPI_TalonSRX(ArmConstants.rightID);

  // Initializing the limit switches
  private final DigitalInput dropSwitch = new DigitalInput(ArmConstants.dropLimitSwitchChannel);
  private final DigitalInput raiseSwitch = new DigitalInput(ArmConstants.raiseLimitSwitchChannel);

  // Mutable holders
  private final MutableMeasure<Voltage> m_appliedVoltage = MutableMeasure.mutable(Volts.of(0));
  private final MutableMeasure<Angle> m_angle = MutableMeasure.mutable(Degrees.of(0));
  private final MutableMeasure<Velocity<Angle>> m_anglePerSecond = MutableMeasure.mutable(DegreesPerSecond.of(0));

  // System Identification
  private final SysIdRoutine m_SysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
    (Measure<Voltage> voltage) -> {
      leftMotor.setVoltage(voltage.in(Volts));
    },
    log -> {
      // We are treating leftMotor and rightMotor as the same motor because rightMotor follows leftMotor?
      log.motor("arm-motor").voltage(m_appliedVoltage.mut_replace(leftMotor.get() * RobotController.getBatteryVoltage(), Volts))
        .angularPosition(m_angle.mut_replace(getAngle(), Degrees))
        .angularVelocity(m_anglePerSecond.mut_replace(getVelocity(), DegreesPerSecond));
    }, this
  ));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_SysIdRoutine.quasistatic(direction);
  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_SysIdRoutine.dynamic(direction);
  }

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // Reseting the motors to factory default
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    // Setting follower motor
    rightMotor.follow(leftMotor);

    // Inverting motors
    rightMotor.setInverted(false);
    leftMotor.setInverted(true);

    // Setting motor neutral mode to brake
    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);
    // leftMotor.setNeutralMode(NeutralMode.Coast);
    // rightMotor.setNeutralMode(NeutralMode.Coast);

    // Resetting encoder positions
    leftMotor.setSelectedSensorPosition(toPosition(-ArmConstants.startingAngle));
    rightMotor.setSelectedSensorPosition(toPosition(ArmConstants.startingAngle));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Motor Speed", leftMotor.get());

    SmartDashboard.putBoolean("Arm raise limit", raiseLimitSwitch());
    SmartDashboard.putBoolean("Arm drop limit", dropLimitSwitch());

    SmartDashboard.putNumber("Arm Angle", getAngle());
    SmartDashboard.putNumber("Arm Position", getSensorPosition());

    if (dropLimitSwitch()) {
      leftMotor.setSelectedSensorPosition(toPosition(ArmConstants.intakeAngle));
    }
    if (raiseLimitSwitch()) {
      leftMotor.setSelectedSensorPosition(toPosition(ArmConstants.shootAngle));
    }
  }
  @Override
  public void simulationPeriodic() {
    /* The thing below is what we are using to "simulate" the encoder... Not reliable, only use to test commands. DO NOT use to tune PID values.
     * There's probably a better way of doing this, but I'm too lazy. - Wilson
    */
    leftMotor.getSimCollection().addQuadraturePosition((int)(leftMotor.get() * 200.0)); // Random multiplier... THe point is that the simulated encoder works, not for it to be accurate.
  }

  public void setMotor(double speed) {
    leftMotor.set(speed); //defining public method to just the left motor, takes doubles, -> speed
   }

  public double getAngle() {
    return (getSensorPosition()*(360.0/ArmConstants.countsPerRev))/ArmConstants.gearRatio;
  }

  public double getVelocity() {
    return (getSensorVelocity()*(360.0/ArmConstants.countsPerRev))/ArmConstants.gearRatio;
  }

  public double toPosition(double angle) {
    return (angle*ArmConstants.gearRatio)/(360.0/ArmConstants.countsPerRev);
  }

  public double getSensorPosition() {
    return leftMotor.getSelectedSensorPosition();
  }

  public double getSensorVelocity() {
    return leftMotor.getSelectedSensorVelocity();
  }

  public void stopMotors() {
    leftMotor.stopMotor(); //def public stopMotors to stop all motors
    rightMotor.stopMotor();
  }

  public boolean raiseLimitSwitch() { // True when clicked, false when not
    return !raiseSwitch.get();
  }

  public boolean dropLimitSwitch() { // True when clicked, false when not
    return !dropSwitch.get();
  }
} 