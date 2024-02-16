// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  // Initializing the TalonSRX motorcontrollers
  private final WPI_TalonSRX leftMotor = new WPI_TalonSRX(ArmConstants.leftID);
  private final WPI_TalonSRX rightMotor = new WPI_TalonSRX(ArmConstants.rightID);

  private double fakeSensorPos = 0;

  // Initializing the limit switches
  private final DigitalInput dropSwitch = new DigitalInput(ArmConstants.dropLimitSwitchChannel);
  private final DigitalInput raiseSwitch = new DigitalInput(ArmConstants.raiseLimitSwitchChannel);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // Reseting the motors to factory default
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    // Setting follower motor
    rightMotor.follow(leftMotor);

    // Inverting motors
    rightMotor.setInverted(true);
    leftMotor.setInverted(false);

    // Setting motor neutral mode to brake
    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);

    // Resetting encoder positions
    leftMotor.setSelectedSensorPosition(0);
    rightMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Motor Speed", leftMotor.get());

    SmartDashboard.putBoolean("Arm raise limit", raiseLimitSwitch());
    SmartDashboard.putBoolean("Arm drop limit", dropLimitSwitch());

    SmartDashboard.putNumber("Arm Angle", getAngle() % 360);
    SmartDashboard.putNumber("Arm Position", leftMotor.getSelectedSensorPosition());
    System.out.println(getAngle());

    if (dropLimitSwitch()) {
      leftMotor.setSelectedSensorPosition(ArmConstants.intakeAngle);
      rightMotor.setSelectedSensorPosition(ArmConstants.intakeAngle);
    } else if (raiseLimitSwitch()) {
      leftMotor.setSelectedSensorPosition(ArmConstants.shootAngle);
      rightMotor.setSelectedSensorPosition(ArmConstants.shootAngle);
    }

    /* The thing below is what we are using to "simulate" the encoder... Not reliable, only use to test commands. DO NOT use to tune PID values.
     * There's probably a better way of doing this, but I'm too lazy. - Wilson
    */
    // fakeSensorPos += (leftMotor.get() * 30);
    // leftMotor.setSelectedSensorPosition(fakeSensorPos);
  }

  public void setMotor(double speed) {
    leftMotor.set(speed);
   }

  public double getAngle() {
    return (leftMotor.getSelectedSensorPosition()*(360.0/ArmConstants.ticksPerRev))/ArmConstants.gearRatio;
  }

  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public boolean raiseLimitSwitch() { // True when clicked, false when not
    return !raiseSwitch.get();
  }

  public boolean dropLimitSwitch() { // True when clicked, false when not
    return !dropSwitch.get();
  }

} 