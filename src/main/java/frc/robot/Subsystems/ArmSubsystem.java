// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax leftMotor = new CANSparkMax(ArmConstants.leftID, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(ArmConstants.rightID, MotorType.kBrushless);

  private final RelativeEncoder encoder = leftMotor.getEncoder(); // Set this to whatever encoder we are using.

  //private final DigitalInput raiseSwitch = new DigitalInput(ArmConstants.raiseLimitSwitchChannel);
  private final DigitalInput dropSwitch = new DigitalInput(ArmConstants.dropLimitSwitchChannel);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    rightMotor.follow(leftMotor);
    leftMotor.setInverted(true);

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Motor Speed", leftMotor.get());
    SmartDashboard.putNumber("Arm Motor Voltage", leftMotor.getAppliedOutput());

    SmartDashboard.putBoolean("Arm raise limit", raiseLimitSwitch());
    SmartDashboard.putBoolean("Arm drop limit", dropLimitSwitch());
  }

  public void setMotor(double speed) {
    leftMotor.set(speed);
   }

  public double getAngle() {
    return (encoder.getPosition()*(360/ArmConstants.ticksPerRev))/ArmConstants.gearRatio;
  }

  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public boolean raiseLimitSwitch() {
    return true;
    //return raiseSwitch.get();
  }

  public boolean dropLimitSwitch() {
    return dropSwitch.get();
  }

} 