// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private final TalonSRX leftMotor = new TalonSRX(ArmConstants.leftID);
  private final TalonSRX rightMotor = new TalonSRX(ArmConstants.rightID);

  //private final DigitalInput raiseSwitch = new DigitalInput(ArmConstants.raiseLimitSwitchChannel);
  private final DigitalInput dropSwitch = new DigitalInput(ArmConstants.dropLimitSwitchChannel);
  private final DigitalInput raiseSwitch = new DigitalInput(ArmConstants.raiseLimitSwitchChannel);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    rightMotor.follow(leftMotor);
    leftMotor.setInverted(true);

    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);

    // leftMotor.setNeutralMode(NeutralMode.Coast);
    // rightMotor.setNeutralMode(NeutralMode.Coast);

    leftMotor.setSelectedSensorPosition(0);
    rightMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Motor Speed", leftMotor.getMotorOutputPercent());

    SmartDashboard.putBoolean("Arm raise limit", raiseLimitSwitch());
    SmartDashboard.putBoolean("Arm drop limit", dropLimitSwitch());

    SmartDashboard.putNumber("Arm Angle", getAngle() % 360);
    //System.out.println("Arm angle: " + getAngle());
    // System.out.println("Arm Pos:" + leftMotor.getSelectedSensorPosition());

    if (!dropLimitSwitch()) {
      leftMotor.setSelectedSensorPosition(0);
      rightMotor.setSelectedSensorPosition(0);
    }
  }

  public void setMotor(double speed) {
    leftMotor.set(TalonSRXControlMode.Position, speed);
   }

  public double getAngle() {
    return -(leftMotor.getSelectedSensorPosition()*(360/ArmConstants.ticksPerRev))/ArmConstants.gearRatio;
  }

  public void stopMotors() {
    leftMotor.set(TalonSRXControlMode.Position, 0);
    leftMotor.set(TalonSRXControlMode.Position, 0);

  }

  public boolean raiseLimitSwitch() {
    return !raiseSwitch.get();
  }

  public boolean dropLimitSwitch() {
    return !dropSwitch.get();
  }

} 