// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax leftMotor = new CANSparkMax(ArmConstants.leftID, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(ArmConstants.rightID, MotorType.kBrushless);

  private final RelativeEncoder encoder = leftMotor.getEncoder(); // Set this to whatever encoder we are using.

  private final DigitalInput switch1 = new DigitalInput(ArmConstants.raiseLimitPort1);
  private final DigitalInput switch2 = new DigitalInput(ArmConstants.raiseLimitPort2);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    rightMotor.follow(leftMotor);
    leftMotor.setInverted(true);

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    encoder.setPosition(0);
  }

  @Override
  public void periodic() {}

  public void setMotor(double speed) {
    leftMotor.set(speed);
  }

  public double getAngle() {
    return (encoder.getPosition() * (360/ArmConstants.ticksPerRev));
  }

  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public boolean raiseLimitSwitched() {
    return (!switch1.get() && !switch2.get());
  }
}