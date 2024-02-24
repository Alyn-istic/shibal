// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private final CANSparkMax leftClimber = new CANSparkMax(ClimberConstants.leftClimberID, MotorType.kBrushless);
  private final CANSparkMax rightClimber = new CANSparkMax(ClimberConstants.rightClimberID, MotorType.kBrushless);
  public ClimberSubsystem() {
    rightClimber.follow(leftClimber);
    leftClimber.setInverted(true);
    leftClimber.setIdleMode(IdleMode.kBrake);
    rightClimber.setIdleMode(IdleMode.kBrake);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setMotor(double speed){
    leftClimber.set(speed);
  }
}
