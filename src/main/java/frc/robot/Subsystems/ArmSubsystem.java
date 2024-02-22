// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

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
  private final DigitalInput raiseSwitch = new DigitalInput(ArmConstants.raiseLimitSwitchChannel);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    //following makes it so it acts in tangent
    rightMotor.follow(leftMotor);
    leftMotor.setInverted(true);

    // leftMotor.setIdleMode(IdleMode.kBrake);
    // rightMotor.setIdleMode(IdleMode.kBrake);

    leftMotor.setIdleMode(IdleMode.kCoast);
    rightMotor.setIdleMode(IdleMode.kCoast);

    encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Motor Speed", leftMotor.get());
    SmartDashboard.putNumber("Arm Motor Voltage", leftMotor.getAppliedOutput());

    SmartDashboard.putBoolean("Arm raise limit", raiseLimitSwitch());
    SmartDashboard.putBoolean("Arm drop limit", dropLimitSwitch());

    SmartDashboard.putNumber("Arm Angle", getAngle() % 360);
    System.out.println("Arm angle: " + getAngle());

    if (!dropLimitSwitch()) {
      encoder.setPosition(0); //if hit, reset encoders
    }
  }

  public void setMotor(double speed) {
    leftMotor.set(speed); //defining public method to just the left motor, takes doubles, -> speed
   }

  public double getAngle() {
    return (encoder.getPosition()*(360/ArmConstants.ticksPerRev))/ArmConstants.gearRatio; //get angle of arm -> encoder position * 360/1024 )/gear ratio-> get angle if done correctly
  }

  public void stopMotors() {
    leftMotor.stopMotor(); //def public stopMotors to stop all motors
    rightMotor.stopMotor();
  }

  public boolean raiseLimitSwitch() {
    return !raiseSwitch.get(); //boolean of raise switch -> will return true if hit,alse if not
  }

  public boolean dropLimitSwitch() {
    return !dropSwitch.get();
  }

} 