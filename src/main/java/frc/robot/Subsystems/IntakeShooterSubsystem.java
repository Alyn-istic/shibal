// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeShooterConstants;

public class IntakeShooterSubsystem extends SubsystemBase {
  // Initiating TalonSRX motors.
  private final WPI_VictorSPX upperWheel = new WPI_VictorSPX(IntakeShooterConstants.upperWheelID);
  private final WPI_VictorSPX lowerWheel = new WPI_VictorSPX(IntakeShooterConstants.lowerWheelID);

  /** Creates a new IntakeShooterSubsystem. */
  public IntakeShooterSubsystem() {
    upperWheel.setInverted(false);
    lowerWheel.setInverted(false);
    upperWheel.setInverted(false);
     // Setting the neutral mode to brake. This means the motors will stop immediately when told to.
    upperWheel.setNeutralMode(NeutralMode.Brake);
    lowerWheel.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // Pushing stuff to SmartDashboard for debugging purposes.
    // SmartDashboard.putNumber("Intake/Shooter Motor 1 Speed", upperWheel.get());
    // SmartDashboard.putNumber("Intake/Shooter Motor 2 Speed", lowerWheel.get());
  }

  public void setMotors(double speed) { // This will set the intake motors' speed.
    upperWheel.set(speed);
    lowerWheel.set(speed);
  }

  public void setSeperateMotor(double upperSpeed, double lowerSpeed){
    upperWheel.set(upperSpeed);
    lowerWheel.set(lowerSpeed);
  }
  
  public boolean getIntakeLimit() { // Returns if the note has been detected by a intake sensor
    return true;
  }
}