// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.IntakeShooter.Test;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.Subsystems.IntakeShooterSubsystem;

public class intakeSeperateCmd extends Command {
  /** Creates a new intakeLowerCmd. */
  private IntakeShooterSubsystem intakeSub;
  private DoubleSupplier upperSpeed;
  private DoubleSupplier lowerSpeed;

  public intakeSeperateCmd(
    IntakeShooterSubsystem intakeSub,
    DoubleSupplier lowerSpeed, 
    DoubleSupplier upperSpeed
  ) {
    this.intakeSub = intakeSub;
    this.upperSpeed = upperSpeed;
    this.lowerSpeed = lowerSpeed;
    addRequirements(intakeSub);
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      intakeSub.setSeperateMotor(upperSpeed.getAsDouble()*IntakeShooterConstants.speed, lowerSpeed.getAsDouble()*IntakeShooterConstants.speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.setSeperateMotor(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
