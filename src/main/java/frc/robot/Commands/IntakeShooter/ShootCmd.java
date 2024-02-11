// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.IntakeShooter;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeShooterSubsystem;




public class ShootCmd extends Command {
  /** Creates a new IntakeCmd. */
  private IntakeShooterSubsystem intakeSub;
  private DoubleSupplier speedInput;
  public ShootCmd(
    IntakeShooterSubsystem intakeSub,
    DoubleSupplier speedInput
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSub = intakeSub;
    this.speedInput = speedInput;


    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = speedInput.getAsDouble();
    intakeSub.setMotors(speed);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.setMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}