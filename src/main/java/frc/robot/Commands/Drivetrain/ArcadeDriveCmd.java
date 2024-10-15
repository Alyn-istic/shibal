// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class ArcadeDriveCmd extends Command {
  private DrivetrainSubsystem driveSub;
  private DoubleSupplier leftInput, rightInput;

  /** Creates a new ArcadeDriveCmd. */
  public ArcadeDriveCmd(
    DrivetrainSubsystem driveSub,
    DoubleSupplier leftInput, DoubleSupplier rightInput
  ) {
    this.driveSub = driveSub;
    this.leftInput = leftInput;
    this.rightInput = rightInput;
    addRequirements(driveSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left = leftInput.getAsDouble() * DrivetrainConstants.speed;
    double right = rightInput.getAsDouble() * DrivetrainConstants.turnSpeed;

    driveSub.arcadeDriveSpeed(
      Math.signum(left)*(Math.sqrt(Math.abs(left))),
      Math.signum(right)*(Math.sqrt(Math.abs(right)))
    );

    // Pushing numbers onto SmartDashboard for debugging purposes.
    SmartDashboard.putNumber("Drivetrain Left Speed Input", leftInput.getAsDouble());
    SmartDashboard.putNumber("Drivetrain Right Speed Input", rightInput.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
