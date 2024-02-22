// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ArmSubsystem;

public class ArmCmd extends Command {
  private ArmSubsystem armSub;
  private DoubleSupplier raiseInput;
  /** Creates a new ArmTestCmd. */
  public ArmCmd(
    ArmSubsystem armSub,
    DoubleSupplier raiseInput
  ) {
    this.armSub = armSub;
    this.raiseInput = raiseInput;
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double raise = raiseInput.getAsDouble();
    if (armSub.raiseLimitSwitch() || armSub.dropLimitSwitch()) {
      raise = 0 ;
    }
    armSub.setMotor(raise);
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
