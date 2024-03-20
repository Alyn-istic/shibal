// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ArmSubsystem;

public class ArmManualCmd extends Command {
  private ArmSubsystem armSub;
  private DoubleSupplier speedInput;
  
  /**
   * Manually apply speed to the arm
   * @param armSub Arm Subsystem
   * @param speedInput Supplier that returns the speed input
   */
  public ArmManualCmd(
    ArmSubsystem armSub,
    DoubleSupplier speedInput
  ) {
    this.armSub = armSub;
    this.speedInput = speedInput;
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = speedInput.getAsDouble();
    if ((speed < 0) && armSub.raiseLimitSwitchHit()) {
      speed = 0;
    } else if ((speed > 0) && armSub.dropLimitSwitchHit()) {
      speed = 0;
    }
    armSub.setMotor(speed);

    armSub.updatePositionIndex(armSub.getAngle());
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
