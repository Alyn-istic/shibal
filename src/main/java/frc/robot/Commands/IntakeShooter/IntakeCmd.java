// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.IntakeShooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.Subsystems.IntakeShooterSubsystem;
import frc.robot.Subsystems.LEDSubsystem;

public class IntakeCmd extends Command {
  private IntakeShooterSubsystem intakeSub;
  private LEDSubsystem led;
  private DoubleSupplier speed;
  /** Creates a new IntakeTest. */
  public IntakeCmd(
    IntakeShooterSubsystem intakeSub,
    LEDSubsystem led,
    DoubleSupplier speed
  ) {
    this.intakeSub = intakeSub;
    this.speed = speed;
    this.led = led;
    addRequirements(intakeSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.setPresetGold();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSub.setMotors(speed.getAsDouble()*IntakeShooterConstants.speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.setMotors(0);
    led.setPresetGreen();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
