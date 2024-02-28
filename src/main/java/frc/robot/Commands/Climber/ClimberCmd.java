// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Climber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimberSubsystem;
import java.util.function.DoubleSupplier;

public class ClimberCmd extends Command {
  /** Creates a new ClimberCmb. */
  private ClimberSubsystem climbSub;
  private DoubleSupplier speed;

  /**
   * Apply speed to the climber motors
   * @param climbSub Climber Subsystem
   * @param speed Supplier that returns the speed of the motors
   */
  public ClimberCmd(ClimberSubsystem climbSub, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed;
    this.climbSub = climbSub;
    addRequirements(climbSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbSub.setMotor(speed.getAsDouble());
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
