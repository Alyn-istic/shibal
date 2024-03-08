// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmSetpointOffset extends InstantCommand {
  private DoubleSupplier increment;
  /**
   * Offsets the arm's setpoint.
   * @param increment The amount added to the offset (degrees)
   */
  public ArmSetpointOffset(
    DoubleSupplier increment
  ) {
    this.increment = increment;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Arm Setpoint Offset", SmartDashboard.getNumber("Arm Setpoint Offset", 0) + increment.getAsDouble());
  }
}
