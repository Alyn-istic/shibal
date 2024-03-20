// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Subsystems.ArmSubsystem;

public class ArmCommandSelector extends InstantCommand {
  private ArmSubsystem armSub;
  private int increment;
  /**
   * This is the command that the arm uses to "rotate" through different setpoints, by recieving different PID commands through a list, and running them one-by-one.
  */
  public ArmCommandSelector(
    ArmSubsystem armSub,
    int increment
  ) {
    this.armSub = armSub;
    this.increment = increment;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int selectedIndex = MathUtil.clamp(armSub.getCurrentPositionIndex() + increment, 0, ArmConstants.angles.length - 1);

    CommandScheduler.getInstance().schedule(new ArmPIDCmd(armSub,
      () -> ArmConstants.angles[selectedIndex] + SmartDashboard.getNumber("Arm Setpoint Offset", 0),
      () -> 0
    ));
  }
}
