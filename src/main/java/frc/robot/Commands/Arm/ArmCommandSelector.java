// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import java.util.function.IntSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmCommandSelector extends InstantCommand {
  private Command[] commandList;
  private IntSupplier increment;

  private NetworkTableEntry armIndexEntry;

  /**
   * This is the command that the arm uses to "rotate" through different setpoints, by recieving different PID commands through a list, and running them one-by-one.
   * @param commandList List containing the PID commands
   * @param increment How much to go up or go down by in the list
   * @param armIndexEntry Currenty running command's index in the list
   */
  public ArmCommandSelector(
    Command[] commandList,
    IntSupplier increment,
    NetworkTableEntry armIndexEntry
  ) {
    this.commandList = commandList;
    this.increment = increment;
    this.armIndexEntry = armIndexEntry;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armIndexEntry.setInteger(
      MathUtil.clamp(
        ((int) armIndexEntry.getInteger(0) + increment.getAsInt()),
        0,
        commandList.length-1)
    );
    CommandScheduler.getInstance().schedule(commandList[(int) armIndexEntry.getInteger(0)]);
  }
}
