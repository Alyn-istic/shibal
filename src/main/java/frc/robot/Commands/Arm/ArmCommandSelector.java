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

  /** Creates a new ArmPIDLoop. */
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
