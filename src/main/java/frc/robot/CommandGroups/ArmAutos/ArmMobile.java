// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.ArmAutos;

import java.util.function.BooleanSupplier;

import frc.robot.Commands.Arm.ArmPIDCmd;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class ArmMobile extends SequentialCommandGroup {
 /**
   * Slowly lowers the arm until 25 deg angle is reached
   * @param Constants Arm Subsystem
   * @param end
   */
  public ArmMobile(
    ArmSubsystem armSub, BooleanSupplier end
  ) {
    addCommands(
      new ArmPIDCmd(armSub,
        () -> ArmConstants.raiseP,
        () -> ArmConstants.raiseI,
        () -> ArmConstants.raiseD,
        () -> ArmConstants.dropP,
        () -> ArmConstants.dropI,
        () -> ArmConstants.dropD,
        () -> ArmConstants.drivingUnderStage,
        () -> ArmConstants.tolerance
      ).until(end)
    );
  }
}

