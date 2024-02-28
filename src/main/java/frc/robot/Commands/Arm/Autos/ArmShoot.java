// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm.Autos;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Commands.Arm.ArmPIDCmd;
import frc.robot.Subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmShoot extends SequentialCommandGroup {
/**
   * Move the arm into shooting position. This is for down-shooting into the amp
   * @param armSub Arm Subsystem
   * @param end Supplier that returns true when the command should end
   */  public ArmShoot(
    ArmSubsystem armSub, BooleanSupplier end
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmPIDCmd(armSub,
        () -> ArmConstants.raiseP,
        () -> ArmConstants.raiseI,
        () -> ArmConstants.raiseD,
        () -> ArmConstants.dropP,
        () -> ArmConstants.dropI,
        () -> ArmConstants.dropD,
        () -> ArmConstants.shootAngle,
        () -> ArmConstants.tolerance,
        () -> ArmConstants.clamp
      ).until(end)
    );
  }
}
