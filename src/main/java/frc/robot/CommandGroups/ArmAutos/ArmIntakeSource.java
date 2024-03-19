// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.ArmAutos;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Commands.Arm.ArmPIDCmd;
import frc.robot.Subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmIntakeSource extends SequentialCommandGroup {
  /**
   * Move the arm into source-intaking position
   * @param armSub Arm Subsystem
   * @param end Supplier that returns true when the command should end
   */
  public ArmIntakeSource(
    ArmSubsystem armSub, BooleanSupplier end
  ) {
    addCommands(
      new ArmPIDCmd(armSub,
        () -> ArmConstants.sourceIntakeAngle,
        () -> ArmConstants.tolerance
      ).until(end)
    );
  }
}
