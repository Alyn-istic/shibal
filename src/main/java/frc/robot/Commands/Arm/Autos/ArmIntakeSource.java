// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm.Autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Commands.Arm.ArmPIDCmd;
import frc.robot.Subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmIntakeSource extends SequentialCommandGroup {
  /** Creates a new ArmIntakeSource. */
  public ArmIntakeSource(
    ArmSubsystem armSub
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
        () -> ArmConstants.sourceIntakeAngle,
        () -> ArmConstants.tolerance,        
        () -> ArmConstants.clamp
      )
    );
  }
}
