// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm.Autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Commands.Arm.ArmPIDCmd;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmIntake extends SequentialCommandGroup {
  /** Creates a new Intake. */
  public ArmIntake(
    ArmSubsystem armSub
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmPIDCmd(armSub, //
          // () -> ArmConstants.kP,
          // () -> ArmConstants.kI,
          // () -> ArmConstants.kD,
          () -> SmartDashboard.getNumber("Arm P", 0),
          () -> SmartDashboard.getNumber("Arm I", 0),
          () -> SmartDashboard.getNumber("Arm D", 0),
          () -> ArmConstants.intakeAngle,
          () -> ArmConstants.tolerance,
          () -> ArmConstants.clamp
      )
    );
  }
}
