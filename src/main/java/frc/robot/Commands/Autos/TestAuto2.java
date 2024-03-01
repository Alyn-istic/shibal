// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Drivetrain.TankDrivePIDCmd;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto2 extends SequentialCommandGroup {
  /** Creates a new TestAuto. */
  public TestAuto2(
    DrivetrainSubsystem driveSub
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TankDrivePIDCmd(driveSub,
        // () -> DrivetrainConstants.driveP,
        // () -> DrivetrainConstants.driveI,
        // () -> DrivetrainConstants.driveD,
        // () -> DrivetrainConstants.turnP,
        // () -> DrivetrainConstants.turnI,
        // () -> DrivetrainConstants.turnD,
        () -> driveSub.getLeftDistance(),
        () -> driveSub.getRightDistance(),
        () -> 0,
        () -> 0,
        () -> 1
      ).until(() -> driveSub.getTurnController().atSetpoint()),
      new TankDrivePIDCmd(driveSub,
        // () -> DrivetrainConstants.driveP,
        // () -> DrivetrainConstants.driveI,
        // () -> DrivetrainConstants.driveD,
        // () -> DrivetrainConstants.turnP,
        // () -> DrivetrainConstants.turnI,
        // () -> DrivetrainConstants.turnD,
        () -> -1,
        () -> -1,
        () -> driveSub.getGyroAngle() % 360,
        () -> 0.1,
        () -> 0
      ).until(() -> (driveSub.getLeftDriveController().atSetpoint() && driveSub.getRightDriveController().atSetpoint()))
    );
  }
}
