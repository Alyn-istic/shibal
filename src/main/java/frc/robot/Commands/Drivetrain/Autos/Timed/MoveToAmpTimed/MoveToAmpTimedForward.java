// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain.Autos.Timed.MoveToAmpTimed;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutonomousConstants.MoveToAmpTimedConstants;
import frc.robot.Commands.Drivetrain.TankDriveAutoCmd;
import frc.robot.Commands.MainAutos.AutoLog;
import frc.robot.Subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveToAmpTimedForward extends SequentialCommandGroup {
  /** Creates a new MoveToAmpTimedForward. */
  public MoveToAmpTimedForward(
    DrivetrainSubsystem driveSub
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoLog("Moving forward to align with the amp"),
      new TankDriveAutoCmd(driveSub,
        () -> MoveToAmpTimedConstants.SPEED_MOVE_TO_AMP_FWD,
        () -> MoveToAmpTimedConstants.SPEED_MOVE_TO_AMP_FWD,
        () -> MoveToAmpTimedConstants.TIME_MOVE_TO_AMP_FWD
      )
    );
  }
}
