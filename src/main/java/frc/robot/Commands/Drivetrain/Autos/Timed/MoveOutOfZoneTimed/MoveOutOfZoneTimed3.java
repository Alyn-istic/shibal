// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain.Autos.Timed.MoveOutOfZoneTimed;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Autos.AutoLog;
import frc.robot.Commands.Drivetrain.TankDriveCmd;
import frc.robot.Constants.AutonomousConstants.MoveOutOfZoneConstants;
import frc.robot.Subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveOutOfZoneTimed3 extends SequentialCommandGroup {
  /** Creates a new MoveOutOfZone. */
  public MoveOutOfZoneTimed3(
    DrivetrainSubsystem driveSub
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoLog("moving out of zone, pos.3"),
      new TankDriveCmd(driveSub, () -> MoveOutOfZoneConstants.SPEED_MOVE_OUT_OF_ZONE3, () -> MoveOutOfZoneConstants.SPEED_MOVE_OUT_OF_ZONE3).withTimeout(MoveOutOfZoneConstants.TIME_MOVE_OUT_OF_ZONE3)
    );
  }
}
