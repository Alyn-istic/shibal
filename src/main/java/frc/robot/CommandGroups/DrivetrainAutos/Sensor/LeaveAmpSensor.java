// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.DrivetrainAutos.Sensor;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandGroups.MainAutos.AutoLog;
import frc.robot.Commands.Drivetrain.TankDrivePIDCmd;
import frc.robot.Commands.Drivetrain.TurnPIDCmd;
import frc.robot.Subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeaveAmpSensor extends SequentialCommandGroup {
  /** Creates a new LeaveAmpSensor. */
  public LeaveAmpSensor(
    DrivetrainSubsystem driveSub
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoLog("Leaving the amp."),
      new TankDrivePIDCmd(driveSub,
        () -> driveSub.getLeftDistance() - 0.25,
        () -> driveSub.getRightDistance() - 0.25,
        () -> 0.05,
        () -> false,
        () -> true,
        () -> driveSub.isDriveControllersAtSetpoint()
        ),
      new AutoLog("Turning away from the amp"),
      new TurnPIDCmd(driveSub,
        () -> 90,
        () -> 10,
        () -> false,
        () -> driveSub.getTurnController().atSetpoint()
      ),
      new TankDrivePIDCmd(driveSub,
        () -> driveSub.getLeftDistance() + 3,
        () -> driveSub.getRightDistance() + 3,
        () -> 0.05,
        () -> false,
        () -> true,
        () -> driveSub.isDriveControllersAtSetpoint()
      )
    );
  }
}
