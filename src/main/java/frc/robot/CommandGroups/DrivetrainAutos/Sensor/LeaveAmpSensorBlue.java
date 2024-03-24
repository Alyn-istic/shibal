// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.DrivetrainAutos.Sensor;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandGroups.MainAutos.AutoLog;
import frc.robot.Commands.Drivetrain.TankDrivePIDCmd;
import frc.robot.Commands.Drivetrain.TurnPIDCmd;
import frc.robot.Constants.AutonomousConstants.MoveOutOfZoneConstants;
import frc.robot.Constants.AutonomousConstants.MoveOutOfZoneSensorConstants;
import frc.robot.Subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeaveAmpSensorBlue extends SequentialCommandGroup {
  /** Creates a new LeaveAmpSensor. */
  public LeaveAmpSensorBlue(
    DrivetrainSubsystem driveSub
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoLog("Leaving the amp."),
      new TankDrivePIDCmd(driveSub,
        () -> driveSub.getLeftDistance() + MoveOutOfZoneSensorConstants.LEAVE_AMP_DISTANCE,
        () -> driveSub.getRightDistance() + MoveOutOfZoneSensorConstants.LEAVE_AMP_DISTANCE,
        () -> 0.05,
        () -> false,
        () -> true,
        () -> driveSub.isDriveControllersAtSetpoint()
        ).withTimeout(1),
      new AutoLog("Turning away from the amp"),
      new TurnPIDCmd(driveSub,
        () -> (-MoveOutOfZoneConstants.LEAVE_AMP_ANGLE) % 360,
        () -> 10,
        () -> false,
        () -> driveSub.getTurnController().atSetpoint()
      ).withTimeout(5),
      new AutoLog("Moving out of zone"),
      new MoveOutOfZoneSensor(driveSub)
    );
  }
}
