// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.MainAutos.Sensor;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.CommandGroups.ArmAutos.ArmIntakePerimeter;
import frc.robot.CommandGroups.ArmAutos.ArmZero;
import frc.robot.Commands.Drivetrain.TankDrivePIDCmd;
import frc.robot.Commands.Drivetrain.TankDriveVisionPIDCmd;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExitZoneSensor extends ParallelCommandGroup {
  /** Creates a new ExitZoneSensor. */
  public ExitZoneSensor(
    DrivetrainSubsystem driveSub,
    ArmSubsystem armSub
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TankDrivePIDCmd(driveSub, 
        () -> driveSub.getLeftDistance() + 7,
        () -> driveSub.getRightDistance() + 7,
        () -> 0.1,
        () -> false,
        () -> false
      ),
      new ArmZero(armSub).andThen(new ArmIntakePerimeter(armSub, () -> armSub.getController().atSetpoint()))
    );
  }
}
