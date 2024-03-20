// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.MainAutos.Sensor;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandGroups.ArmAutos.ArmIntake;
import frc.robot.CommandGroups.ArmAutos.ArmShoot;
import frc.robot.CommandGroups.ArmAutos.ArmZero;
import frc.robot.CommandGroups.DrivetrainAutos.Sensor.ChassisTurn0;
import frc.robot.CommandGroups.DrivetrainAutos.Sensor.ChassisTurn270;
import frc.robot.CommandGroups.DrivetrainAutos.Sensor.ChassisTurn315;
import frc.robot.CommandGroups.MainAutos.AutoLog;
import frc.robot.Commands.Drivetrain.TankDrivePIDCmd;
import frc.robot.Commands.Drivetrain.TankDriveVisionPIDCmd;
import frc.robot.Commands.IntakeShooter.IntakeCmd;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeShooterSubsystem;
import frc.robot.Subsystems.LEDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreInAmpSensor1 extends SequentialCommandGroup {
  /** Creates a new TestAuto. */
  public ScoreInAmpSensor1(
    DrivetrainSubsystem driveSub,
    ArmSubsystem armSub,
    IntakeShooterSubsystem intakeSub,
    LEDSubsystem led
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new ParallelCommandGroup(
      new ArmZero(armSub).andThen(new ArmShoot(armSub, () -> armSub.getController().atSetpoint())),
      new TankDrivePIDCmd( // Drive to Amp
        driveSub,
        () ->driveSub.getLeftDistance() - 0.58,
        () ->driveSub.getRightDistance() - 0.58,
        () -> 0.05,
        () -> false,
        () -> driveSub.isDriveControllersAtSetpoint()
      ),
      new IntakeCmd(intakeSub, () ->-1),
      new TankDrivePIDCmd( // Drive away from Amp
        driveSub,
        () ->driveSub.getLeftDistance() + 0.2,
        () ->driveSub.getRightDistance() + 0.2,
        () -> 0.05,
        () -> false,
        () -> driveSub.isDriveControllersAtSetpoint()
      ).alongWith(new ArmIntake(armSub, () -> armSub.getController().atSetpoint()))

      )
    );
  }
}
