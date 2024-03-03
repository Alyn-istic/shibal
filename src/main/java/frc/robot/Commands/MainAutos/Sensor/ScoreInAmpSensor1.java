// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.MainAutos.Sensor;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Arm.Autos.ArmShoot;
import frc.robot.Commands.Drivetrain.TankDrivePIDCmd;
import frc.robot.Commands.Drivetrain.TurnPIDCmd;
import frc.robot.Commands.Drivetrain.Autos.Sensor.ChassisTurn0;
import frc.robot.Commands.Drivetrain.Autos.Sensor.ChassisTurn90;
import frc.robot.Commands.Drivetrain.Autos.Sensor.MoveOutOfZoneSensor;
import frc.robot.Commands.IntakeShooter.IntakeCmd;
import frc.robot.Commands.MainAutos.AutoLog;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreInAmpSensor1 extends SequentialCommandGroup {
  /** Creates a new TestAuto. */
  public ScoreInAmpSensor1(
    DrivetrainSubsystem driveSub,
    ArmSubsystem armSub,
    IntakeShooterSubsystem intakeSub
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new ChassisTurn0(driveSub),
      new MoveOutOfZoneSensor(driveSub),
      new ChassisTurn90(driveSub),
      new AutoLog("Driving backwards towards amp..."),
      new TankDrivePIDCmd(driveSub, // Moves towards the amp
        () -> driveSub.getLeftDistance() - 0.5,
        () -> driveSub.getRightDistance() - 0.5,
        () -> 0.01,
        () -> false,
        () -> driveSub.isDriveControllersAtSetpoint()
      ),
      new AutoLog("Moving arm into shooting position..."),
      new ArmShoot(armSub, () -> armSub.getController().atSetpoint()),
      new AutoLog("Down-shooting into amp..."),
      new IntakeCmd(intakeSub, () -> -1).withTimeout(2),
      new AutoLog("Done")
    );
  }
}
