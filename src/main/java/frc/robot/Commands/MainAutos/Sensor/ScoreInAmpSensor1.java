// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.MainAutos.Sensor;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Arm.Autos.ArmIntake;
import frc.robot.Commands.Arm.Autos.ArmShoot;
import frc.robot.Commands.Arm.Autos.ArmZero;
import frc.robot.Commands.Drivetrain.TankDrivePIDCmd;
import frc.robot.Commands.Drivetrain.Autos.Sensor.ChassisTurn0;
import frc.robot.Commands.Drivetrain.Autos.Sensor.ChassisTurn270;
import frc.robot.Commands.Drivetrain.Autos.Sensor.ChassisTurn315;
import frc.robot.Commands.IntakeShooter.IntakeCmd;
import frc.robot.Commands.MainAutos.AutoLog;
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
      new ArmZero(armSub),
      // new MoveOutOfZoneSensor(driveSub),
      new AutoLog("Driving backwards towards amp..."),
      new TankDrivePIDCmd(driveSub, // Moves towards the amp
        () -> driveSub.getLeftDistance() - 0.58,
        () -> driveSub.getRightDistance() - 0.58,
        () -> 0.1,
        () -> false,
        () -> driveSub.isDriveControllersAtSetpoint()
      ),
      new AutoLog("Moving arm into shooting position..."),
      new ArmShoot(armSub, () -> armSub.getController().atSetpoint()),
      new AutoLog("Down-shooting into amp..."),
      new IntakeCmd(intakeSub, () -> 1).withTimeout(1),
      new ArmIntake(armSub,  () -> armSub.getController().atSetpoint()),
      new TankDrivePIDCmd(driveSub, // Moves towards the amp
        () -> driveSub.getLeftDistance() + 0.58,
        () -> driveSub.getRightDistance() + 0.58,
        () -> 0.1,
        () -> false,
        () -> driveSub.isDriveControllersAtSetpoint()
      ),
      new ChassisTurn270(driveSub),
      new TankDrivePIDCmd(driveSub, // Moves towards the amp
        () -> driveSub.getLeftDistance() + 1,
        () -> driveSub.getRightDistance() + 1,
        () -> 0.1,
        () -> false,
        () -> driveSub.isDriveControllersAtSetpoint()
      ).raceWith(new IntakeCmd(intakeSub, () -> 1)),

      new TankDrivePIDCmd(driveSub, // Moves towards the amp
        () -> driveSub.getLeftDistance() - 1,
        () -> driveSub.getRightDistance() - 1,
        () -> 0.1,
        () -> false,
        () -> driveSub.isDriveControllersAtSetpoint()
      ),

      new ChassisTurn0(driveSub),
      new TankDrivePIDCmd(driveSub, // Moves towards the amp
        () -> driveSub.getLeftDistance() - 0.58,
        () -> driveSub.getRightDistance() - 0.58,
        () -> 0.1,
        () -> false,
        () -> driveSub.isDriveControllersAtSetpoint()
      ).alongWith(new ArmShoot(armSub,  () -> armSub.getController().atSetpoint())),
      new IntakeCmd(intakeSub, () -> 1).withTimeout(1),
      
      new TankDrivePIDCmd(driveSub, // Moves towards the amp
        () -> driveSub.getLeftDistance() + 0.58,
        () -> driveSub.getRightDistance() + 0.58,
        () -> 0.1,
        () -> false,
        () -> driveSub.isDriveControllersAtSetpoint()
      ),
      new ChassisTurn315(driveSub).alongWith(new ArmIntake(armSub,  () -> armSub.getController().atSetpoint())),

      new AutoLog("Done")
    );
  }
}
