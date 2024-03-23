// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.MainAutos.Sensor;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandGroups.ArmAutos.ArmMobile;
import frc.robot.CommandGroups.ArmAutos.ArmShoot;
import frc.robot.CommandGroups.ArmAutos.ArmZero;
import frc.robot.CommandGroups.ArmAutos.ArmZeroUp;
import frc.robot.CommandGroups.DrivetrainAutos.Sensor.LeaveAmpSensorRed;
import frc.robot.CommandGroups.DrivetrainAutos.Sensor.RamIntoAmpSensor;
import frc.robot.CommandGroups.IntakeShooterAutos.DownShootAmpTimed;
import frc.robot.CommandGroups.MainAutos.AutoLog;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeShooterSubsystem;
import frc.robot.Subsystems.LEDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreInAmpSensor1Red extends SequentialCommandGroup {
  /** Creates a new ScoreInAmpSensor1Red. */
  public ScoreInAmpSensor1Red(
    DrivetrainSubsystem driveSub,
    ArmSubsystem armSub,
    IntakeShooterSubsystem intakeSub,
    LEDSubsystem led
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new ArmZeroUp(armSub),
        new RamIntoAmpSensor(driveSub).withTimeout(1)
      ),
      new ArmShoot(armSub, () -> armSub.getController().atSetpoint()),
      new DownShootAmpTimed(intakeSub),
      new ParallelCommandGroup(
        new LeaveAmpSensorRed(driveSub).andThen(new AutoLog("Done")),
        new ArmMobile(armSub, () -> false)
      )
    );
  }
}
