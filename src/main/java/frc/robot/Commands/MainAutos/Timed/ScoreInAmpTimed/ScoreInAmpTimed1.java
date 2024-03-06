// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.MainAutos.Timed.ScoreInAmpTimed;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.Arm.Autos.ArmShoot;
import frc.robot.Commands.Arm.Autos.ArmZero;
import frc.robot.Commands.Drivetrain.Autos.Timed.LeaveAmpTimed;
import frc.robot.Commands.Drivetrain.Autos.Timed.TurnToAmpTimed;
import frc.robot.Commands.Drivetrain.Autos.Timed.MoveToAmpTimed.MoveToAmpTimedForward;
import frc.robot.Commands.IntakeShooter.Autos.DownShootAmpTimed;
import frc.robot.Subsystems.ArmSubsystem;
//import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreInAmpTimed1 extends SequentialCommandGroup {
  /** Creates a new AutonomousBackup. */
  public ScoreInAmpTimed1(
    DrivetrainSubsystem driveSub,
    IntakeShooterSubsystem intakeShooterSub,
    ArmSubsystem armSub
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmZero(armSub).until(() -> armSub.dropLimitSwitch()),
      
      new MoveToAmpTimedForward(driveSub),
      new WaitCommand(1),

      new TurnToAmpTimed(driveSub, true),
      new WaitCommand(1),

      new ArmShoot(armSub, () -> armSub.getController().atSetpoint()),
      new DownShootAmpTimed(intakeShooterSub),
      new WaitCommand(1),

      new TurnToAmpTimed(driveSub, false),
      new LeaveAmpTimed()
    );
  }
}
