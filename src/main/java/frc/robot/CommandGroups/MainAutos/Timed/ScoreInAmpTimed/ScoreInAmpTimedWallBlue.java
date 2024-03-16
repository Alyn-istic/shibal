// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.MainAutos.Timed.ScoreInAmpTimed;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutonomousConstants.MoveToAmpTimedConstants;
import frc.robot.CommandGroups.ArmAutos.ArmIntakePerimeter;
import frc.robot.CommandGroups.ArmAutos.ArmShoot;
import frc.robot.CommandGroups.ArmAutos.ArmZero;
import frc.robot.CommandGroups.DrivetrainAutos.Timed.TurnToAmpTimed;
import frc.robot.CommandGroups.IntakeShooterAutos.DownShootAmpTimed;
import frc.robot.Commands.Drivetrain.TankDriveCmd;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeShooterSubsystem;
import frc.robot.Subsystems.LEDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreInAmpTimedWallBlue extends SequentialCommandGroup {
  /** Creates a new ScoreInAmpTimedWallBlue. */
  public ScoreInAmpTimedWallBlue(   
    DrivetrainSubsystem driveSub,
    IntakeShooterSubsystem intakeShooterSub,
    LEDSubsystem led,
    ArmSubsystem armSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmZero(armSub).until(() -> armSub.dropLimitSwitch()),
      
      // new MoveToAmpTimedForward(driveSub),
      // new WaitCommand(1),

      // new TurnToAmpTimed(driveSub, -1),
      // new WaitCommand(1),

      new TankDriveCmd(driveSub, () -> 0.4, () -> 0.4).withTimeout(MoveToAmpTimedConstants.TIME_POS_1),// move forward

      new ArmShoot(armSub, () -> armSub.getController().atSetpoint()), // arm to shooting position
      new DownShootAmpTimed(intakeShooterSub), //shoot note
      new TankDriveCmd(driveSub, () -> -0.3, () -> -0.3).withTimeout(0.2),//move back
      new TurnToAmpTimed(driveSub, -1),
      new ArmIntakePerimeter(armSub, () -> armSub.getController().atSetpoint()),
      new TankDriveCmd(driveSub,() -> 0.3, () -> 0.3).withTimeout(3)
    );
  }
}
