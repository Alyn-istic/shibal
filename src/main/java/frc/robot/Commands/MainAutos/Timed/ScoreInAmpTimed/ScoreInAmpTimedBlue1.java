// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.MainAutos.Timed.ScoreInAmpTimed;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.Arm.Autos.ArmIntake;
import frc.robot.Commands.Arm.Autos.ArmIntakePerimeter;
import frc.robot.Commands.Arm.Autos.ArmShoot;
import frc.robot.Commands.Arm.Autos.ArmZero;
import frc.robot.Commands.Drivetrain.TankDriveAutoCmd;
import frc.robot.Commands.Drivetrain.TankDriveCmd;
import frc.robot.Commands.Drivetrain.Autos.Timed.LeaveAmpTimed;
import frc.robot.Commands.Drivetrain.Autos.Timed.TurnToAmpTimed;
import frc.robot.Commands.IntakeShooter.IntakeCmd;
import frc.robot.Commands.IntakeShooter.Autos.DownShootAmpTimed;
import frc.robot.Constants.AutonomousConstants.MoveToAmpTimedConstants;
import frc.robot.Subsystems.ArmSubsystem;
//import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeShooterSubsystem;
import frc.robot.Subsystems.LEDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreInAmpTimedBlue1 extends SequentialCommandGroup {
  /** Creates a new AutonomousBackup. */
  public ScoreInAmpTimedBlue1(
    DrivetrainSubsystem driveSub,
    IntakeShooterSubsystem intakeShooterSub,
    LEDSubsystem led,
    ArmSubsystem armSub
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmZero(armSub).until(() -> armSub.dropLimitSwitch()),
      
      // new MoveToAmpTimedForward(driveSub),
      // new WaitCommand(1),

      // new TurnToAmpTimed(driveSub, -1),
      // new WaitCommand(1),

      new TankDriveCmd(driveSub, () -> 0.3, () -> 0.3).withTimeout(MoveToAmpTimedConstants.TIME_POS_1),// move forward

      new ArmShoot(armSub, () -> armSub.getController().atSetpoint()), // arm to shooting position
      new DownShootAmpTimed(intakeShooterSub), //shoot note

      new TankDriveCmd(driveSub, () -> -0.3, () -> -0.3).withTimeout(1), //rotate
      new WaitCommand(1),

      new TurnToAmpTimed(driveSub, 1),
      new ArmIntake(armSub, () -> armSub.getController().atSetpoint()),
      new ParallelCommandGroup(
        new LeaveAmpTimed(driveSub),
        new IntakeCmd(intakeShooterSub, () -> 1).withTimeout(1.5)
      ),
      new TankDriveAutoCmd(driveSub,
        () -> MoveToAmpTimedConstants.SPEED_TURN_TO_AMP,
        () -> -MoveToAmpTimedConstants.SPEED_TURN_TO_AMP,
        ()-> 0.3
      ),
      new LeaveAmpTimed(driveSub).alongWith(new ArmIntakePerimeter(armSub, () -> armSub.getController().atSetpoint())
)

      // new ArmShoot(armSub, () -> armSub.getController().atSetpoint()),
      // new TankDriveCmd(driveSub, () -> 0.3, () -> 0.3).withTimeout(1.5),
      // new WaitCommand(1),
      // new TankDriveAutoCmd(driveSub,
      //   () -> -MoveToAmpTimedConstants.SPEED_TURN_TO_AMP,
      //   () -> MoveToAmpTimedConstants.SPEED_TURN_TO_AMP,
      //   ()-> 1.1
      // ),
      // new TankDriveCmd(driveSub, () -> 0.3, () -> 0.3).withTimeout(1),
      // new DownShootAmpTimed(intakeShooterSub)
      
    );
  }
}
