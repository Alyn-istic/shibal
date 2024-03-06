// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain.Autos.Timed;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Drivetrain.TankDriveAutoCmd;
import frc.robot.Commands.MainAutos.AutoLog;
import frc.robot.Constants.AutonomousConstants.MoveToAmpTimedConstants;
import frc.robot.Subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAmpTimed extends SequentialCommandGroup {
  /** Creates a new AutonomousBackup. */
  public TurnToAmpTimed(
    DrivetrainSubsystem driveSub, 
    boolean turningToAmp //1:turn to amp, -1: turn to the front
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    int dir;
    if(turningToAmp == true){
      dir = 1;
    }else{
      dir = -1;
    }
    addCommands(
      new AutoLog("Turning back-side towards the amp"),
      new TankDriveAutoCmd(driveSub,
        () -> dir*MoveToAmpTimedConstants.SPEED_TURN_TO_AMP,
        () -> dir*-MoveToAmpTimedConstants.SPEED_TURN_TO_AMP,
        ()-> MoveToAmpTimedConstants.TIME_TURN_TO_AMP
      ).withTimeout(MoveToAmpTimedConstants.TIME_TURN_TO_AMP * 2)
    );
  }
}