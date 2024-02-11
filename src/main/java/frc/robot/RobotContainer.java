// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Drivetrain.TankDriveCmd;
import frc.robot.Commands.Drivetrain.Routines.MoveOutOfZoneTimed;
import frc.robot.Commands.IntakeShooter.IntakeCmd;
import frc.robot.Commands.Routines.ScoreInAmpTimed;
import frc.robot.Constants.DriverConstants;
//import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeShooterSubsystem;

public class RobotContainer {
  // Initiating a ordinary Xbox Controller. Nothing special.
  private final XboxController controller = new XboxController(DriverConstants.port);
  // Initiating a command Xbox Controller. This will allow us to map commands onto specific buttons.
  private final CommandXboxController commandController = new CommandXboxController(DriverConstants.port);

  // Initiating all the subsystems. We will need these in order to properly run commands.
  private final DrivetrainSubsystem driveSub = new DrivetrainSubsystem();
  // private final ArmSubsystem armSub = new ArmSubsystem();
  private final IntakeShooterSubsystem intakeShooterSub = new IntakeShooterSubsystem();

  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    // Telling the robot to run the TankDrive command when no other command is using the Drivetrain.
    driveSub.setDefaultCommand(
      new TankDriveCmd(
        driveSub,
        /** The following two lines are just getting the controller's left and right joysticks, and applying a deadzone to them.
         * This can all be configurated in Constants.java */
        () -> controller.getRawAxis(DriverConstants.leftJoystickAxis),
        () -> controller.getRawAxis(DriverConstants.rightJoystickAxis)
      )
    );

    autoChooser.addOption("MOVE_OUT_OF_ZONE", "MOVE OUT OF ZONE");
    autoChooser.addOption("SCORE_IN_AMP_SENSORS", "SCORE IN AMP (SENSORS)");
    autoChooser.addOption("SCORE_IN_AMP_TIMED", "SCORE IN AMP (TIMED)");
    autoChooser.setDefaultOption("MOVE_OUT_OF_ZONE", "MOVE OUT OF ZONE");
    SmartDashboard.putData("Autonomous Routines", autoChooser);
    configureBindings();
  }

  // This is used to map commands to the Command Xbox Controller.
  private void configureBindings() {
    //commandController.a().whileTrue(new ScoreInAmpTimed(driveSub, intakeShooterSub));
    commandController.b().whileTrue(new IntakeCmd(intakeShooterSub, () -> 1));
  }

  public Command getAutonomousCommand() {
    switch (autoChooser.getSelected()) {
      case "MOVE_OUT_OF_ZONE":
        return new MoveOutOfZoneTimed(driveSub); // Return the auto command that moves out of the zone
      case "SCORE_IN_AMP_SENSORS":
        return null; // Returns the auto command that moves robot to amp, and shoots loaded note, using sensors.
      case "SCORE_IN_AMP_TIMED":
        return new ScoreInAmpTimed(driveSub, intakeShooterSub); // Returns the auto command that moves robot to amp, and shoots loaded note, using timers.
    }
    return null; 
  }
}