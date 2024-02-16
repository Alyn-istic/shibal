// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.EmergencyStopCmd;
import frc.robot.Commands.Arm.ArmCmd;
import frc.robot.Commands.Arm.ArmPID;
import frc.robot.Commands.Drivetrain.TankDriveCmd;
import frc.robot.Commands.IntakeShooter.IntakeCmd;
import frc.robot.Commands.Routines.ExitZoneTimed;
import frc.robot.Commands.Routines.RoutineLog;
import frc.robot.Commands.Routines.ScoreInAmpTimed;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeShooterSubsystem;

public class RobotContainer {
  // Initiating a ordinary Xbox Controller. Nothing special.
  private final XboxController controller = new XboxController(DriverConstants.port);
  // Initiating a command Xbox Controller. This will allow us to map commands onto specific buttons.
  private final CommandXboxController commandController = new CommandXboxController(DriverConstants.port);

  // Initiating all the subsystems. We will need these in order to properly run commands.
  private final DrivetrainSubsystem driveSub = new DrivetrainSubsystem();
  private final ArmSubsystem armSub = new ArmSubsystem();
  private final IntakeShooterSubsystem intakeShooterSub = new IntakeShooterSubsystem();

  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    // Telling the robot to run the TankDrive command when no other command is using the Drivetrain.
    driveSub.setDefaultCommand(
      new TankDriveCmd(
        driveSub,
        /** The following two lines are just getting the controller's left and right joysticks, and applying a deadzone to them.
         * This can all be configurated in Constants.java */
        () -> MathUtil.applyDeadband(controller.getRawAxis(DriverConstants.leftJoystickAxis), DriverConstants.joystickDeadband),
        () -> MathUtil.applyDeadband(controller.getRawAxis(DriverConstants.rightJoystickAxis), DriverConstants.joystickDeadband)
      )
    );

    // armSub.setDefaultCommand(
    //   new ArmCmd(
    //     armSub,
    //     () -> MathUtil.applyDeadband(controller.getRawAxis(DriverConstants.rightTriggerAxis) * 0.25, DriverConstants.triggerDeadband),
    //     () -> MathUtil.applyDeadband(controller.getRawAxis(DriverConstants.leftTriggerAxis) * 0.25, DriverConstants.triggerDeadband)
    //   )
    // );
    
    autoChooser.setDefaultOption("NONE", "NONE");
    autoChooser.addOption("MOVE OUT OF ZONE", "MOVE_OUT_OF_ZONE");
    autoChooser.addOption("SCORE IN AMP (SENSORS)", "SCORE_IN_AMP_SENSORS");
    autoChooser.addOption("SCORE IN AMP (TIMED)", "SCORE_IN_AMP_TIMED");
    SmartDashboard.putData("Autonomous Routines", autoChooser);
    configureBindings();
  }

  // This is used to map commands to the Command Xbox Controller.
  private void configureBindings() {
    commandController.x().onTrue(new EmergencyStopCmd());
    commandController.leftBumper().whileTrue(new IntakeCmd(intakeShooterSub, () -> 1));
    commandController.rightBumper().whileTrue(new IntakeCmd(intakeShooterSub, () -> -1));
    // commandController.povDown().onTrue(new ArmPID(armSub, // When the POV's down button is pressed, the arm goes into intake position
    //     () -> ArmConstants.kP,
    //     () -> ArmConstants.kI,
    //     () -> ArmConstants.kD,
    //     () -> ArmConstants.intakeAngle,
    //     () -> ArmConstants.tolerance
    // ));
    commandController.a().onTrue(new ArmPID(armSub, // When the POV's left or right buttons are pressed, the arm goes back inside the perimeters of the bumpers
        () -> ArmConstants.kP,
        () -> ArmConstants.kI,
        () -> ArmConstants.kD,
        () -> ArmConstants.insideAngle,
        () -> ArmConstants.tolerance
    ));
    // commandController.povUp().onTrue(new ArmPID(armSub, // When the POV's up button is pressed, the arm goes into shooting position.
    //     () -> ArmConstants.kP,
    //     () -> ArmConstants.kI,
    //     () -> ArmConstants.kD,
    //     () -> ArmConstants.shootAngle,
    //     () -> ArmConstants.tolerance
    // ));
  }

  public Command getAutonomousCommand() {
    switch (autoChooser.getSelected()) {
      case "MOVE_OUT_OF_ZONE": // Moves the robot out of the zone.
        return new ExitZoneTimed(driveSub); // Return the auto command that moves out of the zone
      case "SCORE_IN_AMP_SENSORS":
        return new RoutineLog("This routine has not been set up yet."); // Returns the auto command that moves robot to amp, and shoots loaded note, using sensors.
      case "SCORE_IN_AMP_TIMED":
        return new ScoreInAmpTimed(driveSub, intakeShooterSub); // Returns the auto command that moves robot to amp, and shoots loaded note, using timers.
    }
    return new RoutineLog("No auto selected.");
  }
}