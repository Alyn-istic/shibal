// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ClimberCmd;
import frc.robot.Commands.EmergencyStopCmd;
import frc.robot.Commands.Arm.ArmCmd;
import frc.robot.Commands.Arm.ArmPIDCmd;
import frc.robot.Commands.Autos.ExitZoneTimed;
import frc.robot.Commands.Autos.AutoLog;
import frc.robot.Commands.Autos.ScoreInAmpTimed;
// import frc.robot.Commands.Arm.LimitSwitchSimulation;
import frc.robot.Commands.Drivetrain.TankDriveCmd;
import frc.robot.Commands.IntakeShooter.IntakeCmd;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeShooterSubsystem;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Constants.ClimberConstants;

public class RobotContainer {
  // Initiating a ordinary Xbox Controller. Nothing special.
  private final XboxController driver = new XboxController(DriverConstants.driverPort);
  private final XboxController operator = new XboxController(DriverConstants.operatorPort);
  // Initiating a command Xbox Controller. This will allow us to map commands onto specific buttons.
  private final CommandXboxController commandDriver = new CommandXboxController(DriverConstants.driverPort);
  private final CommandXboxController commandOperator = new CommandXboxController(DriverConstants.operatorPort);

  // Initiating all the subsystems. We will need these in order to properly run commands.
  private final DrivetrainSubsystem driveSub = new DrivetrainSubsystem();
  private final ArmSubsystem armSub = new ArmSubsystem();
  private final IntakeShooterSubsystem intakeShooterSub = new IntakeShooterSubsystem();
  private final ClimberSubsystem climbSub = new ClimberSubsystem();

  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    // Telling the robot to run the TankDrive command when no other command is using the Drivetrain.
    driveSub.setDefaultCommand(
      new TankDriveCmd(
        driveSub,
        /** The following two lines are just getting the controller's left and right joysticks, and applying a deadzone to them.
         * This can all be configurated in Constants.java */
        () -> MathUtil.applyDeadband(driver.getRawAxis(DriverConstants.leftJoystickAxis), DriverConstants.joystickDeadband),
        () -> MathUtil.applyDeadband(driver.getRawAxis(DriverConstants.rightJoystickAxis), DriverConstants.joystickDeadband)
      )
    );

    armSub.setDefaultCommand(
      new ArmCmd(armSub,
        () -> -MathUtil.applyDeadband(operator.getRawAxis(DriverConstants.rightJoystickAxis) * ArmConstants.armManualSpeed, DriverConstants.joystickDeadband)
      )
    );

    SmartDashboard.putNumber("Arm P", ArmConstants.kP);
    SmartDashboard.putNumber("Arm I", ArmConstants.kI);
    SmartDashboard.putNumber("Arm D", ArmConstants.kD);
    // SmartDashboard.putNumber("Arm Setpoint", ArmConstants.shootInsideAngle);
    SmartDashboard.putNumber("Arm Clamp", ArmConstants.clamp);
    SmartDashboard.putNumber("Arm Setpoint Offset", ArmConstants.setpointOffset);

    autoChooser.setDefaultOption("NONE", "NONE");
    autoChooser.addOption("MOVE OUT OF ZONE", "MOVE_OUT_OF_ZONE");
    autoChooser.addOption("SCORE IN AMP (SENSORS)", "SCORE_IN_AMP_SENSORS");
    autoChooser.addOption("SCORE IN AMP (TIMED)", "SCORE_IN_AMP_TIMED");
    SmartDashboard.putData("Autonomous Routines", autoChooser);
    configureBindings();
  }

  // This is used to map commands to the Command Xbox driver.
  private void configureBindings() {
    commandDriver.x().onTrue(new EmergencyStopCmd());
    commandDriver.leftBumper().whileTrue(new IntakeCmd(intakeShooterSub, () -> 1));
    commandDriver.rightBumper().whileTrue(new IntakeCmd(intakeShooterSub, () -> -1));

    commandDriver.y().whileTrue(new ClimberCmd(climbSub, () -> ClimberConstants.climberSpeed)); // Extending Climber (This will depend on how arm works)
    commandDriver.b().whileTrue(new ClimberCmd(climbSub, () -> ClimberConstants.climberSpeed * -1)); // Retracting climber
    
    //Intake: Drop into intake angle.//
    commandDriver.povDown().whileTrue(new ArmPIDCmd(armSub, //
        // () -> ArmConstants.kP,
        // () -> ArmConstants.kI,
        // () -> ArmConstants.kD,
        () -> SmartDashboard.getNumber("Arm P", 0),
        () -> SmartDashboard.getNumber("Arm I", 0),
        () -> SmartDashboard.getNumber("Arm D", 0),
        () -> ArmConstants.intakeAngle,
        () -> ArmConstants.tolerance,
        () -> ArmConstants.clamp,
        () -> armSub.dropLimitSwitch()
    ));

    //Inside Angle for Intake: Raise into the perimeters of the robot, while ready for intake//
    commandDriver.povRight().whileTrue(new ArmPIDCmd(armSub,
        // () -> ArmConstants.kP,
        // () -> ArmConstants.kI,
        // () -> ArmConstants.kD,
        () -> SmartDashboard.getNumber("Arm P", 0),
        () -> SmartDashboard.getNumber("Arm I", 0),
        () -> SmartDashboard.getNumber("Arm D", 0),
        () -> ArmConstants.intakeInsideAngle,
        () -> ArmConstants.tolerance,
        () -> ArmConstants.clamp,
        () -> false
    ));

    //Shooter: Raise into shooting position for amp.//
    commandDriver.povUp().whileTrue(new ArmPIDCmd(armSub,
        // () -> ArmConstants.kP,
        // () -> ArmConstants.kI,
        // () -> ArmConstants.kD,
        () -> SmartDashboard.getNumber("Arm P", 0),
        () -> SmartDashboard.getNumber("Arm I", 0),
        () -> SmartDashboard.getNumber("Arm D", 0),
        () -> ArmConstants.shootAngle,
        () -> ArmConstants.tolerance,
        () -> ArmConstants.clamp,
        () -> armSub.raiseLimitSwitch()
    ));

    //Inside angle for Shooter: Raise into the perimeters of robot, while ready to downshoot into amp.//
    commandDriver.povLeft().whileTrue(new ArmPIDCmd(armSub,
        // () -> ArmConstants.kP,
        // () -> ArmConstants.kI,
        // () -> ArmConstants.kD,
        () -> SmartDashboard.getNumber("Arm P", 0),
        () -> SmartDashboard.getNumber("Arm I", 0),
        () -> SmartDashboard.getNumber("Arm D", 0),
        () -> ArmConstants.shootInsideAngle,
        () -> ArmConstants.clamp,  
        () -> ArmConstants.tolerance,
        () -> false
    ));

    //Source Intake: Intake from source.//
    commandDriver.a().whileTrue(new ArmPIDCmd(armSub,
        // () -> ArmConstants.kP,
        // () -> ArmConstants.kI,
        // () -> ArmConstants.kD,
        () -> SmartDashboard.getNumber("Arm P", 0),
        () -> SmartDashboard.getNumber("Arm I", 0),
        () -> SmartDashboard.getNumber("Arm D", 0),
        () -> ArmConstants.sourceIntakeAngle,
        () -> ArmConstants.tolerance,        
        () -> ArmConstants.clamp,
        () -> false
    ));

    // Operator commands
    commandOperator.x().onTrue(new EmergencyStopCmd());
    commandOperator.leftBumper().whileTrue(new IntakeCmd(intakeShooterSub, () -> 1));
    commandOperator.rightBumper().whileTrue(new IntakeCmd(intakeShooterSub, () -> -1));

    commandOperator.povUp().whileTrue(
      new RunCommand(
        () -> SmartDashboard.putNumber("Arm Setpoint Offset", SmartDashboard.getNumber("Arm Setpoint Offset", 0) + 0.1)
      )
    );
    commandOperator.povDown().whileTrue(
      new RunCommand(
        () -> SmartDashboard.putNumber("Arm Setpoint Offset", SmartDashboard.getNumber("Arm Setpoint Offset", 0) - 0.1)
      )
    );
  }

  public Command getAutonomousCommand() {
    switch (autoChooser.getSelected()) {
      case "MOVE_OUT_OF_ZONE": // Moves the robot out of the zone.
        return new ExitZoneTimed(driveSub); // Return the auto command that moves out of the zone
      case "SCORE_IN_AMP_SENSORS":
        return new AutoLog("This routine has not been set up yet."); // Returns the auto command that moves robot to amp, and shoots loaded note, using sensors.
      case "SCORE_IN_AMP_TIMED":
        return new ScoreInAmpTimed(driveSub, intakeShooterSub); // Returns the auto command that moves robot to amp, and shoots loaded note, using timers.
    }
    return new AutoLog("No auto selected.");
  }
}