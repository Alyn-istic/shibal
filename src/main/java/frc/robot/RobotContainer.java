// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.EmergencyStopCmd;
import frc.robot.Commands.Arm.ArmManualCmd;
import frc.robot.Commands.Arm.ArmCommandSelector;
import frc.robot.Commands.Arm.ArmSetpointOffset;
import frc.robot.Commands.Arm.Autos.ArmIntake;
import frc.robot.Commands.Arm.Autos.ArmIntakePerimeter;
// import frc.robot.Commands.Arm.Autos.ArmIntakeSource;
import frc.robot.Commands.Arm.Autos.ArmShoot;
import frc.robot.Commands.Arm.Autos.ArmShootPerimeter;
import frc.robot.Commands.Climber.ClimberCmd;
// import frc.robot.Commands.Arm.LimitSwitchSimulation;
import frc.robot.Commands.Drivetrain.TankDriveCmd;
import frc.robot.Commands.Drivetrain.resetCmd;
import frc.robot.Commands.IntakeShooter.IntakeCmd;
import frc.robot.Commands.IntakeShooter.Test.intakeSeperateCmd;
import frc.robot.Commands.MainAutos.AutoLog;
import frc.robot.Commands.MainAutos.Sensor.ScoreInAmpSensor1;
import frc.robot.Commands.MainAutos.Timed.ExitZoneTimed;
import frc.robot.Commands.MainAutos.Timed.ScoreInAmpTimed.ScoreInAmpTimed1;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeShooterSubsystem;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
  // Initiating a ordinary Xbox Controller. Nothing special.
  private final XboxController driver = new XboxController(DriverConstants.driverPort);
  private final XboxController operator = new XboxController(DriverConstants.operatorPort);
  private final XboxController tester = new XboxController(DriverConstants.testerPort);
  // Initiating a command Xbox Controller. This will allow us to map commands onto specific buttons.
  private final CommandXboxController commandDriver = new CommandXboxController(DriverConstants.driverPort);
  private final CommandXboxController commandOperator = new CommandXboxController(DriverConstants.operatorPort);
  private final CommandXboxController commandTester = new CommandXboxController(DriverConstants.testerPort);

  // Initiating all the subsystems. We will need these in order to properly run commands.
  private final DrivetrainSubsystem driveSub = new DrivetrainSubsystem();
  public final LEDSubsystem led = new LEDSubsystem();
  private final ArmSubsystem armSub = new ArmSubsystem();
  private final IntakeShooterSubsystem intakeShooterSub = new IntakeShooterSubsystem();
  private final ClimberSubsystem climbSub = new ClimberSubsystem();

  // Stuff for ArmPID
  private final Command[] armPIDCommands = {
    new ArmIntake(armSub, ()-> false),
    new ArmIntakePerimeter(armSub, () -> false),
    new ArmShootPerimeter(armSub, () -> false),
    new ArmShoot(armSub, () -> false)
  };
  private final NetworkTableEntry armIndexEntry = NetworkTableInstance.getDefault().getEntry("ArmIndex");

  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    armIndexEntry.setInteger(-1);

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
      new ArmManualCmd(armSub,
        () -> -MathUtil.applyDeadband(operator.getRawAxis(DriverConstants.rightJoystickAxis) * ArmConstants.armManualSpeed, DriverConstants.joystickDeadband)
      )
    );

    SmartDashboard.putNumber("P", DrivetrainConstants.turnP);
    SmartDashboard.putNumber("I",DrivetrainConstants.turnI);
    SmartDashboard.putNumber("D", DrivetrainConstants.turnD);

    // SmartDashboard.putNumber("Arm Setpoint", ArmConstants.shootInsideAngle);
    // SmartDashboard.putNumber("Arm Clamp", ArmConstants.clamp);
    // SmartDashboard.putNumber("Arm Setpoint Offset", ArmConstants.setpointOffset);

    autoChooser.setDefaultOption("NONE", "NONE");
    autoChooser.addOption("MOVE OUT OF ZONE", "MOVE_OUT_OF_ZONE");
    autoChooser.addOption("SCORE IN AMP (SENSORS)", "SCORE_IN_AMP_SENSORS");
    autoChooser.addOption("SCORE IN AMP (TIMED)", "SCORE_IN_AMP_TIMED");
    SmartDashboard.putData("Autonomous Routines", autoChooser);
    configureBindings();
  }

  private void configureBindings() {

    ////////////////////////////////////////// Driver Controls //////////////////////////////////////////

    commandDriver.x().onTrue(new EmergencyStopCmd()); // E-stop

    // Using triggers to control intake speed
    commandDriver.leftTrigger().whileTrue(
      new IntakeCmd(
        intakeShooterSub, () -> MathUtil.applyDeadband(driver.getRawAxis(DriverConstants.leftTriggerAxis), DriverConstants.triggerDeadband)
      )
    );
    commandDriver.rightTrigger().whileTrue(
      new IntakeCmd(
        intakeShooterSub, () -> MathUtil.applyDeadband(-driver.getRawAxis(DriverConstants.rightTriggerAxis), DriverConstants.triggerDeadband)
      )
    );

    // Run the climber motors using y and b
    commandDriver.y().whileTrue(new ClimberCmd(climbSub, () -> ClimberConstants.climberSpeed)); // Extending Climber (This will depend on how arm works)
    commandDriver.b().whileTrue(new ClimberCmd(climbSub, () -> ClimberConstants.climberSpeed * -1)); // Retracting climber

    // Using left/right bumpers to jump between setpoints for PID
    commandDriver.leftBumper().onTrue(new ArmCommandSelector(armPIDCommands, () -> -1, armIndexEntry));
    commandDriver.rightBumper().onTrue(new ArmCommandSelector(armPIDCommands, () -> 1, armIndexEntry));

    commandDriver.a().onTrue(new resetCmd(driveSub));
    // //Intake: Drop into intake angle.//
    // commandDriver.povDown().whileTrue(new ArmIntake(armSub));

    // //Inside Angle for Intake: Raise into the perimeters of the robot, while ready for intake//
    // commandDriver.povRight().whileTrue(new ArmIntakePerimeter(armSub));

    // //Shooter: Raise into shooting position for amp.//
    // commandDriver.povUp().whileTrue(new ArmShoot(armSub));

    // //Inside angle for Shooter: Raise into the perimeters of robot, while ready to downshoot into amp.//
    // commandDriver.povLeft().whileTrue(new ArmShootPerimeter(armSub));

    // //Source Intake: Intake from source.//
    // commandDriver.a().whileTrue(new ArmIntakeSource(armSub));

    ////////////////////////////////////////// Operator Controls //////////////////////////////////////////

    commandOperator.x().onTrue(new EmergencyStopCmd()); // E-stop

    // Use triggers to control intake speed
    commandOperator.leftTrigger().whileTrue(
      new IntakeCmd(
        intakeShooterSub, () -> operator.getRawAxis(DriverConstants.leftTriggerAxis)
      )
    );
    commandOperator.rightTrigger().whileTrue(
      new IntakeCmd(
        intakeShooterSub, () -> -operator.getRawAxis(DriverConstants.rightTriggerAxis)
      )
    );

    // Use bumpers to offset the arm setpoints by increments of 3°
    commandOperator.rightBumper().onTrue(
      new ArmSetpointOffset(
        () -> 3
      )
    );
    commandOperator.leftBumper().onTrue(
      new ArmSetpointOffset(
        () -> -3
      )
    );

    commandOperator.a().and(commandOperator.b()).whileTrue(Commands.run(() -> driveSub.operatorReset(), driveSub));

    ////////////////////////////////////////// Arm Limits //////////////////////////////////////////

    // While the drop limit switch is pressed, reset arm position to intake angle, and reset setpoint offset to 0.
    new Trigger(() -> armSub.dropLimitSwitch()).whileTrue(
      new RunCommand(
        () -> armSub.setSensorPosition(armSub.toPosition(ArmConstants.intakeAngle))
      ).alongWith(
        new RunCommand(
          () -> SmartDashboard.putNumber("Arm Setpoint Offset", 0)
        )
      )
    );
    // While the raise limit switch is pressed, reset arm position to shoot angle, and reset setpoint offset to 0.
    new Trigger(() -> armSub.raiseLimitSwitch()).whileTrue(
      new RunCommand(
        () -> armSub.setSensorPosition(armSub.toPosition(ArmConstants.shootAngle))
      ).alongWith(
        new RunCommand(
          () -> SmartDashboard.putNumber("Arm Setpoint Offset", 0)
        )
      )
    );

    //////////////////////////////////////// Sys ID /////////////////////////////////////////////////////////
    // commandTester.y().whileTrue(driveSub.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // commandTester.a().whileTrue(driveSub.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // commandTester.povUp().whileTrue(driveSub.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // commandTester.povDown().whileTrue(driveSub.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    //////////////////////////////////////// Tester /////////////////////////////////////////////////////////
    commandTester.leftTrigger().whileTrue(
      new intakeSeperateCmd(
        intakeShooterSub, () -> MathUtil.applyDeadband(-tester.getRawAxis(DriverConstants.leftTriggerAxis), DriverConstants.triggerDeadband), 
        () -> MathUtil.applyDeadband(-tester.getRawAxis(DriverConstants.rightTriggerAxis), DriverConstants.triggerDeadband)
      )
    );
  }

  public Command getAutonomousCommand() {
    switch (autoChooser.getSelected()) {
      case "MOVE_OUT_OF_ZONE": // Moves the robot out of the zone.
        return new ExitZoneTimed(driveSub); // Return the auto command that moves out of the zone
      case "SCORE_IN_AMP_SENSORS":
        return new ScoreInAmpSensor1(driveSub, armSub, intakeShooterSub); // Returns the auto command that moves robot to amp, and shoots loaded note, using sensors.
      case "SCORE_IN_AMP_TIMED":
        return new ScoreInAmpTimed1(driveSub, intakeShooterSub, armSub); // Returns the auto command that moves robot to amp, and shoots loaded note, using timers.
    }
    return new AutoLog("No auto selected.");
  }
}