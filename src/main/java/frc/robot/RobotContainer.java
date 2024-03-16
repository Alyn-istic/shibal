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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CommandGroups.ArmAutos.ArmIntake;
import frc.robot.CommandGroups.ArmAutos.ArmIntakePerimeter;
import frc.robot.CommandGroups.ArmAutos.ArmIntakeSource;
import frc.robot.CommandGroups.ArmAutos.ArmShoot;
import frc.robot.CommandGroups.ArmAutos.ArmShootPerimeter;
import frc.robot.CommandGroups.DrivetrainAutos.Sensor.MoveOutOfZoneSensor;
import frc.robot.CommandGroups.DrivetrainAutos.Timed.MoveOutOfZoneTimed.MoveOutOfZoneTimed1;
import frc.robot.CommandGroups.MainAutos.AutoLog;
import frc.robot.CommandGroups.MainAutos.Sensor.ExitZoneSensor;
import frc.robot.CommandGroups.MainAutos.Sensor.ScoreInAmpSensor1;
import frc.robot.CommandGroups.MainAutos.Timed.ExitZoneTimed;
import frc.robot.CommandGroups.MainAutos.Timed.ScoreInAmpTimed.ScoreInAmpTimedBlue1;
import frc.robot.CommandGroups.MainAutos.Timed.ScoreInAmpTimed.ScoreInAmpTimedBlue2;
import frc.robot.CommandGroups.MainAutos.Timed.ScoreInAmpTimed.ScoreInAmpTimedBlue3;
import frc.robot.CommandGroups.MainAutos.Timed.ScoreInAmpTimed.ScoreInAmpTimedOnly;
import frc.robot.CommandGroups.MainAutos.Timed.ScoreInAmpTimed.ScoreInAmpTimedRed1;
import frc.robot.CommandGroups.MainAutos.Timed.ScoreInAmpTimed.ScoreInAmpTimedRed2;
import frc.robot.CommandGroups.MainAutos.Timed.ScoreInAmpTimed.ScoreInAmpTimedRed3;
import frc.robot.CommandGroups.MainAutos.Timed.ScoreInAmpTimed.ScoreInAmpTimedWallBlue;
import frc.robot.CommandGroups.MainAutos.Timed.ScoreInAmpTimed.ScoreInAmpTimedWallRed;
import frc.robot.Commands.EmergencyStopCmd;
import frc.robot.Commands.Arm.ArmManualCmd;
import frc.robot.Commands.Arm.ArmCommandSelector;
import frc.robot.Commands.Arm.ArmSetpointOffset;
import frc.robot.Commands.Climber.ClimberCmd;
// import frc.robot.Commands.Arm.LimitSwitchSimulation;
import frc.robot.Commands.Drivetrain.TankDriveCmd;
import frc.robot.Commands.IntakeShooter.IntakeCmd;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.CameraSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeShooterSubsystem;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Constants.ClimberConstants;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
  // Initiating a ordinary Xbox Controller. Nothing special.
  private final XboxController driver = new XboxController(DriverConstants.driverPort);
  private final XboxController operator = new XboxController(DriverConstants.operatorPort);
  //private final XboxController tester = new XboxController(DriverConstants.testerPort);
  
  // Initiating a command Xbox Controller. This will allow us to map commands onto specific buttons.
  private final CommandXboxController commandDriver = new CommandXboxController(DriverConstants.driverPort);
  private final CommandXboxController commandOperator = new CommandXboxController(DriverConstants.operatorPort);
  //private final CommandXboxController commandTester = new CommandXboxController(DriverConstants.testerPort);

  // Initiating all the subsystems. We will need these in order to properly run commands.
  private final DrivetrainSubsystem driveSub = new DrivetrainSubsystem();
  public final LEDSubsystem led = new LEDSubsystem();
  private final ArmSubsystem armSub = new ArmSubsystem();
  private final IntakeShooterSubsystem intakeShooterSub = new IntakeShooterSubsystem();
  private final ClimberSubsystem climbSub = new ClimberSubsystem();
  //private final CameraSubsystem camSub = new CameraSubsystem();

  // Stuff for ArmPID
  private final Command[] armPIDCommands = {
    new ArmIntake(armSub, ()-> false),
    new ArmIntakePerimeter(armSub, () -> false),
    //new ArmIntakeSource(armSub, () -> false),
    new ArmShootPerimeter(armSub, () -> false),
    new ArmShoot(armSub, () -> false)
  };
  private final NetworkTableEntry armIndexEntry = NetworkTableInstance.getDefault().getEntry("ArmIndex");

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

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
    // led.setDefaultCommand(
    //   Commands.run(() -> led.setPresetGreen(), led)se
    // );

    // SmartDashboard.putNumber("P", DrivetrainConstants.turnP);
    // SmartDashboard.putNumber("I",DrivetrainConstants.turnI);
    // SmartDashboard.putNumber("D", DrivetrainConstants.turnD);

    // SmartDashboard.putNumber("Arm Setpoint", ArmConstants.shootInsideAngle);
    // SmartDashboard.putNumber("Arm Clamp", ArmConstants.clamp);
    // SmartDashboard.putNumber("Arm Setpoint Offset", ArmConstants.setpointOffset);

    autoChooser.setDefaultOption("NONE", new AutoLog("No auto selected."));
    autoChooser.addOption("MOVE OUT OF ZONE (TIMED)", new ExitZoneTimed(driveSub, armSub));
    autoChooser.addOption("MOVE OUT OF ZONE (SENSOR)", new ExitZoneSensor(driveSub, armSub));
    //autoChooser.addOption("SCORE IN AMP (SENSORS)", new ScoreInAmpSensor1(driveSub, armSub, intakeShooterSub, led));
    autoChooser.addOption("SCORE IN AMP ONLY (TIMED)", new ScoreInAmpTimedOnly(driveSub, intakeShooterSub, led, armSub));
    autoChooser.addOption("SCORE IN AMP 1 BLUE (TIMED)", new ScoreInAmpTimedBlue1(driveSub, intakeShooterSub, led, armSub));
    autoChooser.addOption("SCORE IN AMP 2 BLUE (TIMED)", new ScoreInAmpTimedBlue2(driveSub, intakeShooterSub, led, armSub));
    autoChooser.addOption("SCORE IN AMP 3 BLUE (TIMED)", new ScoreInAmpTimedBlue3(driveSub, intakeShooterSub, led, armSub));
    autoChooser.addOption("SCORE IN AMP 1 RED (TIMED)", new ScoreInAmpTimedRed1(driveSub, intakeShooterSub, led, armSub));
    autoChooser.addOption("SCORE IN AMP 2 RED (TIMED)", new ScoreInAmpTimedRed2(driveSub, intakeShooterSub, led, armSub));
    autoChooser.addOption("SCORE IN AMP 3 RED (TIMED)", new ScoreInAmpTimedRed3(driveSub, intakeShooterSub, led, armSub));
    autoChooser.addOption("SCORE IN AMP 3 Blue (TIMED)-new", new ScoreInAmpTimedWallBlue(driveSub, intakeShooterSub, led, armSub));
    autoChooser.addOption("SCORE IN AMP 3 RED (TIMED)", new ScoreInAmpTimedWallRed(driveSub, intakeShooterSub, led, armSub));

    // autoChooser.addOption("PATH TEST 0", driveSub.testPath0());
    // autoChooser.addOption("PATH TEST 1", driveSub.testPath1());
    // autoChooser.addOption("PATH TEST 2", driveSub.testPath2());
    // autoChooser.addOption("AUTO 1", driveSub.testAuto1());
    SmartDashboard.putData("Autonomous Routines", autoChooser);
    //SmartDashboard.put("Camera stream", camSub.getOutputStream());
    configureBindings();
  }

  private void configureBindings() {

    ////////////////////////////////////////// Driver Controls //////////////////////////////////////////

    //commandDriver.x().onTrue(new EmergencyStopCmd(driveSub, armSub, intakeShooterSub)); // E-stop

    // Using triggers to control intake speed
    commandDriver.leftTrigger().whileTrue(
      new IntakeCmd(
        intakeShooterSub, () -> MathUtil.applyDeadband(driver.getRawAxis(DriverConstants.leftTriggerAxis), DriverConstants.triggerDeadband)
      )
      //.alongWith(Commands.run(()->led.setPresetGold(), led))
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

    commandOperator.x().onTrue(new EmergencyStopCmd(driveSub, armSub, intakeShooterSub)); // E-stop

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

    commandOperator.povUp().or(commandOperator.povUpLeft()).or(commandOperator.povUpRight()).whileTrue(
      new ClimberCmd(climbSub, () -> ClimberConstants.climberSpeed));
    commandOperator.povDown().or(commandOperator.povDownLeft()).or(commandOperator.povDownRight()).whileTrue(
      new ClimberCmd(climbSub, () -> -ClimberConstants.climberSpeed));


    ////////////////////////////////////////// Arm Limits //////////////////////////////////////////

    // While the drop limit switch is pressed, reset arm position to intake angle, and reset setpoint offset to 0.
    new Trigger(() -> armSub.dropLimitSwitchHit()).whileTrue(
      new RunCommand(
        () -> armSub.setSensorPosition(armSub.toPosition(ArmConstants.intakeAngle))
      ).alongWith(
        new RunCommand(
          () -> SmartDashboard.putNumber("Arm Setpoint Offset", 0)
        )
      )
    );
    // While the raise limit switch is pressed, reset arm position to shoot angle, and reset setpoint offset to 0.
    new Trigger(() -> armSub.raiseLimitSwitchHit()).whileTrue(
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
    // commandTester.leftTrigger().whileTrue(
    //   new intakeSeperateCmd(
    //     intakeShooterSub, () -> MathUtil.applyDeadband(-tester.getRawAxis(DriverConstants.leftTriggerAxis), DriverConstants.triggerDeadband), 
    //     () -> MathUtil.applyDeadband(-tester.getRawAxis(DriverConstants.rightTriggerAxis), DriverConstants.triggerDeadband)
    //   )
    // );

  }

  public Command getAutonomousCommand() {
    return new WaitCommand(AutonomousConstants.waitBeforeExecRoutine).andThen(autoChooser.getSelected());
  }
}