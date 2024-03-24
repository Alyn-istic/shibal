// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EmergencyStopCmd extends InstantCommand {
  private DrivetrainSubsystem driveSub;
  private ArmSubsystem armSub;
  private IntakeShooterSubsystem intakeSub;

  /**
   * This command cancels all ongoing commands, aswell as stop all motors.
   * @param driveSub Drivetrain subsystem
   * @param armSub Arm subsystem
   * @param intakeSub Intake subsystem
   */
  public EmergencyStopCmd(
    DrivetrainSubsystem driveSub,
    ArmSubsystem armSub,
    IntakeShooterSubsystem intakeSub
  ) {
    this.driveSub = driveSub;
    this.armSub = armSub;
    this.intakeSub = intakeSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandScheduler.getInstance().cancelAll();

    // Stopping motors
    driveSub.tankDriveSpeed(0, 0);
    armSub.setMotor(0);
    intakeSub.setMotors(0);

    System.out.println("EmergencyStopCmd - Human controller stopped all running commands.");

  }
}