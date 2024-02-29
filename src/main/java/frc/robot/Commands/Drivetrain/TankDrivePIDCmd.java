// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class TankDrivePIDCmd extends Command {
  // Create the necessary variables.
  private DrivetrainSubsystem driveSub;
  private DoubleSupplier setpoint, tolerance, driveP, driveI, driveD;
  private PIDController controller;

  /** Creates a new TankDriveCmd. */
  public TankDrivePIDCmd(
    // The arguments (settings) that this command will accept.
    DrivetrainSubsystem driveSub,
    DoubleSupplier driveP, DoubleSupplier driveI, DoubleSupplier driveD, DoubleSupplier setpoint, DoubleSupplier tolerance //supplied from robotcontainer
  ) {
    this.driveSub = driveSub;
    this.driveP = driveP;
    this.driveI = driveI;
    this.driveD = driveD;
    this.setpoint = setpoint;
    this.tolerance = tolerance;
    addRequirements(driveSub);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setP(driveP.getAsDouble());
    controller.setI(driveI.getAsDouble());
    controller.setD(driveD.getAsDouble());

    controller.setTolerance(tolerance.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controller.setSetpoint(setpoint.getAsDouble());
    double speed = controller.calculate(driveSub.getLeftDistance());
  
    driveSub.tankDriveSpeed(speed, speed);
    // Calling the "tankDrive" function in the DrivetrainSubsystem.java file.

    // Pushing numbers onto SmartDashboard for debugging purposes.
    SmartDashboard.putNumber("Drivetrain Straight PID Output", speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
