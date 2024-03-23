// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class TankDrivePIDCmd extends Command {
  // Create the necessary variables.
  private DrivetrainSubsystem driveSub;
  private DoubleSupplier leftDriveSetpoint, rightDriveSetpoint, driveTolerance; //driveP, driveI, driveD, turnP, turnI, turnD;
  private PIDController leftDriveController, rightDriveController;
  private BooleanSupplier periodicalUpdate, correctGyroError, endSupplier;

  private SlewRateLimiter leftLimiter, rightLimiter;

  private double initialGyroAngle;

  /**
   * Applies speed to the left and right motors using PID controllers based on left and right distances
   * @param driveSub The drivetrain subsystem
   * @param leftDriveSetpoint Returns the setpoint for the left encoders in meters
   * @param rightDriveSetpoint Returns the setpoint for the right encoders in meters
   * @param driveTolerance Returns the tolerance for the PID controllers
   * @param periodicalUpdate Returns if the controllers's gains should be periodically updated
   * @param correctGyroError If the chassis will try to creect using gyro
   * @param end Returns true when the command should end
   */
  public TankDrivePIDCmd(
    // The arguments (settings) that this command will accept.
    DrivetrainSubsystem driveSub,
    DoubleSupplier leftDriveSetpoint,
    DoubleSupplier rightDriveSetpoint,
    DoubleSupplier driveTolerance, //supplied from robotcontainer
    BooleanSupplier periodicalUpdate,
    BooleanSupplier correctGyroError,
    BooleanSupplier end
  ) {
    this.driveSub = driveSub;
    this.periodicalUpdate = periodicalUpdate;
    
    this.leftDriveSetpoint = leftDriveSetpoint;
    this.rightDriveSetpoint = rightDriveSetpoint;
    this.driveTolerance = driveTolerance;
    this.correctGyroError = correctGyroError;
    this.endSupplier = end;

    this.leftLimiter = driveSub.getLeftSlewRateLimiter();
    this.rightLimiter = driveSub.getRightSlewRateLimiter();

    this.initialGyroAngle = driveSub.getGyroAngle();

    addRequirements(driveSub);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftDriveController = driveSub.getLeftDriveController();
    rightDriveController = driveSub.getRightDriveController();

    leftDriveController.setP(DrivetrainConstants.driveP);
    leftDriveController.setI(DrivetrainConstants.driveI);
    leftDriveController.setD(DrivetrainConstants.driveD);

    rightDriveController.setP(DrivetrainConstants.driveP);
    rightDriveController.setI(DrivetrainConstants.driveI);
    rightDriveController.setD(DrivetrainConstants.driveD);

    leftDriveController.setTolerance(driveTolerance.getAsDouble());
    rightDriveController.setTolerance(driveTolerance.getAsDouble());

    leftDriveController.setSetpoint(leftDriveSetpoint.getAsDouble());
    rightDriveController.setSetpoint(rightDriveSetpoint.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double gyroError = (driveSub.getGyroAngle() - initialGyroAngle);
    gyroError = Math.signum(gyroError) * (Math.abs(gyroError) % 360);

    double gyroErrorCorrection = gyroError * SmartDashboard.getNumber("Gyro P", DrivetrainConstants.gyroErrorCorrectionFactor);

    double leftSpeed = leftDriveController.calculate(driveSub.getLeftDistance());
    double rightSpeed = rightDriveController.calculate(driveSub.getRightDistance());

    if (correctGyroError.getAsBoolean()) {
      leftSpeed += (gyroErrorCorrection);
      rightSpeed += (-gyroErrorCorrection);
    }
  
    driveSub.tankDriveSpeed(
      // leftLimiter.calculate(leftSpeed),
      // rightLimiter.calculate(rightSpeed)
      leftSpeed,
      rightSpeed
    );

    if (periodicalUpdate.getAsBoolean()) {
      leftDriveController.setTolerance(driveTolerance.getAsDouble());
      rightDriveController.setTolerance(driveTolerance.getAsDouble());

      leftDriveController.setSetpoint(leftDriveSetpoint.getAsDouble());
      rightDriveController.setSetpoint(rightDriveSetpoint.getAsDouble());
    }

    leftDriveController.setP(SmartDashboard.getNumber("P", 0));
    leftDriveController.setI(SmartDashboard.getNumber("I", 0));
    leftDriveController.setD(SmartDashboard.getNumber("D", 0));

    rightDriveController.setP(SmartDashboard.getNumber("P", 0));
    rightDriveController.setI(SmartDashboard.getNumber("I", 0));
    rightDriveController.setD(SmartDashboard.getNumber("D", 0));

    // Pushing numbers onto SmartDashboard for debugging purposes.
    SmartDashboard.putNumber("Drivetrain Left PID Output", leftSpeed);
    SmartDashboard.putNumber("Drivetrain Right PID Output", rightSpeed);
    SmartDashboard.putNumber("Drivetrain Left PID Setpoint", leftDriveController.getSetpoint());
    SmartDashboard.putNumber("Drivetrain Right PID Setpoint", rightDriveController.getSetpoint());
    SmartDashboard.putNumber("Drivetrain Gyro Error", gyroError);
    SmartDashboard.putNumber("Drivetrain Gyro Error Correction", gyroErrorCorrection);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endSupplier.getAsBoolean();
  }
}
