// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class TankDrivePIDCmd extends Command {
  // Create the necessary variables.
  private DrivetrainSubsystem driveSub;
  private DoubleSupplier leftDriveSetpoint, rightDriveSetpoint, angleSetpoint, driveTolerance, turnTolerance; //driveP, driveI, driveD, turnP, turnI, turnD;
  private PIDController leftDriveController, rightDriveController, turnController;

  public TankDrivePIDCmd(
    // The arguments (settings) that this command will accept.
    DrivetrainSubsystem driveSub,
    // DoubleSupplier driveP,
    // DoubleSupplier driveI,
    // DoubleSupplier driveD,
    // DoubleSupplier turnP,
    // DoubleSupplier turnI,
    // DoubleSupplier turnD,
    DoubleSupplier leftDriveSetpoint,
    DoubleSupplier rightDriveSetpoint,
    DoubleSupplier angleSetpoint,
    DoubleSupplier driveTolerance, //supplied from robotcontainer
    DoubleSupplier turnTolerance
  ) {
    this.driveSub = driveSub;
    // this.driveP = driveP;
    // this.driveI = driveI;
    // this.driveD = driveD;
    // this.turnP = turnP;
    // this.turnI = turnI;
    // this.turnD = turnD;
    
    this.leftDriveSetpoint = leftDriveSetpoint;
    this.rightDriveSetpoint = rightDriveSetpoint;
    this.angleSetpoint = angleSetpoint;
    this.driveTolerance = driveTolerance;
    this.turnTolerance = turnTolerance;
    addRequirements(driveSub);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftDriveController = driveSub.getLeftDriveController();
    rightDriveController = driveSub.getRightDriveController();
    turnController = driveSub.getTurnController();

    // driveController.setP(driveP.getAsDouble());
    // driveController.setI(driveI.getAsDouble());
    // driveController.setD(driveD.getAsDouble());
    leftDriveController.setP(DrivetrainConstants.driveP);
    leftDriveController.setI(DrivetrainConstants.driveI);
    leftDriveController.setD(DrivetrainConstants.driveD);

    rightDriveController.setP(DrivetrainConstants.driveP);
    rightDriveController.setI(DrivetrainConstants.driveI);
    rightDriveController.setD(DrivetrainConstants.driveD);

    turnController.enableContinuousInput(DrivetrainConstants.minAngle, DrivetrainConstants.maxAngle);
    // turnController.setP(turnP.getAsDouble());
    // turnController.setI(turnI.getAsDouble());
    // turnController.setD(turnD.getAsDouble());
    turnController.setP(DrivetrainConstants.turnP);
    turnController.setI(DrivetrainConstants.turnI);
    turnController.setD(DrivetrainConstants.turnD);


    leftDriveController.setTolerance(driveTolerance.getAsDouble());
    rightDriveController.setTolerance(driveTolerance.getAsDouble());
    turnController.setTolerance(turnTolerance.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leftDriveController.setSetpoint(leftDriveSetpoint.getAsDouble());
    rightDriveController.setSetpoint(rightDriveSetpoint.getAsDouble());
    turnController.setSetpoint(angleSetpoint.getAsDouble());

    double turn = turnController.calculate(driveSub.getGyroAngle() % 360);
    double leftSpeed = -leftDriveController.calculate(driveSub.getLeftDistance()) * DrivetrainConstants.speed;
    double rightSpeed = -rightDriveController.calculate(driveSub.getRightDistance()) * DrivetrainConstants.speed;
  
    driveSub.tankDriveSpeed(
      (leftSpeed + turn),
      (rightSpeed - turn)
    );

    // Pushing numbers onto SmartDashboard for debugging purposes.
    SmartDashboard.putNumber("Drivetrain Left PID Output", leftSpeed);
    SmartDashboard.putNumber("Drivetrain Right PID Output", rightSpeed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
