// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

public class TankDriveVisionPIDCmd extends Command {
  // Create the necessary variables.
  private DrivetrainSubsystem driveSub;
  private VisionSubsystem visionSub;
  private DoubleSupplier driveTolerance; //driveP, driveI, driveD, turnP, turnI, turnD;
  private PIDController leftDriveController, rightDriveController;
  private BooleanSupplier periodicalUpdate, endSupplier;

  /**
   * Applies speed to the left and right motors using PID controllers based on left and right distances
   * @param driveSub The drivetrain subsystem
   * @param visionSub The vision subsystem
   * @param leftDriveSetpoint Returns the setpoint for the left encoders in meters
   * @param rightDriveSetpoint Returns the setpoint for the right encoders in meters
   * @param driveTolerance Returns the tolerance for the PID controllers
   * @param periodicalUpdate Returns if the controllers's gains should be periodically updated
   * @param end Returns true when the command should end
   */
  public TankDriveVisionPIDCmd(
    // The arguments (settings) that this command will accept.
    DrivetrainSubsystem driveSub,
    VisionSubsystem visionSub,
    DoubleSupplier driveTolerance, //supplied from robotcontainer
    BooleanSupplier periodicalUpdate,
    BooleanSupplier end
  ) {
    this.driveSub = driveSub;
    this.visionSub = visionSub;
    this.driveTolerance = driveTolerance;
    this.endSupplier = end;
    this.periodicalUpdate = periodicalUpdate;
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

    leftDriveController.setSetpoint(VisionConstants.cameraDistanceFromNote);
    rightDriveController.setSetpoint(VisionConstants.cameraDistanceFromNote);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = -leftDriveController.calculate(visionSub.getDistance()) * DrivetrainConstants.speed; // Dont know why this is negated
    double rightSpeed = -rightDriveController.calculate(visionSub.getDistance()) * DrivetrainConstants.speed;
  
    driveSub.tankDriveSpeed(
      (leftSpeed),
      (rightSpeed)
    );
    
    if (periodicalUpdate.getAsBoolean()){
      leftDriveController.setTolerance(driveTolerance.getAsDouble());
      rightDriveController.setTolerance(driveTolerance.getAsDouble());

      leftDriveController.setSetpoint(VisionConstants.cameraDistanceFromNote);
      rightDriveController.setSetpoint(VisionConstants.cameraDistanceFromNote);
    }
      

    // leftDriveController.setP(SmartDashboard.getNumber("P", 0));
    // leftDriveController.setI(SmartDashboard.getNumber("I", 0));
    // leftDriveController.setD(SmartDashboard.getNumber("D", 0));

    // rightDriveController.setP(SmartDashboard.getNumber("P", 0));
    // rightDriveController.setI(SmartDashboard.getNumber("I", 0));
    // rightDriveController.setD(SmartDashboard.getNumber("D", 0));

    // Pushing numbers onto SmartDashboard for debugging purposes.
    // SmartDashboard.putNumber("Drivetrain Left PID Output", leftSpeed);
    // SmartDashboard.putNumber("Drivetrain Right PID Output", rightSpeed);
    
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
