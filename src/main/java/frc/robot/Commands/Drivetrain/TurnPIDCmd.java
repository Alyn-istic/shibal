// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class TurnPIDCmd extends Command {
  // Create the necessary variables.
  private DrivetrainSubsystem driveSub;
  private DoubleSupplier angleSetpoint, turnTolerance; //driveP, driveI, driveD, turnP, turnI, turnD;
  private PIDController turnController;
  private BooleanSupplier periodicalUpdate, endSupplier;

  /**
   * Applies speed to the left and right motors using a PID controller based on gyro angle
   * @param driveSub The drivetrain subsystem
   * @param angleSetpoint Returns the setpoint for the gyro in degrees
   * @param turnTolerance Returns the tolerance for the PID controller
   * @param periodicalUpdate Returns if the controllers's gains should be periodically updated
   * @param end Returns true when the command should end
   */
  public TurnPIDCmd(
    // The arguments (settings) that this command will accept.
    DrivetrainSubsystem driveSub,
    DoubleSupplier angleSetpoint,
    DoubleSupplier turnTolerance,
    BooleanSupplier periodicalUpdate,
    BooleanSupplier end
  ) {
    this.driveSub = driveSub;
    this.periodicalUpdate = periodicalUpdate;
    this.angleSetpoint = angleSetpoint;
    this.turnTolerance = turnTolerance;
    this.endSupplier = end;
    addRequirements(driveSub);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController = driveSub.getTurnController();

    turnController.enableContinuousInput(DrivetrainConstants.minAngle, DrivetrainConstants.maxAngle);
    
    turnController.setP(DrivetrainConstants.turnP);
    turnController.setI(DrivetrainConstants.turnI);
    turnController.setD(DrivetrainConstants.turnD);
    turnController.setTolerance(turnTolerance.getAsDouble());
    turnController.setSetpoint(angleSetpoint.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turn = turnController.calculate(driveSub.getGyroAngle() % 360);
  
    driveSub.tankDriveSpeed(
      MathUtil.clamp(- turn, -0.5, 0.5),
      MathUtil.clamp(+ turn, -0.5, 0.5)
    );

    if (periodicalUpdate.getAsBoolean()) {
      turnController.setTolerance(turnTolerance.getAsDouble());
      turnController.setSetpoint(angleSetpoint.getAsDouble());  
    }

    // turnController.setP(SmartDashboard.getNumber("P", 0));
    // turnController.setI(SmartDashboard.getNumber("I", 0));
    // turnController.setD(SmartDashboard.getNumber("D", 0));

    // Pushing numbers onto SmartDashboard for debugging purposes.  
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
