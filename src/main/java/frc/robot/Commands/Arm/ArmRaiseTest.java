// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm.Routines;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Subsystems.ArmSubsystem;

public class ArmRaiseTest extends Command {
  /** Creates a new ArmRaiseTeleop. */
      private ArmSubsystem armSub;
      private DoubleSupplier raiseSpeed;
      private DoubleSupplier dropSpeed;

  public ArmRaiseTest(ArmSubsystem armSub, DoubleSupplier speed1, DoubleSupplier speed2) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSub = armSub;
    this.raiseSpeed = speed1;
    this.dropSpeed = speed2;
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double raise = raiseSpeed.getAsDouble();
    double drop = dropSpeed.getAsDouble();

    if (armSub.getAngle() < ArmConstants.raiseAngleMax){
      armSub.setMotor(raise);
    } else{
      armSub.setMotor(0);
    }

  if (armSub.getAngle() > 0){
    armSub.setMotor(-drop);
  } else{
    armSub.setMotor(0);
  }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSub.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
