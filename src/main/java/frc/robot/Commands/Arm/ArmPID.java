// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmPID extends PIDCommand {
  /** Creates a new ArmRaise. */
  public ArmPID(
    ArmSubsystem armSub,
    DoubleSupplier kP,
    DoubleSupplier kI,
    DoubleSupplier kD,
    DoubleSupplier setpoint,
    DoubleSupplier tolerance
  ) {
    super(
        // The controller that the command will use
        new PIDController(kP.getAsDouble(), kP.getAsDouble(), kP.getAsDouble()),
        // This should return the measurement
        () -> armSub.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint.getAsDouble(),
        // This uses the output
        output -> {
          armSub.setMotor(MathUtil.clamp(output, -1, 1));
        });
    getController(tolerance.getAsDouble());
    addRequirements(armSub);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (getController().atSetpoint()); // When the are is at the setpoint OR if the limit switches are hit.
  }
}