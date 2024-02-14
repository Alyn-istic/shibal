// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmRaisePID extends PIDCommand {
  private ArmSubsystem armSub;
  /** Creates a new ArmRaise. */
  public ArmRaisePID(
    ArmSubsystem armSub
  ) {
    super(
        // The controller that the command will use
        new PIDController(ArmConstants.raiseP, ArmConstants.raiseI, ArmConstants.raiseD),
        // This should return the measurement
        () -> armSub.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> ArmConstants.raiseAngle,
        // This uses the output
        output -> {
          armSub.setMotor(MathUtil.clamp(output, -1, 1));
        });
    this.armSub = armSub;
    addRequirements(armSub);
    getController().setTolerance(ArmConstants.raiseTolerance);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (getController().atSetpoint()); // When the are is at the setpoint OR if the limit switches are hit.
  }
}