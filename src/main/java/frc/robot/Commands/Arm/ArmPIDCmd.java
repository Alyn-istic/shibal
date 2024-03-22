// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmPIDCmd extends Command {
  private ArmSubsystem armSub;
  private DoubleSupplier setpoint, tolerance;
  private PIDController controller;

  /**
   * PID command that manages the arm
   * @param armSub Arm Subsystem
   * @param setpoint The setpoint/angle that the arm should be at (degrees)
   * @param tolerance The tolerance for the PID controller
   */
  public ArmPIDCmd(
    ArmSubsystem armSub,
    // DoubleSupplier raiseP,
    // DoubleSupplier raiseI,
    // DoubleSupplier raiseD,
    // DoubleSupplier dropP,
    // DoubleSupplier dropI,
    // DoubleSupplier dropD,
    DoubleSupplier setpoint,
    DoubleSupplier tolerance
  ) {
    // System.out.println(armSub.getAngle());
    this.armSub = armSub;
    // this.raiseP = raiseP;
    // this.raiseI = raiseI;
    // this.raiseD = raiseD;
    // this.dropP = dropP;
    // this.dropI = dropI;
    // this.dropD = dropD;
    this.setpoint = setpoint;
    this.tolerance = tolerance;
    addRequirements(armSub);
  }


  @Override
  public void initialize() {
    controller = armSub.getController();
    controller.setTolerance(tolerance.getAsDouble());
    controller.setSetpoint(setpoint.getAsDouble());
  }

  @Override
  public void execute() {
    double speed = -controller.calculate(armSub.getAngle() % 360);
    // armSub.setMotor(speed);
    //System.out.println("Arm velocity recieved (experimenting, currently does nothing):" + armSub.getSensorVelocity());
    if ((speed < 0)) { // Raising
      armSub.setMotor(speed);
      controller.setP(ArmConstants.raiseP);
      controller.setI(ArmConstants.raiseI);
      controller.setD(ArmConstants.raiseD);
    } else if (speed > 0) { // Dropping
      if (!armSub.dropLimitSwitchHit()) {
        armSub.setMotor(speed);
      } else {
        armSub.setMotor(0);
      }
      controller.setP(ArmConstants.dropP);
      controller.setI(ArmConstants.dropI);
      controller.setD(ArmConstants.dropD);
    }

    controller.setSetpoint(setpoint.getAsDouble() + SmartDashboard.getNumber("Arm Setpoint Offset", 0));
    controller.setTolerance(tolerance.getAsDouble());

    armSub.updatePositionIndex(controller.getSetpoint());

    // Pushing number to SmartDashboard
    SmartDashboard.putNumber("Arm P", controller.getP());
    SmartDashboard.putNumber("Arm I", controller.getI());
    SmartDashboard.putNumber("Arm D", controller.getD());
    SmartDashboard.putNumber("Arm PID Output", controller.calculate(armSub.getAngle() % 360));
    SmartDashboard.putNumber("Arm PID Setpoint", setpoint.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    armSub.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /* This should probably not be set to controller.atSetpoint() or limit.getAsBoolean()
    Because, it would be nice if the arm would be able to go back to the setpoint on its own if something (like an collision) moved the arm a little.
    The motor speed is automatically set to 0 when the limit returns true, and the motors shouldn't be able to move when settled at the setpoint */
    return false;
    // return (controller.atSetpoint() || limit.getAsBoolean());
  }
}