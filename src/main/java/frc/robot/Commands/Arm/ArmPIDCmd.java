// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmPIDCmd extends Command {
  private ArmSubsystem armSub;
  private DoubleSupplier kP, kI, kD, setpoint, tolerance;
  private BooleanSupplier limit;
  private PIDController controller;
  /** Creates a new ArmRaise. */
  public ArmPIDCmd(
    ArmSubsystem armSub,
    DoubleSupplier kP,
    DoubleSupplier kI,
    DoubleSupplier kD,
    DoubleSupplier setpoint,
    DoubleSupplier tolerance,
    BooleanSupplier limit
  ) {
    // System.out.println(armSub.getAngle());
    this.armSub = armSub;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.setpoint = setpoint;
    this.tolerance = tolerance;
    this.limit = limit;
    addRequirements(armSub);
  }

  @Override
  public void initialize() {
    controller = new PIDController(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble());
    controller.setTolerance(tolerance.getAsDouble());
    controller.setSetpoint(setpoint.getAsDouble());
  }

  @Override
  public void execute() {
    // Applying the output to the arm
    armSub.setMotor(limit.getAsBoolean() ? 0 : MathUtil.clamp(controller.calculate(armSub.getAngle() % 360), -1, 1)); // If limit = true, then speed is set to 0. Else, speed is set to clamped output.

    // Updating the PID valuess
    controller.setP(kP.getAsDouble());
    controller.setI(kI.getAsDouble());
    controller.setD(kD.getAsDouble());

    // Pushing number to SmartDashboard
    SmartDashboard.putNumber("Arm PID Output", controller.calculate(armSub.getAngle() % 360));
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

// ======= Sadra's stuff
//  // Copyright (c) FIRST and other WPILib contributors.
//  // Open Source Software; you can modify and/or share it under the terms of
//  // the WPILib BSD license file in the root directory of this project.

//  package frc.robot.Commands.Arm;

//  import edu.wpi.first.math.MathUtil;
//  import edu.wpi.first.math.controller.PIDController;
//  import edu.wpi.first.wpilibj2.command.PIDCommand;
//  import frc.robot.Constants.ArmConstants;
//  import frc.robot.Subsystems.ArmSubsystem;
//  import java.util.function.DoubleSupplier;

//  // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
//  // information, see:
//  // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
//  public class ArmPID extends PIDCommand {
//    private ArmSubsystem armSub;
//    private DoubleSupplier Kp,Ki,Kd;
//    private DoubleSupplier setpoint;
//    private DoubleSupplier tolerance;
//    /** Creates a new ArmRaise. */
//    public ArmPID(
//      ArmSubsystem armSub,
//      DoubleSupplier Kp,
//      DoubleSupplier Ki,
//      DoubleSupplier Kd,
//      DoubleSupplier setpoint,
//      DoubleSupplier tolerance
//    ) {
//      super(
//          // The controller that the command will use
//          new PIDController(Kp.getAsDouble(), Ki.getAsDouble(), Kd.getAsDouble()),
//          // This should return the measurement
//          () -> armSub.getAngle(),
//          // This should return the setpoint (can also be a constant)
//          () -> setpoint.getAsDouble(),
//          // This uses the output
//          output -> {
//            armSub.setMotor(MathUtil.clamp(output, -1, 1));
//          });
//      this.armSub = armSub;
//      this.Kp = Kp;
//      this.Ki = Ki;
//      this.Kd = Kd;
//      this.setpoint = setpoint;
//      this.tolerance = tolerance;
//      addRequirements(armSub);
//      getController().setTolerance(ArmConstants.Tolerance);
//    }
//    // Returns true when the command should end.
//    @Override
//    public boolean isFinished() {
//      return (getController().atSetpoint() || armSub.dropLimitSwitch() || armSub.raiseLimitSwitch()); // When the are is at the setpoint OR if the limit switches are hit.
//    }
//  }
// >>>>>>> Sadra_H
