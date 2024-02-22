// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmPID extends PIDCommand { //class
  private ArmSubsystem armSub; //defining class into object, maybe defing object
  /** Creates a new ArmRaise. */
  public ArmPID(
    ArmSubsystem armSub, //supplying from configurebidings from robotcontainer
    DoubleSupplier kP, //constants
    DoubleSupplier kI,
    DoubleSupplier kD,
    DoubleSupplier setpoint,
    DoubleSupplier tolerance
  ) {
    super(//Creates a new PIDCommand, which controls the given output with a PIDController.
    // Parameters:
    // controller the controller that controls the output.
    // measurementSource the measurement of the process variable
    // setpointSource the controller's setpoint
    // useOutput the controller's output
    // requirements the subsystems required by this command
        // The controller that the command will use
        new PIDController(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble()),
        // This should return the measurement
        () -> armSub.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint.getAsDouble(),
        // This uses the output
        output -> {
          armSub.setMotor(MathUtil.clamp(output, -1, 1)); //setting motor to "outpit" with clamp on min and max values
          SmartDashboard.putNumber("Arm PID Output", output);
        });
    this.armSub = armSub; //not sure why we use this
    getController().setTolerance(tolerance.getAsDouble()); //assuming its just setting the PID tolerance to the supplier values
    addRequirements(armSub);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //either target reached by arm, or arm has hit limit switches
    return (getController().atSetpoint() || armSub.dropLimitSwitch() || armSub.raiseLimitSwitch()); // When the arm is at the setpoint OR if the limit switches are hit.
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
