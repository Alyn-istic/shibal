// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.
// package frc.robot.Commands.Drivetrain;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.robot.Constants.DrivetrainConstants;
// import frc.robot.Subsystems.DrivetrainSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class TurnGyroPID extends PIDCommand {
//   public TurnGyroPID(
//     // The arguments (settings) this command accepts.
//     DrivetrainSubsystem driveSub, // The subsystem (in this case, the DrivetrainSubsystem)
//     Double setpoint // The setpoint is what angle we want our gyro to be at.
//   ) {
//     super(
//         new PIDController(
//           // The PID values we are using goes here. All configurable in Constants.java.
//           DrivetrainConstants.gyroP,
//           DrivetrainConstants.gyroI,
//           DrivetrainConstants.gyroD
//         ),
//         () -> driveSub.getGyroAngle() % 360, // We are getting the gyro angle by calling its function. Then, we mod it by 360 (math stuff)
//         () -> setpoint, // setpoint argument goes here. What it does is already explained above.
//         output -> { // After calculating everything 
//           driveSub.tankDrive( // Running the Arcade Drive function. Doesn't have to be arcade drive, could also be Tank Drive.
//             MathUtil.clamp(output, -1, 1), // Locking the output in between -1 and 1..
//             -MathUtil.clamp(output, -1, 1) // Locking the output in between -1 and 1..
//           );
//         });
//     addRequirements(driveSub); // Telling the robot that this command is using the DrivetrainSubsystem

//     /** The gyro angle will never be exactly at the setpoint.
//      * So, we have to give the robot a range around the setpoint where it would consider the angle to be at the setpoint.
//      * This is called the tolerance.
//     */
//     getController().setTolerance(DrivetrainConstants.gyroTolerance);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return getController().atSetpoint(); // Tells the robot to stop running this command once the gyro's angle is within the tolerated range.
//   }
// }
