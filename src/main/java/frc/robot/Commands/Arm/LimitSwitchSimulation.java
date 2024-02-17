// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.Commands.Arm;

// import java.util.function.BooleanSupplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Subsystems.ArmSubsystem;

// public class LimitSwitchSimulation extends Command {
//   private ArmSubsystem armSub;
//   private BooleanSupplier input;
//   /** Creates a new LimitSwitchSimulation. */
//   public LimitSwitchSimulation(
//     ArmSubsystem armSub,
//     BooleanSupplier input
//   ) {
//     this.armSub = armSub;
//     this.input = input;
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     armSub.setLimitSwitchSim(input.getAsBoolean());
//   }

//   @Override
//   public void end(boolean interrupted) {
//     armSub.setLimitSwitchSim(!input.getAsBoolean());
//   }
// }
