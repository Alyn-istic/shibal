// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  // Getting the NetworkTable for limelight, aswell as AprilTag coordinate entries.
  private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("Limelight"); // From the NetworkTable, get table called "Limelight" or whatever it's gonna be called.
  private final NetworkTableEntry tx = limelightTable.getEntry("tx");
  private final NetworkTableEntry ty = limelightTable.getEntry("ty");
  private final NetworkTableEntry botPos = limelightTable.getEntry("BotPos");

  private Pose3d visionPose;

  private double poseArray[] = botPos.getDoubleArray(new double[6]);

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double[] getPoseArray() {
    return poseArray;
  }

  public Pose3d getVisionPose() {
    return visionPose;
  }
}
