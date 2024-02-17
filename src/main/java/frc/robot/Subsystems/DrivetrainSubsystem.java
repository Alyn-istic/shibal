// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
// import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  // Getting the NetworkTable for limelight, aswell as AprilTag coordinate entries.
  // private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Limelight"); // From the NetworkTable, get table called "Limelight" or whatever it's gonna be called.
  // private final NetworkTableEntry tx = table.getEntry("tx");
  // private final NetworkTableEntry ty = table.getEntry("ty");

  // Initializing VictorSPX motors for the drivetrain.
  private final WPI_VictorSPX frontLeft = new WPI_VictorSPX(DrivetrainConstants.frontLeftID);
  private final WPI_VictorSPX frontRight = new WPI_VictorSPX(DrivetrainConstants.frontRightID);
  private final WPI_VictorSPX backLeft = new WPI_VictorSPX(DrivetrainConstants.backLeftID);
  private final WPI_VictorSPX backRight = new WPI_VictorSPX(DrivetrainConstants.backRightID);

  // Kinematics (Math shinanigans. Don't worry about it, the robot calculates it for you).
  //private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(DrivetrainConstants.distLeftRight));
  
  // Creating a Differential Drive using the front motors. This is like grouping all the motors together.
  private final DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight);

  // Initiating the gyro.
  //private final AHRS gyro = new AHRS(DrivetrainConstants.gyroPort);

  /* Vision shinanigans, learning from 7476 code.
   * Coordinate order (for the poseCoordinates array):
   * 0: X
   * 1: Y
   * 2: Z
   * 3: Roll
   * 4: Pitch
   * 5: Yaw
  */
  // private final double poseCoordinates[] = table.getEntry("visionpose").getDoubleArray(new double[6]);
  // private Pose3d visionPose;
  // private DifferentialDrivePoseEstimator poseEstimator;

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    // Inverting the left motors
    frontLeft.setInverted(true);
    backLeft.setInverted(true);
    frontRight.setInverted(false);
    backRight.setInverted(false);

    //Telling back-motors to follow the front-motors because FIRST decided to remove MotorControllerGroups.
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);

    // Setting the neutral mode of the motors to "Brake". This means that they will stop immediately when told to.
    frontLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);

    // Resetting probably non-existing encoders just for the sake of it.
    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
    backLeft.setSelectedSensorPosition(0);
    backRight.setSelectedSensorPosition(0);
    // visionPose and poseEstimator's values are assigned here because the necessary values may not have been recieved yet.
    //visionPose = new Pose3d(poseCoordinates[0], poseCoordinates[1], poseCoordinates[2], new Rotation3d(poseCoordinates[3], poseCoordinates[4], poseCoordinates[5]));
    //poseEstimator = new DifferentialDrivePoseEstimator(kinematics, gyro.getRotation2d(), getLeftDistance(), getRightDistance(), visionPose.toPose2d());
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Gyro Yaw", getGyroAngle() % 360);
    SmartDashboard.putNumber("Front Left Motor Speed", frontLeft.get());
    SmartDashboard.putNumber("Front Right Motor Speed", frontRight.get());

  }

  public void tankDrive(double leftSpeed, double rightSpeed) { // Tankdrive function
    drive.tankDrive(leftSpeed, rightSpeed);
  }
  // public double getGyroAngle() { // Function for getting the gyro's angle.
  //   return gyro.getAngle();
  // }

  /* "getLeftDistance()" and "getRightDistance()" should return left/right distance. Some math will have to be done using encoder positions. 

   * The equation used is the following: distance = encoderPosition / gearRatio * (ticksPerRevolution/wheelCircumference)
   * 
   * The idea is to get the distance travelled per tick, and multiply it by the amount of ticks registered by the encoder and applying gear ratios.
   * Distance per tick can be found by dividing the total amount of ticks per full revolution by the circumference, which can be calculated by multiplying the radius by 2PI.
   * 
   * The result would be a distance in inches. But that's not efficent. So, we then convert it to meters using the "Units" class.
  */
  // public double getLeftDistance() {
  //   return Units.inchesToMeters(frontLeft.getSelectedSensorPosition() / DrivetrainConstants.gearRatio * (DrivetrainConstants.ticksPerRev / (DrivetrainConstants.wheelRadius * Math.PI * 2)));
  // }
  // public double getRightDistance() {
  //   return Units.inchesToMeters(frontRight.getSelectedSensorPosition() / DrivetrainConstants.gearRatio * (DrivetrainConstants.ticksPerRev / (DrivetrainConstants.wheelRadius * Math.PI * 2)));
  // }

  // Using estimator (sort of stolen from 7476 code), this would (sort of) return estimated x & y coordinates of the robot.
  // public Pose2d getEstimatedPos() {
  //   return poseEstimator.getEstimatedPosition();
  // }
}