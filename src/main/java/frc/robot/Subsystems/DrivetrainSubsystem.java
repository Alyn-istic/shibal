// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DrivetrainConstants;

// Static imports for units
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class DrivetrainSubsystem extends SubsystemBase {

  // Initializing VictorSPX motors for the drivetrain.
  private final WPI_VictorSPX frontLeft = new WPI_VictorSPX(DrivetrainConstants.frontLeftID);
  private final WPI_VictorSPX frontRight = new WPI_VictorSPX(DrivetrainConstants.frontRightID);
  private final WPI_VictorSPX backLeft = new WPI_VictorSPX(DrivetrainConstants.backLeftID);
  private final WPI_VictorSPX backRight = new WPI_VictorSPX(DrivetrainConstants.backRightID);

  // Creating a Differential Drive using the front motors. This is like grouping all the motors together.
  private final DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight); //new object, using two parameteres for both sides

  // Initiating the gyro.
  private final AHRS gyro = new AHRS(DrivetrainConstants.gyroPort);

  // Kinematics
  private DifferentialDrivePoseEstimator poseEstimator;
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(DrivetrainConstants.distLeftRight));

  /* Mutable holders
    The idea behind a mutable object is that you can modify its value.
    For example, you can create a variable called x, and assign it's value with integer 1.
    When you run (x += 1;), you are modifying the value and adding 1, making it a mutable object.

    However, when you create a variable called x, and assign it's value with string "hello", you have created a immutable object, meaning you can't modify it.
    If you concatenate it with "!" like so (x = x + "!";), you are not actually modifying the variable. You are creating a seperate copy with a assigned value of "hello!".
    We use mutable objects because we can modify them without creating new "copies". This in turn reduces the memory usage.

    Which values are mutable or immutable? It all comes down to which language you are using.

    The following variables hold measurements in special stores in the RoboRIO.
   */
  private final MutableMeasure<Voltage> m_appliedVoltage = MutableMeasure.mutable(Volts.of(0));
  private final MutableMeasure<Distance> m_distance = MutableMeasure.mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> m_velocity = MutableMeasure.mutable(MetersPerSecond.of(0));

  // System Identification, used to help find some values
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
      (Measure<Voltage> voltage) -> {
        frontLeft.setVoltage(voltage.in(Volts));
        frontRight.setVoltage(voltage.in(Volts));
      },
      /* 
       * The idea for the voltage is that motor speed is influenced by battery voltage.
       * For example: (voltage = motorSpeed * batteryVoltage) can be re-arranged into (motorSpeed = voltage / batteryVoltage)
       * The other calculations such as position and velocity can be found in their repspective functions
       */
      log -> {
        // Create frame for left motors
        log.motor("drive-left").voltage(m_appliedVoltage.mut_replace(frontLeft.get() * RobotController.getBatteryVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(getLeftDistance(), Meters))
          .linearVelocity(m_velocity.mut_replace(getLeftVelocity(), MetersPerSecond));

        // Create frame for right motors
        log.motor("drive-right").voltage(m_appliedVoltage.mut_replace(frontRight.get() * RobotController.getBatteryVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(getRightDistance(), Meters))
          .linearVelocity(m_velocity.mut_replace(getRightVelocity(), MetersPerSecond)); 
      }, this
    )
  );

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

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

    // Resetting gyro
    gyro.reset();

    // Pose estimation
    poseEstimator = new DifferentialDrivePoseEstimator(kinematics, gyro.getRotation2d(), getLeftDistance(), getRightDistance(), new Pose2d(DrivetrainConstants.startPosX, DrivetrainConstants.startPosY, new Rotation2d(0)));
  }

  @Override
  public void periodic() {
    poseEstimator.update(gyro.getRotation2d(), getLeftDistance(), getRightDistance());

    SmartDashboard.putNumber("Front Left Motor Speed", frontLeft.get());
    SmartDashboard.putNumber("Front Right Motor Speed", frontRight.get());

    SmartDashboard.putNumber("Bot X", getBotPose().getX());
    SmartDashboard.putNumber("Bot Y", getBotPose().getY());
    SmartDashboard.putNumber("Bot Rotation", getBotPose().getRotation().getDegrees());

    SmartDashboard.putNumber("Left Pos", frontLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Pos", frontRight.getSelectedSensorPosition());
    SmartDashboard.putNumber("Left Distance", getLeftDistance());
    SmartDashboard.putNumber("Right Distance", getRightDistance());

    SmartDashboard.putNumber("Gyro Angle", getGyroAngle() % 360);
  }

  public void tankDriveSpeed(double leftSpeed, double rightSpeed) { // Tankdrive using speed.
    drive.tankDrive(leftSpeed, rightSpeed);
  }
  public void tankDriveVoltage(double leftVolts, double rightVolts) { // Tankdrive using voltage. The reason we have this is because "speed" is based on battery voltage (12V).
    frontLeft.setVoltage(leftVolts);
    frontRight.setVoltage(rightVolts);
    drive.feed();
  }

  public double getGyroAngle() { // Function for getting the gyro's angle.
    return gyro.getAngle();
  }

  public Pose2d getBotPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /* Following math is copied from 7476:
   * input / gearRatio * 2PI * wheelRadius
   */
  public double getLeftDistance() {
    return Units.inchesToMeters(frontLeft.getSelectedSensorPosition() / DrivetrainConstants.gearRatio * (2*Math.PI) * DrivetrainConstants.wheelRadius);
  }
  public double getRightDistance() {
    return Units.inchesToMeters(frontRight.getSelectedSensorPosition() / DrivetrainConstants.gearRatio * (2*Math.PI) * DrivetrainConstants.wheelRadius);
  }
  public double getLeftVelocity() {
    return Units.inchesToMeters(frontLeft.getSelectedSensorVelocity() / DrivetrainConstants.gearRatio * (2*Math.PI) * DrivetrainConstants.wheelRadius);
  }
  public double getRightVelocity() {
    return Units.inchesToMeters(frontRight.getSelectedSensorVelocity() / DrivetrainConstants.gearRatio * (2*Math.PI) * DrivetrainConstants.wheelRadius);
  }

  public DifferentialDriveWheelSpeeds wheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightDistance());
  }
}