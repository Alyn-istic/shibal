// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
  private final WPI_TalonSRX frontLeft = new WPI_TalonSRX(DrivetrainConstants.frontLeftID);
  private final WPI_TalonSRX frontRight = new WPI_TalonSRX(DrivetrainConstants.frontRightID);
  private final WPI_VictorSPX backLeft = new WPI_VictorSPX(DrivetrainConstants.backLeftID);
  private final WPI_VictorSPX backRight = new WPI_VictorSPX(DrivetrainConstants.backRightID);

  // Creating a Differential Drive using the front motors. This is like grouping all the motors together.
  private final DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight); //new object, using two parameteres for both sides

  // Initiating the gyro.
  private final AHRS gyro = new AHRS(DrivetrainConstants.gyroPort);

  //PID controller
  // private final PIDController driveController = new PIDController(DrivetrainConstants.driveP, DrivetrainConstants.driveI, DrivetrainConstants.driveD);
  private final SimpleMotorFeedforward leftFeedForward = new SimpleMotorFeedforward(DrivetrainConstants.kS, DrivetrainConstants.kV);
  private final SimpleMotorFeedforward rightFeedForward = new SimpleMotorFeedforward(DrivetrainConstants.kS, DrivetrainConstants.kV);
  private final PIDController leftDriveController = new PIDController(DrivetrainConstants.driveP, DrivetrainConstants.driveI, DrivetrainConstants.driveD);
  private final PIDController rightDriveController = new PIDController(DrivetrainConstants.driveP, DrivetrainConstants.driveI, DrivetrainConstants.driveD);
  private final PIDController turnController = new PIDController(DrivetrainConstants.turnP, DrivetrainConstants.turnI, DrivetrainConstants.turnD);  

  // Path planner constraints
  PathConstraints constraints = new PathConstraints(3.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  // Kinematics
  private DifferentialDrivePoseEstimator poseEstimator;
  private Field2d field = new Field2d();
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
        // Create frame for left motors (negated speed)
        log.motor("drive-left").voltage(m_appliedVoltage.mut_replace(-frontLeft.get() * RobotController.getBatteryVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(getLeftDistance(), Meters))
          .linearVelocity(m_velocity.mut_replace(getLeftVelocity(), MetersPerSecond));

        // Create frame for right motors (negated speed)
        log.motor("drive-right").voltage(m_appliedVoltage.mut_replace(-frontRight.get() * RobotController.getBatteryVoltage(), Volts))
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
    frontLeft.configFactoryDefault();
    backLeft.configFactoryDefault();
    frontRight.configFactoryDefault();
    backRight.configFactoryDefault();

    // Inverting the left motors
    frontLeft.setInverted(true);
    backLeft.setInverted(true);
    frontRight.setInverted(false);
    backRight.setInverted(false);

    //Telling back-motors to follow the front-motors because FIRST decided to remove MotorControllerGroups.
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);

    // Setting the neutral mode of the motors to "Brake". This means that they will stop immediately when told to.
    //They are now in "Cost", no power will be passed in and the moters will continue going a while due to momentum. 
    frontLeft.setNeutralMode(NeutralMode.Coast);//was brake
    frontRight.setNeutralMode(NeutralMode.Coast);
    backLeft.setNeutralMode(NeutralMode.Coast);
    frontRight.setNeutralMode(NeutralMode.Coast);

    // Resetting probably non-existing encoders just for the sake of it.

    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
    backLeft.setSelectedSensorPosition(0);
    backRight.setSelectedSensorPosition(0);
   
    // Resetting gyro
    gyro.reset();

    // Pose estimation
    poseEstimator = new DifferentialDrivePoseEstimator(
      kinematics,
      gyro.getRotation2d(),
      getLeftDistance(),
      getRightDistance(),
      new Pose2d(DrivetrainConstants.startPosX, DrivetrainConstants.startPosY, new Rotation2d(0))
    );
    SmartDashboard.putData("Field ", field);

    AutoBuilder.configureRamsete(
      this::getBotPose,
      this::resetBotPose,
      this::getChassisSpeeds,
      this::setChassisSpeed,
      new ReplanningConfig(),
      () -> false,
      this
    );
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
    SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right Velocity", getRightVelocity());

    SmartDashboard.putNumber("Gyro Angle", getGyroAngle() % 360);
    System.out.println(getPathInverted());
  }

  public Boolean getPathInverted() {
    if (DriverStation.getAlliance().isPresent()) {
      return (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red));
    }
    return false;
  }

  @Override
  public void simulationPeriodic() {
    int left = (int)(MathUtil.clamp(frontLeft.get(), -1, 1) * 1101.0);
    int right = (-(int)(MathUtil.clamp(frontRight.get(), -1, 1) * 1101.0));

    frontLeft.getSimCollection().addQuadraturePosition(left);
    frontRight.getSimCollection().addQuadraturePosition(right);

    frontLeft.getSimCollection().setQuadratureVelocity(left);
    frontRight.getSimCollection().setQuadratureVelocity(right);

    gyro.setAngleAdjustment(Units.radiansToDegrees(kinematics.toTwist2d(-getLeftDistance(), -getRightDistance()).dtheta) * 0.75);

    field.setRobotPose(getBotPose());
  }

  public void tankDriveSpeed(double leftSpeed, double rightSpeed) { // Tankdrive using speed.
    drive.tankDrive(
      MathUtil.clamp(leftSpeed, -DrivetrainConstants.motorClamp, DrivetrainConstants.motorClamp),
      MathUtil.clamp(rightSpeed, -DrivetrainConstants.motorClamp, DrivetrainConstants.motorClamp)
    );
  }

  public double getGyroAngle() { // Function for getting the gyro's angle.
    return gyro.getAngle();
  }

  public Pose2d getBotPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetBotPose(Pose2d pose) {
    poseEstimator.resetPosition(gyro.getRotation2d(), new DifferentialDriveWheelPositions(getLeftDistance(), getRightDistance()), pose);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity()));
  }

  public void setChassisSpeed(ChassisSpeeds chassis) {
    DifferentialDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(chassis);

    double leftFF = leftFeedForward.calculate(-speeds.leftMetersPerSecond);
    double rightFF = rightFeedForward.calculate(-speeds.rightMetersPerSecond);

    double leftVelocityCorrection = leftDriveController.calculate(getLeftVelocity(), -speeds.leftMetersPerSecond);
    double rightVelocityCorrection = rightDriveController.calculate(getRightVelocity(), -speeds.rightMetersPerSecond);

    frontLeft.setVoltage(leftVelocityCorrection + leftFF);
    frontRight.setVoltage(rightVelocityCorrection + rightFF);
    
    drive.feed();
  }

  // Math: pos * ((2PI*radius)/CPR)/GearRatio -> convert from inches to meters
  public double getLeftDistance(){
    return Units.inchesToMeters(
      frontLeft.getSelectedSensorPosition() * ((2 * Math.PI * DrivetrainConstants.wheelRadius)/DrivetrainConstants.countsPerRev) / DrivetrainConstants.gearRatio
    );
  }
  public double getRightDistance(){
    return Units.inchesToMeters(
      frontRight.getSelectedSensorPosition() * ((2 * Math.PI * DrivetrainConstants.wheelRadius)/DrivetrainConstants.countsPerRev) / DrivetrainConstants.gearRatio
    );
  }
  public double getLeftVelocity(){
    return frontLeft.getSelectedSensorVelocity() * ((2 * Math.PI * DrivetrainConstants.wheelRadius)/DrivetrainConstants.countsPerRev) / DrivetrainConstants.gearRatio;
  }

  public double getRightVelocity(){
    return frontRight.getSelectedSensorVelocity() * ((2 * Math.PI * DrivetrainConstants.wheelRadius)/DrivetrainConstants.countsPerRev) / DrivetrainConstants.gearRatio;

  }
  public PIDController getLeftDriveController() {
    return leftDriveController;
  }

  public PIDController getRightDriveController() {
    return rightDriveController;
  }

  public boolean isDriveControllersAtSetpoint() {
    return (getLeftDriveController().atSetpoint() && getRightDriveController().atSetpoint());
  }

  public PIDController getTurnController() {
    return turnController;
  }

  public Command testPath1() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("ToNote1Path");
    return AutoBuilder.pathfindThenFollowPath(path, constraints);
  }

  public Command testPath2() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("TestPath2");
    return AutoBuilder.pathfindThenFollowPath(path, constraints);
  }

  public void operatorReset() {
    frontLeft.configFactoryDefault();
    backLeft.configFactoryDefault();
    frontRight.configFactoryDefault();
    backRight.configFactoryDefault();

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
  }
}