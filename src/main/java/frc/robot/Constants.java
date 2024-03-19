package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C.Port;

// Feel free to add more constants to the list.

public class Constants {
    public class AutonomousConstants {
        public static final double waitBeforeExecRoutine = 0; // The amount of seconds to wait before executing routines
        public class MoveToAmpTimedConstants {
            public static final double TIME_POS_1 = 2.5;
            public static final double TIME_POS_2 = 2;
            public static final double TIME_POS_3 = 2;           

            public static final double SPEED_POS_1 = 0.4; //speed to the amp from pos1

            // 1 = first position, 2 = second position, 3 third position
            //all constant values are wrong, need testing
            public static final double TIME_TURN_TO_AMP = 1; // In seconds
            public static final double SPEED_TURN_TO_AMP = 0.65; // Motor speed. Was 0.6

            public static final double TIME_LEAVE_AMP = 2; // In seconds
            public static final double SPEED_LEAVE_AMP = -0.55; // Motor speed

            public static final double TIME_MOVE_TO_AMP_FWD = 0.6;// In seconds. Move forward before turn and move sideways. 
            public static final double TIME_MOVE_TO_AMP1 = 0.5; // In seconds. Was 4.5
            public static final double TIME_MOVE_TO_AMP2 = 0.8; // In seconds
            public static final double TIME_MOVE_TO_AMP3= 1.4; // In seconds

            public static final double TIME_SHOOT_IN_AMP = 2; // In seconds
            public static final double SPEED_SHOOT_IN_AMP = 1; // Motor speed

            public static final double SPEED_MOVE_TO_AMP_FWD = -0.55; // Motor speed//was 0.4
            public static final double SPEED_MOVE_TO_AMP1 = -0.4; // Motor speed//was 0.4
            public static final double SPEED_MOVE_TO_AMP2 = -0.4; // Motor speed
            public static final double SPEED_MOVE_TO_AMP3 = -0.4; // Motor speed
        }
        public class MoveOutOfZoneConstants {
            public static final double TIME_MOVE_OUT_OF_ZONE1 = 3; // In seconds. Was 1.4s
            public static final double SPEED_MOVE_OUT_OF_ZONE1 = -0.5; // Motor speed. Was -0.4
        }
    }

    public class DrivetrainConstants {
        public static final int frontLeftID = 0; // Chassis's front-left motor ID
        public static final int frontRightID = 2; // Chassis's front-right motor ID
        public static final int backLeftID = 1; // Chassis's back-left motor ID
        public static final int backRightID = 3; // Chassis's back-right motor ID

        public static final double distLeftRight = 24.8; // The distance (in inches) between the left wheels and the right wheels.
        public static final double wheelRadius = 3; // The radius of the wheels (in inches)
        public static final double gearRatio = 8.46; // Gear ratio

        public static final int countsPerRev = 42; //1024; // The amount of encoder counts in a full rotation

        public static final double encoderCountsToMeters = Units.inchesToMeters(((2 * Math.PI * wheelRadius)/countsPerRev) / gearRatio);

        public static final Port gyroPort = Port.kOnboard; // The port that the gyro is connected to.

        public static final double speed = 1; // hello world

        public static final double startPosX = 2; //
        public static final double startPosY = 7; //

        public static final double driveP = 1.9; // 
        public static final double driveI = 0.0; // 
        public static final double driveD = 0.2; // 
        public static final double driveTolerance = 0; // 

        public static final double motorClamp = 1; //
        public static final double autoSlewRate = 1; // Slewrate for open-looped autonomous routines. was 0.3

        public static final double minAngle = 0; // 
        public static final double maxAngle = 360; // 

        public static final double turnP = 0.02; // P value for turning PID command
        public static final double turnI = 0.005; // I value for turning PID command
        public static final double turnD = 0.00002; // D value for turning PID command
        public static final double gyroTolerance = 1; // Tolerance for turning PID command
        public static final double kS = 1.4796; // 
        public static final double kV = 0.6432; // 
    }

    public class ArmConstants {
        public static final int leftID = 6; // Arm's left motor ID
        public static final int rightID = 5; // Arm's right motor ID

        public static final double armManualSpeed = 0.3;

        public static final int countsPerRev = 4096; // The amount of encoder ticks in a full rotation
        public static final double gearRatio = 4; // The gear ratio of the arm
        public static final double startingAngle = 90; // 

        public static final int raiseLimitSwitchChannel1 = 1; // 
        public static final int raiseLimitSwitchChannel2 = 2; //
        public static final int dropLimitSwitchChannel1 = 9; // 
        public static final int dropLimitSwitchChannel2 = 8; // 

        public static final double raiseP = 0.03; // P value for the arm PID command
        public static final double raiseI = 0; // I value for the arm PID command
        public static final double raiseD = 0; // D value for the arm PID command

        public static final double dropP = 0.025; // P value for the arm PID command. Was 0.03.
        public static final double dropI = 0; // I value for the arm PID command
        public static final double dropD = 0.00003; // D value for the arm PID command. Was 0.000025.

        public static final double minAngle = 0; // 
        public static final double maxAngle = 360; // 

        public static final double tolerance = 1; // 
        public static final double setpointOffset = 0; // 

        public static final double shootAngle = 111; // The angle of the arm in shooting position. Was 100.
        public static final double sourceIntakeAngle = 95; //Intake from source angle. Not using
        public static final double intakeInsideAngle = 50; // The angle of the arm inside the perimeter preparing to intake.
        public static final double drivingUnderStage = 25; // The angle of the position of the arm while the robot is driving under stage
        public static final double shootInsideAngle = 90; // The angle of the arm inside the perimeter preparing to move to shoot position.
        public static final double intakeAngle = 0; // The angle of the arm in intaking position
        public static final double[] angles = {
            intakeAngle,
            drivingUnderStage,
            shootInsideAngle,
            shootAngle
        };
        
        public static final double raiseMotorClamp = 0.68; // Should be between 0 and 1;
        public static final double dropMotorClamp = 0.6;
        // public static final double dropAngle = 0;  // This is relative to the starting position of the encoders.
    }  

    public class IntakeShooterConstants {
        public static final int upperWheelID  = 9; // The motor ID for the intake/shooter's upper wheels
        public static final int lowerWheelID = 8; // The motor ID for the intake/shooter's lower wheels

        public static final double speed = 0.85; // The intake/shooter's motor speed.
    }
    public class ClimberConstants{ //Need to change values based on testing
        public static final int climberID = 10; // 
        public static final double climberSpeed = 1; // 
    }

    public class VisionConstants {
        public static final double cameraAngle = 1; // Camera angle offset in degrees
    }

    public class LEDconstants {
        public static final int LEDTalonPort = 8; // 
    }

    public class DriverConstants {
        public static final int driverPort = 0; // Controller port for driver
        public static final int operatorPort = 1; // Controller port for operator
        public static final int testerPort = 2; // 

        public static final double joystickDeadband = 0.08; // Deadzone for controller L/R joysticks
        public static final double triggerDeadband = 0.08; // Deadzone for controller L/R triggers

        public static final int leftJoystickAxis = 1; // Left joystick's axis
        public static final int rightJoystickAxis = 5; // Right joystick's axis

        public static final int leftTriggerAxis = 2; // Left trigger axis
        public static final int rightTriggerAxis = 3; // Right trigger axis

        public static final int leftBumperButton = 5; // 
        public static final int rightBumperButton = 6; // 
    }
}