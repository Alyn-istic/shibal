package frc.robot;

// Feel free to add more constants to the list.

public class Constants {
    public class AutonomousConstants {
        public class MoveToAmpTimedConstants {
            public static final double TIME_TURN_TO_AMP = 3.5; // In seconds
            public static final double TIME_MOVE_TO_AMP = 4.5; // In seconds
            public static final double TIME_SHOOT_IN_AMP = 1; // In seconds

            public static final double SPEED_TURN_TO_AMP = -0.5; // Motor speed
            public static final double SPEED_MOVE_TO_AMP = 0.4; // Motor speed
            public static final double SPEED_SHOOT_IN_AMP = 1; // Motor speed
        }
        public class MoveOutOfZoneConstants {
            public static final double TIME_MOVE_OUT_OF_ZONE = 1; // In seconds
            public static final double SPEED_MOVE_OUT_OF_ZONE = 0.35; // Motor speed
        }
    }

    public class DrivetrainConstants {
        public static final int frontLeftID = 2; // Chassis's front-left motor ID
        public static final int frontRightID = 0; // Chassis's front-right motor ID
        public static final int backLeftID = 3; // Chassis's back-left motor ID
        public static final int backRightID = 1; // Chassis's back-right motor ID

        // public static final double distLeftRight = 1; // The distance (in inches) between the left wheels and the right wheels.
        // public static final double wheelRadius = 3; // The radius of the wheels
        // public static final double gearRatio = 12.75; // Gear ratio

        // public static final int ticksPerRev = 2048; // The amount of encoder ticks in a full rotation

        // public static final Port gyroPort = Port.kUSB; // The port that the gyro is connected to.

        // public static final double gyroP = 0.0001; // P value for turning PID command
        // public static final double gyroI = 0; // I value for turning PID command
        // public static final double gyroD = 0; // D value for turning PID command
        // public static final double gyroTolerance = 1; // Tolerance for turning PID command
    }

    public class ArmConstants {
        public static final int leftID = 5; // Arm's left motor ID
        public static final int rightID = 6; // Arm's right motor ID

        public static final int countsPerRev = 1024; // The amount of encoder ticks in a full rotation
        public static final double gearRatio = 4; // The gear ratio of the arm

        public static final int raiseLimitSwitchChannel = 1;
        public static final int dropLimitSwitchChannel = 0;

        public static final double kP = 0.005; // P value for the arm PID command
        public static final double kI = 0; // I value for the arm PID command
        public static final double kD = 0; // D value for the arm PID command
        public static final double tolerance = 1;

        public static final double shootAngle = 100; // The angle of the arm in shooting position
        public static final double insideAngle = 90; // The angle of the arm inside the perimeter of the bumpers
        public static final double intakeAngle = 0; // The angle of the arm in intaking position
        // public static final double dropAngle = 0;  // This is relative to the starting position of the encoders.
    }

    public class IntakeShooterConstants {
        public static final int upperWheelID  = 7; // The motor ID for the intake/shooter's upper wheels
        public static final int lowerWheelID = 8; // The motor ID for the intake/shooter's lower wheels

        public static final double speed = 1; // The intake/shooter's motor speed.
    }

    public class VisionConstants {}

    public class DriverConstants {
        public static final int port = 0; // Controller port

        public static final double joystickDeadband = 0.08; // Deadzone for controller L/R joysticks
        public static final double triggerDeadband = 0.08; // Deadzone for controller L/R triggers

        public static final int leftJoystickAxis = 1; // Left joystick's axis
        public static final int rightJoystickAxis = 5; // Right joystick's axis

        public static final int leftTriggerAxis = 2; // Left trigger axis
        public static final int rightTriggerAxis = 3; // Right trigger axis
    }
}