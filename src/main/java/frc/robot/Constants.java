package frc.robot;

public class Constants {

    public static class SwerveDrivetrain {
        // Physical Attributes
        public static final double WHEELBASE_IN_METERS = 0.56515;
        public static final double WHEEL_RADIUS_IN_METERS = 0.0508;
        public static final double WHEEL_CIRCUMFERENCE_IN_METERS = WHEEL_RADIUS_IN_METERS * (Math.PI * 2);
        public static final boolean TURNING_MOTORS_INVERTED = true;
        public static final boolean DRIVE_MOTORS_INVERTED = false;


        // Control flags
        public static final boolean FIELD_ORIENTED = false;
        public static final boolean BRAKING_DURING_AUTONOMOUS = true;
        public static final boolean BRAKING_DURING_TELEOP = false;


        //Control Tuning
        public static final double CONTROLLER_DEADZONE = 0.1;
        public static final double CONTROLLER_CUBIC_LINEARITY = 0.4;


        // Robot speed and acceleration limiters
        public static final double MOVEMENT_MAX_SPEED_IN_METERS_PER_SECOND = 3.0;  // Max ~3.0
        public static final double MOVEMENT_MAX_ACCELERATION_IN_METERS_PER_SECOND_SQUARED = 3.0;  // 1 second to full speed
        public static final double TURNING_MAX_ANGULAR_VELOCITY_IN_RADIANS_PER_SECOND = Math.toRadians(360);  // 1 rotation per second
        public static final double TURNING_MAX_ANGULAR_ACCELERATION_IN_RADIANS_PER_SECOND_SQUARED = Math.toRadians(720);  // one half second to full turn speed


        // Wheel rotation speed and acceleration limiters
        public static final double WHEEL_MAX_ANGULAR_VELOCITY_IN_RADIANS_PER_SECOND_SQUARED = Math.PI * 2;
        public static final double WHEEL_MAX_ANGULAR_ACCELERATION_IN_RADIANS_PER_SECOND_SQUARED = Math.PI * 200;


        // PID for rotating the robot during auto, disabled during TeleOp
        public static final double TURNING_KP = 1;
        public static final double TURNING_KI = 0;
        public static final double TURNING_KD = 0;


        // Thresholds for movement and turning in auto to allow for navigating to positions that are "close enough"
        public static final double ALLOWED_DISTANCE_FROM_TARGET_IN_METERS = 0.125;
        // The standard allowed rotation is for when we want to be less precise about our angle (when driving)
        public static final double ALLOWED_ROTATION_FROM_TARGET_IN_RADIANS = Math.toRadians(3);
        // The precise allowed rotation is for when we want to be more precise about our angle (when shooting)
        public static final double ALLOWED_ROTATION_FROM_TARGET_PRECISE_IN_RADIANS = Math.toRadians(1);


        public static class CANIds {
            public static final int FRONT_LEFT_DRIVE_MOTOR = 1;  // On can bus "CTRE"
            public static final int FRONT_LEFT_ROTATION_MOTOR = 2;  // On can bus "rio"
            public static final int FRONT_LEFT_ROTATION_ENCODER = 2;  // On can bus "CTRE"

            public static final int FRONT_RIGHT_DRIVE_MOTOR = 3;  // On can bus "CTRE"
            public static final int FRONT_RIGHT_ROTATION_MOTOR = 3;  // On can bus "rio"
            public static final int FRONT_RIGHT_ROTATION_ENCODER = 4;  // On can bus "CTRE"

            public static final int BACK_LEFT_DRIVE_MOTOR = 5;  // On can bus "CTRE"
            public static final int BACK_LEFT_ROTATION_MOTOR = 4;  // On can bus "rio"
            public static final int BACK_LEFT_ROTATION_ENCODER = 6;  // On can bus "CTRE"

            public static final int BACK_RIGHT_DRIVE_MOTOR = 7;  // On can bus "CTRE"
            public static final int BACK_RIGHT_ROTATION_MOTOR = 5;  // On can bus "rio"
            public static final int BACK_RIGHT_ROTATION_ENCODER = 8;  // On can bus "CTRE"
        }
    }

    public static class Intake {
        public static final double GEAR_RATIO = (8.0 / 60.0) * (16.0 / 60.0) * (12.0 / 36.0);
        public static final boolean TOP_INTAKE_REVERSED = false;
        public static final boolean BOTTOM_INTAKE_REVERSED = false;
        public static final boolean WRIST_REVERSED = true;

        public static final double WRIST_STARTUP_POSITION = 0.0;

        public static final double WRIST_PID_KP = 8;
        public static final double WRIST_PID_KI = 0.0;
        public static final double WRIST_PID_KD = 0.1;

        public static final double WRIST_MAX_SPEED = 0.6;

        public static final double WRIST_FOLDED_SETPOINT_IN_ROTATIONS = 0.0;
        public static final double WRIST_HALF_EXTENDED_SETPOINT_IN_ROTATIONS = 0.297 * (2.0 / 5.0);
        public static final double WRIST_EXTENDED_SETPOINT_IN_ROTATIONS = 0.297;

        public static final int TOP_INTAKE_CURRENT_LIMIT_FREE_IN_AMPS = 15;
        public static final int TOP_INTAKE_CURRENT_LIMIT_STALLED_IN_AMPS = 15;
        public static final int BOTTOM_INTAKE_CURRENT_LIMIT_FREE_IN_AMPS = 15;
        public static final int BOTTOM_INTAKE_CURRENT_LIMIT_STALLED_IN_AMPS = 15;

        public static final double INTAKE_IN_POWER = 1.0;
        public static final double INTAKE_OUT_POWER = -0.5;

        public static class CANIds {
            public static final int TOP_INTAKE = 6;  // On can bus "rio"
            public static final int BOTTOM_INTAKE = 7;  // On can bus "rio"
            public static final int WRIST = 8;  // On can bus "rio"
        }
    }

    public static class Shooter {
        public static final double SHOOTER_ROTATION_GEAR_RATIO = (10.0 / 64.0);
        public static final double SHOOTER_WHEELS_GEAR_RATIO = (10.0 / 64.0);

        public static final boolean SHOOTER_TOP_INVERTED = true;
        public static final boolean SHOOTER_BOTTOM_INVERTED = true;

        public static final double SHOOTER_ROTATION_PID_KP = 0;
        public static final double SHOOTER_ROTATION_PID_KI = 0;
        public static final double SHOOTER_ROTATION_PID_KD = 0.0;

        public static final double SHOOTER_ROTATION_STARTUP_POSITION = 0.15;
        public static final double SHOOTER_ROTATION_MANUAL_ADJUST_START_DEGREES = 20.0;



        public static final double SHOOTER_FORWARD_SPEED = 1.0;
        public static double SHOOTER_FORWARD_SLOW_SPEED = 0.25;
        public static final double SHOOTER_REVERSE_SPEED = -0.2;

        public static class CANIds {
            public static final int TOP_SHOOTER = 9;  // On can bus "CTRE"
            public static final int BOTTOM_SHOOTER = 10;  // On can bus "CTRE"
            public static final int SHOOTER_ROTATION = 9;  // On can bus "rio"
        }
    }

    public static class Climber {
        public static final double LEFT_CLIMBER_GEAR_RATIO = (8.0 / 54.0);
        public static final double RIGHT_CLIMBER_GEAR_RATIO = (8.0 / 54.0);

        public static final boolean CLIMBER_LEFT_INVERTED = false;
        public static final boolean CLIMBER_RIGHT_INVERTED = false;

        public static final double CALIBRATION_DELAY_MS = 500;


        public static final double CLIMBER_PID_KP = 0.2;
        public static final double CLIMBER_PID_KI = 0.0;
        public static final double CLIMBER_PID_KD = 0.0;
        public static final double CLIMBER_PID_MAX_SPEED_IN_ROTATIONS_PER_SECOND = 5.0;
        public static final double CLIMBER_PID_MAX_ACCELERATION_IN_ROTATIONS_PER_SECOND_SQUARED = 10.0;

        public static final double LEFT_CLIMBER_STARTUP_POSITION = 0.0;
        public static final double RIGHT_CLIMBER_STARTUP_POSITION = 0.0;

        public static final double CLIMBER_DOWN_POSITION = 0.0;
        public static final double CLIMBER_UP_POSITION = 10.0;

        public static class CANIds {
            public static final int LEFT_CLIMBER = 11;  // On can bus "CTRE"
            public static final int RIGHT_CLIMBER = 12;  // On can bus "CTRE"
        }
    }

    public static class Limelight {
        public static final double FIELD_WIDTH_IN_METERS = 16.4592;
        public static final double FIELD_HEIGHT_IN_METERS = 8.21055;
    }
}
