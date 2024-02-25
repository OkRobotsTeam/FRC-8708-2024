package frc.robot;

public class Constants {

    public static class Drivetrain {

        public static final double WHEELBASE_METERS = 0.56515;
        public static final double WHEEL_RADIUS_METERS = 0.0508;
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_RADIUS_METERS * (Math.PI * 2);

        public static final boolean FIELD_ORIENTED = false;
        public static final double CONTROLLER_DEADBAND = 0.1;

        public static final double MOVEMENT_MAX_ACCELERATION_METERS_PER_SECOND = 0.1;
        public static final double TURNING_MAX_ACCELERATION_RADIANS_PER_SECOND = 0.1;

        public static final double TURNING_KP = 1;
        public static final double TURNING_KI = 0;
        public static final double TURNING_KD = 0;

        public static final double ALLOWED_DISTANCE_FROM_TARGET_METERS = 0.125;
        public static final double ALLOWED_ROTATION_FROM_TARGET_DEGREES = 3;
        public static final double ALLOWED_ROTATION_FROM_TARGET_PRECISE_DEGREES = 1;

        public static class CANIds {
            public static final int FRONT_LEFT_DRIVE_MOTOR = 2;
            public static final int FRONT_LEFT_ROTATION_MOTOR = 3;
            public static final int FRONT_LEFT_ENCODER_ENCODER = 10;

            public static final int FRONT_RIGHT_DRIVE_MOTOR = 4;
            public static final int FRONT_RIGHT_ROTATION_MOTOR = 5;
            public static final int FRONT_RIGHT_ENCODER_ENCODER = 11;

            public static final int BACK_LEFT_DRIVE_MOTOR = 6;
            public static final int BACK_LEFT_ROTATION_MOTOR = 7;
            public static final int BACK_LEFT_ROTATION_ENCODER = 12;

            public static final int BACK_RIGHT_DRIVE_MOTOR = 8;
            public static final int BACK_RIGHT_ROTATION_MOTOR = 9;
            public static final int BACK_RIGHT_ROTATION_ENCODER = 13;
        }
    }

    public static class Intake {
        public static final double GEAR_RATIO = (8.0 / 60.0) * (16.0 / 60.0) * (12.0 / 36.0);

        public static final double WRIST_STARTUP_POSITION = 0.0;

        public static final double WRIST_PID_KP = 0.2;
        public static final double WRIST_PID_KI = 0.0;
        public static final double WRIST_PID_KD = 0.0;

        public static final double WRIST_FOLDED_SETPOINT_IN_ROTATIONS = 0.0;
        public static final double WRIST_EXTENDED_SETPOINT_IN_ROTATIONS = 0.5;

        public static final int TOP_INTAKE_CURRENT_LIMIT_FREE_IN_AMPS = 10;
        public static final int TOP_INTAKE_CURRENT_LIMIT_STALLED_IN_AMPS = 2;
        public static final int BOTTOM_INTAKE_CURRENT_LIMIT_FREE_IN_AMPS = 10;
        public static final int BOTTOM_INTAKE_CURRENT_LIMIT_STALLED_IN_AMPS = 2;

        public static final double INTAKE_IN_POWER = 1.0;
        public static final double INTAKE_OUT_POWER = -0.5;

        public static class CANIds {
            public static final int TOP_INTAKE = -1;
            public static final int BOTTOM_INTAKE = -1;
            public static final int WRIST = -1;
        }
    }

    public static class Shooter {
        public static final double SHOOTER_ROTATION_GEAR_RATIO = (10.0 / 64.0);
        public static final double SHOOTER_WHEELS_GEAR_RATIO = (10.0 / 64.0);

        public static final boolean SHOOTER_TOP_INVERTED = false;
        public static final boolean SHOOTER_BOTTOM_INVERTED = false;

        public static final double SHOOTER_ROTATION_PID_KP = 0.2;
        public static final double SHOOTER_ROTATION_PID_KI = 0.0;
        public static final double SHOOTER_ROTATION_PID_KD = 0.0;

        public static final double SHOOTER_ROTATION_STARTUP_POSITION = 0.0;

        public static final double SHOOTER_FORWARD_SPEED = 1.0;
        public static final double SHOOTER_REVERSE_SPEED = -0.2;

        public static class CANIds {
            public static final int TOP_SHOOTER = -1;
            public static final int BOTTOM_SHOOTER = -1;
            public static final int SHOOTER_ROTATION = -1;
        }
    }

    public static class Climber {
        public static final double LEFT_CLIMBER_GEAR_RATIO = (8.0 / 54.0);
        public static final double RIGHT_CLIMBER_GEAR_RATIO = (8.0 / 54.0);

        public static final boolean CLIMBER_LEFT_INVERTED = false;
        public static final boolean CLIMBER_RIGHT_INVERTED = false;

        public static final double CLIMBER_PID_KP = 0.2;
        public static final double CLIMBER_PID_KI = 0.0;
        public static final double CLIMBER_PID_KD = 0.0;
        public static final double CLIMBER_PID_MAX_SPEED_IN_ROTATIONS_PER_SECOND = 1000.0;
        public static final double CLIMBER_PID_MAX_ACCELERATION_IN_ROTATIONS_PER_SECOND_SQUARED = 5.0;

        public static final double LEFT_CLIMBER_STARTUP_POSITION = 0.0;
        public static final double RIGHT_CLIMBER_STARTUP_POSITION = 0.0;

        public static final double CLIMBER_DOWN_POSITION = 0.0;
        public static final double CLIMBER_UP_POSITION = 10.0;

        public static class CANIds {
            public static final int LEFT_CLIMBER = -1;
            public static final int RIGHT_CLIMBER = -1;
        }
    }


}
