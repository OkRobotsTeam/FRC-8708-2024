package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class Constants {

    public static class SwerveDrivetrain {
        // Physical Attributes
        public static final double WHEELBASE_IN_METERS = 0.56515;
        public static final double DRIVEBASE_RADIUS_IN_METERS = (WHEELBASE_IN_METERS / 2) * Math.sqrt(2);
        public static final double WHEEL_RADIUS_IN_METERS = 0.044;
        public static final double DRIVE_GEAR_RATIO = (1.0 / 5.60);
        public static final double WHEEL_CIRCUMFERENCE_IN_METERS = WHEEL_RADIUS_IN_METERS * (Math.PI * 2);
        public static final boolean TURNING_MOTORS_INVERTED = true;
        public static final boolean DRIVE_MOTORS_INVERTED = false;


        // Control flags
        public static final boolean BRAKING_DURING_AUTONOMOUS = true;
        public static final boolean BRAKING_DURING_TELEOP = false;


        //Control Tuning
        public static final double CONTROLLER_DEADZONE = 0.1;
        public static final double CONTROLLER_CUBIC_LINEARITY = 0.4;


        // Robot speed and acceleration limiters
        public static final double MOVEMENT_MAX_SPEED_IN_METERS_PER_SECOND = 6.0;  // Max ~3.0
        public static final double MOVEMENT_MAX_ACCELERATION_IN_METERS_PER_SECOND_SQUARED = 3.0;  // 1 second to full speed
        public static final double TURNING_MAX_ANGULAR_VELOCITY_IN_RADIANS_PER_SECOND = Math.toRadians(360);  // 1 rotation per second
        public static final double TURNING_MAX_ANGULAR_ACCELERATION_IN_RADIANS_PER_SECOND_SQUARED = Math.toRadians(720);  // one half second to full turn speed


        // Wheel rotation speed and acceleration limiters
        public static final double WHEEL_MAX_ANGULAR_VELOCITY_IN_RADIANS_PER_SECOND_SQUARED = Math.PI * 2;
        public static final double WHEEL_MAX_ANGULAR_ACCELERATION_IN_RADIANS_PER_SECOND_SQUARED = Math.PI * 200;

        public static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
                new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5, 0.0, 0.0), // Rotation PID constants
                1.5, // Max module speed, in m/s
                DRIVEBASE_RADIUS_IN_METERS, // Drive base radius in meters. Distance from robot center to the furthest module.
                new ReplanningConfig() // Default path re-planning config. See the API for the options here
        );


        public static class CANIds {
            public static final int FRONT_LEFT_DRIVE_MOTOR = 9;
            public static final int FRONT_LEFT_ROTATION_MOTOR = 2;
            public static final int FRONT_LEFT_ROTATION_ENCODER = 10;

            public static final int FRONT_RIGHT_DRIVE_MOTOR = 11;
            public static final int FRONT_RIGHT_ROTATION_MOTOR = 3;
            public static final int FRONT_RIGHT_ROTATION_ENCODER = 12;

            public static final int BACK_LEFT_DRIVE_MOTOR = 13;
            public static final int BACK_LEFT_ROTATION_MOTOR = 4;
            public static final int BACK_LEFT_ROTATION_ENCODER = 14;

            public static final int BACK_RIGHT_DRIVE_MOTOR = 15;
            public static final int BACK_RIGHT_ROTATION_MOTOR = 5;
            public static final int BACK_RIGHT_ROTATION_ENCODER = 16;
        }
    }

    public static class Intake {
        public static final double GEAR_RATIO = (8.0 / 60.0) * (16.0 / 60.0) * (12.0 / 36.0);
        public static final boolean TOP_INTAKE_REVERSED = false;
        public static final boolean BOTTOM_INTAKE_REVERSED = false;
        public static final boolean WRIST_REVERSED = true;

        public static final double WRIST_STARTUP_POSITION = -0.01;

        public static final double WRIST_PID_KP = 8;
        public static final double WRIST_PID_KI = 0.0;
        public static final double WRIST_PID_KD = 0.2;

        public static final double WRIST_MAX_SPEED = 0.6;

        public static final double WRIST_FOLDED_SETPOINT_IN_ROTATIONS = 0.0;
        public static final double WRIST_HALF_EXTENDED_SETPOINT_IN_ROTATIONS = 0.297 * (2.0 / 5.0);
        public static final double WRIST_EXTENDED_SETPOINT_IN_ROTATIONS = 0.29;

        public static final int TOP_INTAKE_CURRENT_LIMIT_FREE_IN_AMPS = 8;
        public static final int TOP_INTAKE_CURRENT_LIMIT_STALLED_IN_AMPS = 8;
        public static final int BOTTOM_INTAKE_CURRENT_LIMIT_FREE_IN_AMPS = 8;
        public static final int BOTTOM_INTAKE_CURRENT_LIMIT_STALLED_IN_AMPS = 8;

        public static final double INTAKE_IN_TOP_POWER = 0.55;
        public static final double INTAKE_IN_BOTTOM_POWER = 0.4;
        public static final double INTAKE_OUT_POWER = -0.5;

        public static class CANIds {
            public static final int TOP_INTAKE = 6;
            public static final int BOTTOM_INTAKE = 7;
            public static final int WRIST = 8;
        }
    }

    public static class Shooter {
        //public static final double SHOOTER_ROTATION_GEAR_RATIO = (10.0 / 64.0);
        public static final double SHOOTER_ROTATION_GEAR_RATIO = (1.0/198.0);
        public static final double SHOOTER_WHEELS_GEAR_RATIO = (10.0 / 64.0);


        public static final boolean SHOOTER_TOP_INVERTED = true;
        public static final boolean SHOOTER_BOTTOM_INVERTED = true;
        public static final boolean SHOOTER_ROTATION_INVERTED = true;

        public static final double SHOOTER_ROTATION_PID_KP = 14;
        public static final double SHOOTER_ROTATION_PID_KI = 0.08;
        public static final double SHOOTER_ROTATION_PID_KD = 0.18;

        public static final double SHOOTER_ROTATION_STARTUP_POSITION = 0;
        public static final double SHOOTER_ROTATION_MANUAL_ADJUST_START_DEGREES = 5.0;



        public static final double SHOOTER_FORWARD_SPEED = 1.0;
        public static double SHOOTER_FORWARD_SLOW_SPEED = 0.25;
        public static final double SHOOTER_REVERSE_SPEED = -0.2;

        public static class CANIds {
            public static final int TOP_SHOOTER = 17;
            public static final int BOTTOM_SHOOTER = 18;
            public static final int SHOOTER_ROTATION = 9;
        }
    }

    public static class Climber {
        public static final double LEFT_CLIMBER_GEAR_RATIO  = 1;//(8.0 / 54.0);
        public static final double RIGHT_CLIMBER_GEAR_RATIO = 1;//(8.0 / 54.0);

        public static final boolean CLIMBER_LEFT_INVERTED = false;
        public static final boolean CLIMBER_RIGHT_INVERTED = true;

        public static final double CALIBRATION_DELAY_MS = 500;

        public static final int RIGHT_CLIMBER_DIGITAL_SWITCH_PORT = 1;
        public static final int LEFT_CLIMBER_DIGITAL_SWITCH_PORT = 0;


        public static final double CLIMBER_PID_KP = 0.1;
        // MJB TESTING public static final double CLIMBER_PID_KP = 0.2;
        public static final double CLIMBER_PID_KI = 0.0;
        public static final double CLIMBER_PID_KD = 0.0;
        // MJB WAS public static final double CLIMBER_PID_MAX_SPEED_IN_ROTATIONS_PER_SECOND = 100.0;
        public static final double CLIMBER_PID_MAX_SPEED_IN_ROTATIONS_PER_SECOND = 200.0;
        public static final double CLIMBER_PID_MAX_ACCELERATION_IN_ROTATIONS_PER_SECOND_SQUARED = 100.0;

        public static final double LEFT_CLIMBER_STARTUP_POSITION = 0.0;
        public static final double RIGHT_CLIMBER_STARTUP_POSITION = 0.0;

        public static final double CLIMBER_DOWN_POSITION = 0.0;
        public static final double CLIMBER_UP_POSITION = 120.0;

        public static class CANIds {
            public static final int LEFT_CLIMBER = 19;
            public static final int RIGHT_CLIMBER = 20;
        }
    }

    public static class Limelight {
        public static final double FIELD_WIDTH_IN_METERS = 16.4592;
        public static final double FIELD_HEIGHT_IN_METERS = 8.21055;
    }
}
