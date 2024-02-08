package frc.robot;

public class Constants {

    public static final double WHEELBASE_METERS = 0.56515;

    public static final boolean FIELD_ORIENTED = true;
    public static final double CONTROLLER_DEADBAND = 0.1d;

    // The amount of seconds it should take to accelerate from 0% to 100% speed
    public static final double MOVEMENT_MAX_ACCELERATION_METERS_PER_SECOND = 1d/3d;

    // The amount of seconds it should take to accelerate from 0.0 to 1.0 radians per second
    public static final double TURNING_MAX_ACCELERATION_RADIANS_PER_SECOND = 1d/3d;

    public static final int FRONT_LEFT_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_ROTATION_MOTOR = 2;
    public static final int FRONT_LEFT_DRIVE_ENCODER_CHANNEL_A = 0;
    public static final int FRONT_LEFT_DRIVE_ENCODER_CHANNEL_B = 1;
    public static final int FRONT_LEFT_ROTATION_ENCODER_CHANNEL_A = 2;
    public static final int FRONT_LEFT_ROTATION_ENCODER_CHANNEL_B = 3;

    public static final int FRONT_RIGHT_DRIVE_MOTOR = 3;
    public static final int FRONT_RIGHT_ROTATION_MOTOR = 4;
    public static final int FRONT_RIGHT_DRIVE_ENCODER_CHANNEL_A = 4;
    public static final int FRONT_RIGHT_DRIVE_ENCODER_CHANNEL_B = 5;
    public static final int FRONT_RIGHT_ROTATION_ENCODER_CHANNEL_A = 6;
    public static final int FRONT_RIGHT_ROTATION_ENCODER_CHANNEL_B = 7;

    public static final int BACK_LEFT_DRIVE_MOTOR = 5;
    public static final int BACK_LEFT_ROTATION_MOTOR = 6;
    public static final int BACK_LEFT_DRIVE_ENCODER_CHANNEL_A = 8;
    public static final int BACK_LEFT_DRIVE_ENCODER_CHANNEL_B = 9;
    public static final int BACK_LEFT_ROTATION_ENCODER_CHANNEL_A = 10;
    public static final int BACK_LEFT_ROTATION_ENCODER_CHANNEL_B = 11;

    public static final int BACK_RIGHT_DRIVE_MOTOR = 7;
    public static final int BACK_RIGHT_ROTATION_MOTOR = 8;
    public static final int BACK_RIGHT_DRIVE_ENCODER_CHANNEL_A = 12;
    public static final int BACK_RIGHT_DRIVE_ENCODER_CHANNEL_B = 13;
    public static final int BACK_RIGHT_ROTATION_ENCODER_CHANNEL_A = 14;
    public static final int BACK_RIGHT_ROTATION_ENCODER_CHANNEL_B = 15;
}
