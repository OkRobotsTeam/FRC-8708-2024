package frc.robot;

public class Constants {

    public static final double WHEELBASE_METERS = 0.56515;

    public static final boolean FIELD_ORIENTED = false;
    public static final double CONTROLLER_DEADBAND = 0.1d;

    // The amount of seconds it should take to accelerate from 0% to 100% speed
    public static final double MOVEMENT_MAX_ACCELERATION_METERS_PER_SECOND = 0.0d;

    // The amount of seconds it should take to accelerate from 0.0 to 1.0 radians per second
    public static final double TURNING_MAX_ACCELERATION_RADIANS_PER_SECOND = 0.0d;

    public static final int FRONT_LEFT_DRIVE_MOTOR_CAN_ID = 2;
    public static final int FRONT_LEFT_ROTATION_MOTOR_CAN_ID = 3;
    public static final int FRONT_LEFT_ENCODER_ENCODER_CAN_ID = 10;

    public static final int FRONT_RIGHT_DRIVE_MOTOR_CAN_ID = 4;
    public static final int FRONT_RIGHT_ROTATION_MOTOR_CAN_ID = 5;
    public static final int FRONT_RIGHT_ENCODER_ENCODER_CAN_ID = 11;

    public static final int BACK_LEFT_DRIVE_MOTOR_CAN_ID = 6;
    public static final int BACK_LEFT_ROTATION_MOTOR_CAN_ID = 7;
    public static final int BACK_LEFT_ROTATION_ENCODER_CAN_ID = 12;

    public static final int BACK_RIGHT_DRIVE_MOTOR_CAN_ID = 8;
    public static final int BACK_RIGHT_ROTATION_MOTOR_CAN_ID = 9;
    public static final int BACK_RIGHT_ROTATION_ENCODER_CAN_ID = 13;
}
