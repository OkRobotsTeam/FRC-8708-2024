// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;


/**
 * Represents a swerve drive style drivetrain.
 */
public class SwerveDrivetrain {
    public static final double MAX_SPEED_METERS_PER_SECOND = 3.0;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;

    private final Translation2d frontLeftLocation = new Translation2d(Constants.WHEELBASE_METERS/2, Constants.WHEELBASE_METERS/2);
    private final Translation2d frontRightLocation = new Translation2d(Constants.WHEELBASE_METERS/2, -Constants.WHEELBASE_METERS/2);
    private final Translation2d backLeftLocation = new Translation2d(-Constants.WHEELBASE_METERS/2, Constants.WHEELBASE_METERS/2);
    private final Translation2d backRightLocation = new Translation2d(-Constants.WHEELBASE_METERS/2, -Constants.WHEELBASE_METERS/2);

    private final SwerveModule frontLeft = new SwerveModule(Constants.FRONT_LEFT_DRIVE_MOTOR, Constants.FRONT_LEFT_ROTATION_MOTOR, Constants.FRONT_LEFT_DRIVE_ENCODER_CHANNEL_A, Constants.FRONT_LEFT_DRIVE_ENCODER_CHANNEL_B, Constants.FRONT_LEFT_ROTATION_ENCODER_CHANNEL_A, Constants.FRONT_LEFT_ROTATION_ENCODER_CHANNEL_B);
    private final SwerveModule frontRight = new SwerveModule(Constants.FRONT_RIGHT_DRIVE_MOTOR, Constants.FRONT_RIGHT_ROTATION_MOTOR, Constants.FRONT_RIGHT_DRIVE_ENCODER_CHANNEL_A, Constants.FRONT_RIGHT_DRIVE_ENCODER_CHANNEL_B, Constants.FRONT_RIGHT_ROTATION_ENCODER_CHANNEL_A, Constants.FRONT_RIGHT_ROTATION_ENCODER_CHANNEL_B);
    private final SwerveModule backLeft = new SwerveModule(Constants.BACK_LEFT_DRIVE_MOTOR, Constants.BACK_LEFT_ROTATION_MOTOR, Constants.BACK_LEFT_DRIVE_ENCODER_CHANNEL_A, Constants.BACK_LEFT_DRIVE_ENCODER_CHANNEL_B, Constants.BACK_LEFT_ROTATION_ENCODER_CHANNEL_A, Constants.BACK_LEFT_ROTATION_ENCODER_CHANNEL_B);
    private final SwerveModule backRight = new SwerveModule(Constants.BACK_RIGHT_DRIVE_MOTOR, Constants.BACK_RIGHT_ROTATION_MOTOR, Constants.BACK_RIGHT_DRIVE_ENCODER_CHANNEL_A, Constants.BACK_RIGHT_DRIVE_ENCODER_CHANNEL_B, Constants.BACK_RIGHT_ROTATION_ENCODER_CHANNEL_A, Constants.BACK_RIGHT_ROTATION_ENCODER_CHANNEL_B);

    private final AnalogGyro gyro = new AnalogGyro(0);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});


    public SwerveDrivetrain() {
        gyro.reset();
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }


    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rotation      Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
        var swerveModuleStates = kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, gyro.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }


    /**
     * Updates the field relative position of the robot.
     */
    public void updateOdometry() {
        odometry.update(gyro.getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});
    }
}
