// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.SwerveModule;


/**
 * Represents a swerve drive style drivetrain.
 */
public class SwerveDrivetrain extends SubsystemBase {
    public static final double MAX_SPEED_METERS_PER_SECOND = 3.0;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;

    private final Translation2d frontLeftLocation = new Translation2d(Constants.Drivetrain.WHEELBASE_METERS/2, Constants.Drivetrain.WHEELBASE_METERS/2);
    private final Translation2d frontRightLocation = new Translation2d(Constants.Drivetrain.WHEELBASE_METERS/2, -Constants.Drivetrain.WHEELBASE_METERS/2);
    private final Translation2d backLeftLocation = new Translation2d(-Constants.Drivetrain.WHEELBASE_METERS/2, Constants.Drivetrain.WHEELBASE_METERS/2);
    private final Translation2d backRightLocation = new Translation2d(-Constants.Drivetrain.WHEELBASE_METERS/2, -Constants.Drivetrain.WHEELBASE_METERS/2);

    private final SwerveModule frontLeft = new SwerveModule(Constants.Drivetrain.CANIds.FRONT_LEFT_DRIVE_MOTOR_CAN_ID, Constants.Drivetrain.CANIds.FRONT_LEFT_ROTATION_MOTOR_CAN_ID, Constants.Drivetrain.CANIds.FRONT_LEFT_ENCODER_ENCODER_CAN_ID, "FL");
    private final SwerveModule frontRight = new SwerveModule(Constants.Drivetrain.CANIds.FRONT_RIGHT_DRIVE_MOTOR_CAN_ID, Constants.Drivetrain.CANIds.FRONT_RIGHT_ROTATION_MOTOR_CAN_ID, Constants.Drivetrain.CANIds.FRONT_RIGHT_ENCODER_ENCODER_CAN_ID, "FR");
    private final SwerveModule backLeft = new SwerveModule(Constants.Drivetrain.CANIds.BACK_LEFT_DRIVE_MOTOR_CAN_ID, Constants.Drivetrain.CANIds.BACK_LEFT_ROTATION_MOTOR_CAN_ID, Constants.Drivetrain.CANIds.BACK_LEFT_ROTATION_ENCODER_CAN_ID, "BL");
    private final SwerveModule backRight = new SwerveModule(Constants.Drivetrain.CANIds.BACK_RIGHT_DRIVE_MOTOR_CAN_ID, Constants.Drivetrain.CANIds.BACK_RIGHT_ROTATION_MOTOR_CAN_ID, Constants.Drivetrain.CANIds.BACK_RIGHT_ROTATION_ENCODER_CAN_ID, "BR");

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});

    // Slew rate limiters to make joystick inputs less abrupt
    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(1d / Constants.Drivetrain.MOVEMENT_MAX_ACCELERATION_METERS_PER_SECOND);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(1d / Constants.Drivetrain.MOVEMENT_MAX_ACCELERATION_METERS_PER_SECOND);
    private final SlewRateLimiter rotateLimiter = new SlewRateLimiter(1d / Constants.Drivetrain.TURNING_MAX_ACCELERATION_RADIANS_PER_SECOND);



    public SwerveDrivetrain() {
        resetGyro();
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void driveWithController(CommandXboxController controller) {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -xSpeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftX(), Constants.Drivetrain.CONTROLLER_DEADBAND)) * SwerveDrivetrain.MAX_SPEED_METERS_PER_SECOND;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. The Xbox controller
        // returns positive values when you pull to the right by default.
        final var ySpeed = -ySpeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), Constants.Drivetrain.CONTROLLER_DEADBAND)) * SwerveDrivetrain.MAX_SPEED_METERS_PER_SECOND;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). The Xbox controller returns positive values when you pull to
        // the right by default.
        final var rot = -rotateLimiter.calculate(MathUtil.applyDeadband(controller.getRightX(), Constants.Drivetrain.CONTROLLER_DEADBAND)) * SwerveDrivetrain.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        drive(xSpeed, ySpeed, rot, Constants.Drivetrain.FIELD_ORIENTED);
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