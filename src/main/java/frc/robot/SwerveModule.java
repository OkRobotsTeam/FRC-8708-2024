// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;


public class SwerveModule {
    private static final double WHEEL_RADIUS_METERS = 0.0508;
    private static final int ENCODER_RESOLUTION_TICKS_PER_REV = 4096;

    private static final double MODULE_MAX_ANGULAR_VELOCITY = SwerveDrivetrain.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
    private static final double MODULE_MAX_ANGULAR_ACCELERATION = 2 * Math.PI; // radians per second squared

    private final TalonFX driveMotor;
    private final CANSparkMax turningMotor;

    private final CANcoder turningEncoder;

    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController drivePIDController = new PIDController(1, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController turningPIDController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(MODULE_MAX_ANGULAR_VELOCITY, MODULE_MAX_ANGULAR_ACCELERATION));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(1, 0.5);


    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
     *
     * @param driveMotorCANID       CAN ID for the drive motor.
     * @param turningMotorCANID     CAN ID for the turning motor.
     * @param turningEncoderCANID   CAN ID for turning encoder.
     */
    public SwerveModule(int driveMotorCANID, int turningMotorCANID, int turningEncoderCANID) {
        driveMotor = new TalonFX(driveMotorCANID);
        turningMotor = new CANSparkMax(turningMotorCANID, CANSparkLowLevel.MotorType.kBrushless);

        turningEncoder = new CANcoder(turningEncoderCANID);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }


    private double getRotationRadians() {
        double rotationRevolutions = turningEncoder.getPosition().getValueAsDouble();

        double rotationRadians = rotationRevolutions * (Math.PI * 2);

        return rotationRadians;
    }

    private double getDistanceMeters() {
        double drivePositionRotations = driveMotor.getPosition().getValueAsDouble();

        double drivePositionMeters = drivePositionRotations * (2 * Math.PI) * WHEEL_RADIUS_METERS;

        return drivePositionMeters;
    }


    private double getVelocityMetersPerSecond() {
        double driveVelocityRPM = driveMotor.getVelocity().getValueAsDouble();

        double driveVelocityRPS = driveVelocityRPM / 60;

        double driveVelocityMetersPerSec = driveVelocityRPS * (2 * Math.PI) * WHEEL_RADIUS_METERS;

        return driveVelocityMetersPerSec;
    }


    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSecond(), new Rotation2d(getRotationRadians()));
    }


    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistanceMeters(), new Rotation2d(getRotationRadians()));
    }


    /**
     * Stops the module
     */
    public void stop() {
        driveMotor.setVoltage(0);
        turningMotor.setVoltage(0);
    }


    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(getRotationRadians()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = drivePIDController.calculate(getVelocityMetersPerSecond(), optimizedDesiredState.speedMetersPerSecond);

        final double driveFeedforward = this.driveFeedforward.calculate(optimizedDesiredState.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = turningPIDController.calculate(getRotationRadians(), optimizedDesiredState.angle.getRadians());

        final double turnFeedforward = this.turnFeedforward.calculate(turningPIDController.getSetpoint().velocity);

        driveMotor.setVoltage(driveOutput + driveFeedforward);
        turningMotor.setVoltage(turnOutput + turnFeedforward);
    }
}
