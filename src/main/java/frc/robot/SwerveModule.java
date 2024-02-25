package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.SwerveDrivetrain;

import java.util.Objects;


public class SwerveModule {
    private static final double MODULE_MAX_ANGULAR_VELOCITY = SwerveDrivetrain.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
    private static final double MODULE_MAX_ANGULAR_ACCELERATION = 2 * Math.PI; // radians per second squared
    private final TalonFX driveMotor;
    private final CANSparkMax turningMotor;
    private final CANcoder turningEncoder;
    private final PIDController drivePIDController = new PIDController(1, 0, 0);
    private final ProfiledPIDController turningPIDController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(MODULE_MAX_ANGULAR_VELOCITY, MODULE_MAX_ANGULAR_ACCELERATION));
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(1, 0.5);
    private final String name;
    private int i;



    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
     *
     * @param driveMotorCANID       CAN ID for the drive motor.
     * @param turningMotorCANID     CAN ID for the turning motor.
     * @param turningEncoderCANID   CAN ID for turning encoder.
     */
    public SwerveModule(int driveMotorCANID, int turningMotorCANID, int turningEncoderCANID, String nameIn) {
        name = nameIn;
        driveMotor = new TalonFX(driveMotorCANID, "CTRE_BUS");
        turningMotor = new CANSparkMax(turningMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
        turningMotor.setInverted(true);

        turningEncoder = new CANcoder(turningEncoderCANID, "CTRE_BUS");

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }


    private double getRotationRadians() {
        double rotationRevolutions = turningEncoder.getPosition().getValueAsDouble();

        return rotationRevolutions * (Math.PI * 2);
    }

    private double getDistanceMeters() {
        double drivePositionRotations = driveMotor.getPosition().getValueAsDouble();

        return drivePositionRotations * Constants.Drivetrain.WHEEL_CIRCUMFERENCE_METERS;
    }


    private double getVelocityMetersPerSecond() {
        double driveVelocityRPM = driveMotor.getVelocity().getValueAsDouble();

        double driveVelocityRPS = driveVelocityRPM / 60;

        return driveVelocityRPS * Constants.Drivetrain.WHEEL_CIRCUMFERENCE_METERS;
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
        if (i++%10==0 && Objects.equals(name, "FL")) {
            System.out.printf("%2.2f %2.2f\n",  Math.toDegrees(getRotationRadians()), optimizedDesiredState.angle.getDegrees());
        }
        final double turnFeedforward = this.turnFeedforward.calculate(turningPIDController.getSetpoint().velocity);

        driveMotor.setVoltage(driveOutput + driveFeedforward);
        turningMotor.setVoltage(turnOutput + turnFeedforward);
    }
    public void setDriveMotorBraking(boolean braking) {
        if (braking) {
            driveMotor.setNeutralMode(NeutralModeValue.Brake);
        } else {
            driveMotor.setNeutralMode(NeutralModeValue.Coast);
        }
    }
}
