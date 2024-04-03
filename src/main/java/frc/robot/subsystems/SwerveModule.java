package frc.robot.subsystems;

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
import frc.robot.Constants;
import frc.robot.Debug;

import static frc.robot.Constants.SwerveDrivetrain.*;


public class SwerveModule {
    private final TalonFX driveMotor;
    private final CANSparkMax turningMotor;
    private final CANcoder turningEncoder;

    private final PIDController drivePIDController = new PIDController(1, 0, 0);
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.13, 2);
    
    private final ProfiledPIDController turningPIDController = new ProfiledPIDController(30, 0, 0.05, new TrapezoidProfile.Constraints(WHEEL_MAX_ANGULAR_VELOCITY_IN_RADIANS_PER_SECOND_SQUARED, WHEEL_MAX_ANGULAR_ACCELERATION_IN_RADIANS_PER_SECOND_SQUARED));
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(0.5, 2);
    
    
    private final String name;


    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
     *
     * @param driveMotorCANID       CAN ID for the drive motor.
     * @param turningMotorCANID     CAN ID for the turning motor.
     * @param turningEncoderCANID   CAN ID for turning encoder.
     */
    public SwerveModule(int driveMotorCANID, int turningMotorCANID, int turningEncoderCANID, String name) {
        this.name = name;

        driveMotor = new TalonFX(driveMotorCANID);
        driveMotor.setInverted(DRIVE_MOTORS_INVERTED);

        turningMotor = new CANSparkMax(turningMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
        turningMotor.setInverted(TURNING_MOTORS_INVERTED);

        turningEncoder = new CANcoder(turningEncoderCANID);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }


    public Rotation2d getRotation() {
        double rotationRevolutions = turningEncoder.getPosition().getValueAsDouble();

        return Rotation2d.fromRotations(rotationRevolutions);
    }

    private double getDistanceMeters() {
        double drivePositionRotations = driveMotor.getPosition().getValueAsDouble();
        return drivePositionRotations * DRIVE_GEAR_RATIO * Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE_IN_METERS;
    }


    private double getVelocityMetersPerSecond() {
        double driveVelocityRPS = driveMotor.getVelocity().getValueAsDouble();
        return driveVelocityRPS * DRIVE_GEAR_RATIO * Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE_IN_METERS;
    }


    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSecond(), getRotation());
    }


    /**import edu.wpi.first.math.controller.PIDController;

     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistanceMeters(), getRotation());
    }


    /**
     * Stops the module
     */
    public void stop() {
        driveMotor.setVoltage(0);
        turningMotor.setVoltage(0);
    }

     String fmt(double num) {
        return Debug.sixPlaces.format(num);
    }

    public void testSetTurnMotor(double speed) {
        speed = speed * 2;
        turningMotor.setVoltage(speed);
        //System.out.println("Turning motor speed:" + fmt(speed)) ;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, getRotation());

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = drivePIDController.calculate(getVelocityMetersPerSecond(), optimizedDesiredState.speedMetersPerSecond);

        final double driveFeedforward = this.driveFeedforward.calculate(optimizedDesiredState.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        double turnFF = turnFeedforward.calculate(optimizedDesiredState.angle.minus(getRotation()).getRadians());
        double turnOutput = turningPIDController.calculate(getRotation().getRadians(), optimizedDesiredState.angle.getRadians());

        if (name == "FL") {
            // if (optimizedDesiredState.speedMetersPerSecond > 0) {
            //     Debug.debugPrint(name, " FF:" + fmt(driveFeedforward) + " + DO:" + fmt(driveOutput) + " CS:"
            //             + fmt(getVelocityMetersPerSecond()) + " DS:" + fmt(optimizedDesiredState.speedMetersPerSecond) +
            //             " Ratio: " + fmt(getVelocityMetersPerSecond() / optimizedDesiredState.speedMetersPerSecond));
            // }

            // //Debug.debugPrint("Voltage",
            // fmt(driveMotor.getMotorVoltage().getValueAsDouble()));
            // //Debug.debugPrint("RPS", fmt(driveMotor.getVelocity().getValueAsDouble()));
            
            // System.out.println("Turning" 
            //         + " diff: " + fmt(optimizedDesiredState.angle.minus(getRotation()).getDegrees())
            //         + " PID: " + fmt(turnOutput)
            //         + " FF: " + fmt(turnFF));
            

        }


        turnOutput = turnOutput / 4;
        //turningMotor.setVoltage(turnOutput);
        turningMotor.set(turnOutput/12);
        driveMotor.set( (driveOutput + driveFeedforward) / 12);
        //driveMotor.setVoltage(driveOutput);
        //System.out.println("Setting turning motor voltage to " + turnOutput);
        
    }
    public void setDriveMotorBraking(boolean braking) {
        if (braking) {
            driveMotor.setNeutralMode(NeutralModeValue.Brake);
        } else {
            driveMotor.setNeutralMode(NeutralModeValue.Coast);
        }
    }
}
