package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import java.util.Optional;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Shooter.*;


public class Shooter {
    private final TalonFX topShooter = new TalonFX(CANIds.TOP_SHOOTER, "CTRE");
    private final TalonFX bottomShooter = new TalonFX(CANIds.BOTTOM_SHOOTER, "CTRE");
    private final CANSparkMax shooterRotation = new CANSparkMax(CANIds.SHOOTER_ROTATION, kBrushless);

    private final RelativeEncoder shooterRotationEncoder = shooterRotation.getEncoder();
    private PIDController shooterRotationPID = new PIDController(SHOOTER_ROTATION_PID_KP, SHOOTER_ROTATION_PID_KI, SHOOTER_ROTATION_PID_KD);
    // Create a new ArmFeedforward with gains kS, kG, kV, and kA
//    ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV, kA);
    private GenericEntry shooterAngleEntry;
    public int adjustment = -1;
    private int resetCount = 0;

    public Shooter(GenericEntry shooterAngleEntry) {
        this.shooterAngleEntry = shooterAngleEntry;

        topShooter.setInverted(SHOOTER_TOP_INVERTED);
        bottomShooter.setInverted(SHOOTER_BOTTOM_INVERTED);

        shooterRotationEncoder.setPositionConversionFactor(SHOOTER_ROTATION_GEAR_RATIO);
        shooterRotationEncoder.setVelocityConversionFactor(SHOOTER_ROTATION_GEAR_RATIO);

        shooterRotationEncoder.setPosition(SHOOTER_ROTATION_STARTUP_POSITION);
        shooterRotationPID.setSetpoint(SHOOTER_ROTATION_STARTUP_POSITION);

        shooterRotationPID.setTolerance(0.01);

//        SmartDashboard.putData("Rotation PID: ", shooterRotationPID);
    }

    public void init() {
        adjustment = -1;
        setTargetShooterDegreesFromHorizon(0.0);
        shooterRotationPID.reset();
        shooterRotationPID = new PIDController(SHOOTER_ROTATION_PID_KP, SHOOTER_ROTATION_PID_KI, SHOOTER_ROTATION_PID_KD);

        resetCount++;
//        SmartDashboard.putNumber("Reset count: ", resetCount);

    }

    public double getShooterRotationPositionInRotations() {
        return shooterRotationEncoder.getPosition();
    }

    public void setTopShooterSpeed(double speed) {
        topShooter.set(speed);
    }

    public void setBottomShooterSpeed(double speed) {
        bottomShooter.set(speed);
    }

    public void setShooterSpeed(double speed) {
        setTopShooterSpeed(speed);
        setBottomShooterSpeed(speed);
    }

    public void runShooterForward() {
        setShooterSpeed(SHOOTER_FORWARD_SPEED);
        System.out.println("Info: Running shooter forward");
    }

    public void runShooterSlow() {
        setShooterSpeed(SHOOTER_FORWARD_SLOW_SPEED);
        System.out.println("Info: Running shooter forward slow");
    }
    
    public void runShooterBackward() {
        setShooterSpeed(SHOOTER_REVERSE_SPEED);
        System.out.println("Info: Running shooter backward");
    }

    public void stopShooter() {
        setShooterSpeed(0.0);
        System.out.println("Info: Stopping shooter");
    }

    public void setTargetShooterDegreesFromHorizon(double angle) {
        shooterRotationPID.setSetpoint(angle / 360.0 + SHOOTER_ROTATION_STARTUP_POSITION);
//        SmartDashboard.putNumber("Shooter Angle", getTargetShooterDegreesFromHorizon());
    }

    public double getTargetShooterDegreesFromHorizon() {
        return (shooterRotationPID.getSetpoint() - SHOOTER_ROTATION_STARTUP_POSITION) * 360.0;

    }

    public void shooterRotationReset() {
        setTargetShooterDegreesFromHorizon(0);
        adjustment = 0;
    }

    public void shooterManualAdjustUp() {
        adjustment++;
        updateShooterManualAdjustment();
    }

    public void shooterManualAdjustDown() {
        adjustment--;
        updateShooterManualAdjustment();
    }

    public void updateShooterManualAdjustment() {
        if (adjustment < -1) {
            adjustment = -1;
        }
        if (adjustment == -1) {
            setTargetShooterDegreesFromHorizon(0.0);
        } else {
            setTargetShooterDegreesFromHorizon(SHOOTER_ROTATION_MANUAL_ADJUST_START_DEGREES + (adjustment * 5.0));
        }
    }

    public void setShooterRotationBraking(boolean braking) {
        if (braking) {
            shooterRotation.setIdleMode(CANSparkBase.IdleMode.kBrake);
        } else {
            shooterRotation.setIdleMode(CANSparkBase.IdleMode.kCoast);
        }
    }

    public void tickShooterRotation() {
//        if (adjustment == -1 && shooterRotationPID.atSetpoint()) {
//            // If we are in the docked position (position -1), we want to lock the shooter
////            SmartDashboard.putNumber("Shooter Angle Current", 0);
//            setShooterRotationBraking(true);
//        } else {
            setShooterRotationBraking(false);
            double PIDOutput = shooterRotationPID.calculate(getShooterRotationPositionInRotations());
            PIDOutput= PIDOutput * 0.6;
            PIDOutput = Math.min(0.2,PIDOutput);
            PIDOutput = Math.max(-0.2,PIDOutput);

            double gravityCompensationCoefficient = Math.sin(Units.degreesToRadians(getTargetShooterDegreesFromHorizon())) * 0.07;


            SmartDashboard.putNumber("Gravity compensation: ", (gravityCompensationCoefficient));
            SmartDashboard.putNumber("PID Output: ", PIDOutput);

            PIDOutput = PIDOutput + gravityCompensationCoefficient;


            shooterRotation.set(PIDOutput);
//        }

        shooterAngleEntry.setDouble(Math.round(shooterRotationEncoder.getPosition() * 360));

        SmartDashboard.putNumber("Rotation motor position: ", Math.round(shooterRotationEncoder.getPosition() * 360));
        SmartDashboard.putNumber("Rotation PID setpoint: ", Math.round(shooterRotationPID.getSetpoint() * 360));
    }

    public void autoAngle(Limelight limelight) {
        Optional<Translation2d> goalOffset = limelight.getOffsetFromGoalInMeters();

        if (goalOffset.isPresent()) {

            double goalDistanceInMeters = goalOffset.get().getNorm();
            Rotation2d angleFromRobotToGoal = goalOffset.get().getAngle();

            double GOAL_HEIGHT_METERS = 2.2;
            double SHOOTER_HEIGHT = 0.4826;

            Rotation2d angleFromShooterToGoal = Rotation2d.fromRadians(Math.atan((GOAL_HEIGHT_METERS - SHOOTER_HEIGHT) / goalDistanceInMeters));

//            SmartDashboard.putString("Offset from Goal (in): ", goalOffset.get().times(39.3701).toString());
//            SmartDashboard.putNumber("Distance from Goal (in): ", Units.metersToInches(goalDistanceInMeters));
//            SmartDashboard.putNumber("Angle to goal from robot (deg): ", angleFromRobotToGoal.getDegrees());
//            SmartDashboard.putNumber("Shooter angle (deg): ", angleFromShooterToGoal.getDegrees());

            setTargetShooterDegreesFromHorizon(angleFromShooterToGoal.getDegrees());
        }
    }
}
