package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Shooter.*;


public class Shooter {
    private final TalonFX topShooter = new TalonFX(CANIds.TOP_SHOOTER, "CTRE");
    private final TalonFX bottomShooter = new TalonFX(CANIds.BOTTOM_SHOOTER, "CTRE");
    private final CANSparkMax shooterRotation = new CANSparkMax(CANIds.SHOOTER_ROTATION, kBrushless);

    private final RelativeEncoder shooterRotationEncoder = shooterRotation.getEncoder();
    private final PIDController shooterRotationPID = new PIDController(SHOOTER_ROTATION_PID_KP, SHOOTER_ROTATION_PID_KI, SHOOTER_ROTATION_PID_KD);

    private int adjustment = 0;

    public Shooter() {
        topShooter.setInverted(SHOOTER_TOP_INVERTED);
        bottomShooter.setInverted(SHOOTER_BOTTOM_INVERTED);

        shooterRotationEncoder.setPositionConversionFactor(SHOOTER_ROTATION_GEAR_RATIO);
        shooterRotationEncoder.setVelocityConversionFactor(SHOOTER_ROTATION_GEAR_RATIO);

        shooterRotationEncoder.setPosition(SHOOTER_ROTATION_STARTUP_POSITION);
        shooterRotationPID.setSetpoint(SHOOTER_ROTATION_STARTUP_POSITION);
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
        SmartDashboard.putNumber("Shooter Angle", getTargetShooterDegreesFromHorizon());
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
            setTargetShooterDegreesFromHorizon(SHOOTER_ROTATION_MANUAL_ADJUST_START_DEGREES + (adjustment * 5));
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
        if (adjustment == -1 && shooterRotationPID.atSetpoint()) {
            // If we are in the docked position (position -1), we want to lock the shooter
            SmartDashboard.putNumber("Shooter Angle Current", 0);
            setShooterRotationBraking(true);
        } else {
            setShooterRotationBraking(false);
            double PIDOutput = shooterRotationPID.calculate(getShooterRotationPositionInRotations());
            double gravityCompensationCoefficient = (getTargetShooterDegreesFromHorizon() / 90);

            shooterRotation.set(PIDOutput + gravityCompensationCoefficient);
        }
    }
}
