package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;

public class Shooter {
    private final TalonFX topShooter = new TalonFX(1);
    private final TalonFX bottomShooter = new TalonFX(2);
    private final CANSparkMax shooterRotation = new CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder shooterRotationEncoder = shooterRotation.getEncoder();
    private final PIDController shooterRotationPID = new PIDController(1.0, 0.0, 0.0);

    public Shooter() {
        topShooter.setInverted(false);
        bottomShooter.setInverted(false);

        shooterRotationEncoder.setPositionConversionFactor(10.0 / 64.0);
        shooterRotationEncoder.setVelocityConversionFactor(10.0 / 64.0);

        shooterRotationEncoder.setPosition(0);
        shooterRotationPID.setSetpoint(0);
    }

    public double getTopPositionInRotations() {
        return topShooter.getPosition().getValueAsDouble() * (10.0 / 64.0);
    }

    public double getBottomPositionInRotations() {
        return topShooter.getPosition().getValueAsDouble() * (10.0 / 64.0);
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
        setShooterSpeed(1.0);
    }

    public void runShooterBackward() {
        setShooterSpeed(-0.5);
    }

    public void stopShooter() {
        setShooterSpeed(0.0);
    }

    public void setTargetShooterDegreesFromHorizon(double angle) {
        shooterRotationPID.setSetpoint(angle / 360.0);
    }

    public void tickShooterRotation() {
        double PIDOutput = shooterRotationPID.calculate(getShooterRotationPositionInRotations());
        shooterRotation.set(PIDOutput);
    }
}
