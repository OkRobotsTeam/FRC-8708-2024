package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Shooter.*;


public class Shooter {
    private final TalonFX topShooter = new TalonFX(CANIds.TOP_SHOOTER, "CTRE");
    private final TalonFX bottomShooter = new TalonFX(CANIds.BOTTOM_SHOOTER, "CTRE");
    private final CANSparkMax shooterRotation = new CANSparkMax(CANIds.SHOOTER_ROTATION, kBrushless);

    private final RelativeEncoder shooterRotationEncoder = shooterRotation.getEncoder();
    private final PIDController shooterRotationPID = new PIDController(SHOOTER_ROTATION_PID_KP, SHOOTER_ROTATION_PID_KI, SHOOTER_ROTATION_PID_KD);

    public Shooter() {
        topShooter.setInverted(SHOOTER_TOP_INVERTED);
        bottomShooter.setInverted(SHOOTER_BOTTOM_INVERTED);

        shooterRotationEncoder.setPositionConversionFactor(SHOOTER_ROTATION_GEAR_RATIO);
        shooterRotationEncoder.setVelocityConversionFactor(SHOOTER_ROTATION_GEAR_RATIO);

        shooterRotation.setIdleMode(CANSparkBase.IdleMode.kBrake);

        shooterRotationEncoder.setPosition(SHOOTER_ROTATION_STARTUP_POSITION);
        shooterRotationPID.setSetpoint(SHOOTER_ROTATION_STARTUP_POSITION);
    }

    public double getTopPositionInRotations() {
        return topShooter.getPosition().getValueAsDouble() * SHOOTER_WHEELS_GEAR_RATIO;
    }

    public double getBottomPositionInRotations() {
        return bottomShooter.getPosition().getValueAsDouble() * SHOOTER_WHEELS_GEAR_RATIO;
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

    public void runShooterBackward() {
        setShooterSpeed(SHOOTER_REVERSE_SPEED);
        System.out.println("Info: Running shooter backward");
    }

    public void stopShooter() {
        setShooterSpeed(0.0);
        System.out.println("Info: Stopping shooter");
    }

    public void setTargetShooterDegreesFromHorizon(double angle) {
        shooterRotationPID.setSetpoint(angle / 360.0);
    }

    public void shooterRotationReset() {
        setTargetShooterDegreesFromHorizon(0);
    }

    public void shooterManualAdjustUp() {
        shooterRotationPID.setSetpoint(shooterRotationPID.getSetpoint() + (15/360.0));
    }

    public void shooterManualAdjustDown() {
        shooterRotationPID.setSetpoint(shooterRotationPID.getSetpoint() - (15/360.0));
    }

    public void tickShooterRotation() {
        double PIDOutput = shooterRotationPID.calculate(getShooterRotationPositionInRotations());
        shooterRotation.set(PIDOutput);
    }
}
