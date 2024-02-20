package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;

public class Intake {
    private final CANSparkMax topIntake = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax bottomIntake = new CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax wrist = new CANSparkMax(2, CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder wristEncoder = wrist.getEncoder();
    private final PIDController wristPID = new PIDController(1.0, 0.0, 0.0);

    public Intake() {
        topIntake.setInverted(false);
        bottomIntake.setInverted(true);
        wrist.setInverted(false);

        wristEncoder.setPositionConversionFactor((8.0 / 60.0) * (16.0 / 60.0) * (12.0 / 36.0));
        wristEncoder.setVelocityConversionFactor((8.0 / 60.0) * (16.0 / 60.0) * (12.0 / 36.0));

        wristEncoder.setPosition(0.0);
        wristPID.setSetpoint(0.0);

        topIntake.setSmartCurrentLimit(2, 10);
        bottomIntake.setSmartCurrentLimit(2, 10);
    }

    void runTopIntake(double power) {
        topIntake.set(power);
    }

    void runBottomIntake(double power) {
        bottomIntake.set(power);
    }

    void runIntake(double speed) {
        runTopIntake(speed);
        runBottomIntake(speed);
    }

    void runIntakeIn() {
        runIntake(1.0);
    }

    void runIntakeOut() {
        runIntake(-1.0);
    }

    void extendWrist() {
        wristPID.setSetpoint(0.5); // Rotations
    }

    void foldWrist() {
        wristPID.setSetpoint(0.0);
    }

    double getWristPositionInRotations() {
        return wristEncoder.getPosition();
    }

    void tickWrist() {
        double PIDOutput = wristPID.calculate(getWristPositionInRotations());
        wrist.set(PIDOutput);
    }
}
