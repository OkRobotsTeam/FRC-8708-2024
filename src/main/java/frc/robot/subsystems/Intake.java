package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Intake.*;
import static java.lang.Double.*;


public class Intake {
    private final CANSparkMax topIntake = new CANSparkMax(CANIds.TOP_INTAKE, kBrushless);
    private final CANSparkMax bottomIntake = new CANSparkMax(CANIds.BOTTOM_INTAKE, kBrushless);
    private final CANSparkMax wrist = new CANSparkMax(CANIds.WRIST, kBrushless);

    private final RelativeEncoder wristEncoder = wrist.getEncoder();
    private final PIDController wristPID = new PIDController(WRIST_PID_KP, WRIST_PID_KI, WRIST_PID_KD);

    public Intake() {
        topIntake.setInverted(TOP_INTAKE_REVERSED);
        bottomIntake.setInverted(BOTTOM_INTAKE_REVERSED);
        wrist.setInverted(WRIST_REVERSED);

        wristEncoder.setPositionConversionFactor(GEAR_RATIO);
        wristEncoder.setVelocityConversionFactor(GEAR_RATIO);

        wristEncoder.setPosition(WRIST_STARTUP_POSITION);
        wristPID.setSetpoint(WRIST_STARTUP_POSITION);

        topIntake.setSmartCurrentLimit(TOP_INTAKE_CURRENT_LIMIT_STALLED_IN_AMPS, TOP_INTAKE_CURRENT_LIMIT_FREE_IN_AMPS);
        bottomIntake.setSmartCurrentLimit(BOTTOM_INTAKE_CURRENT_LIMIT_STALLED_IN_AMPS, BOTTOM_INTAKE_CURRENT_LIMIT_FREE_IN_AMPS);
    }

    public void runTopIntake(double power) {
        topIntake.set(power);
    }

    public void runBottomIntake(double power) {
        bottomIntake.set(power);
    }

    public void runIntake(double speed) {
        runTopIntake(speed * 0.25);
        runBottomIntake(speed);
    }

    public void runIntakeIn() {
        runIntake(INTAKE_IN_POWER);
        System.out.println("Info: Running intake in");
    }

    public void runIntakeOut() {
        runIntake(INTAKE_OUT_POWER);
        System.out.println("Info: Running intake out");
    }

    public void fullSpeedOut() {
        runIntake(-1);
        System.out.println("Info: Running intake out full speed");
    }

    public void stopIntake() {
        runIntake(0.0);
        System.out.println("Info: Stopping intake");
    }

    public void extendWrist() {
        wristPID.setSetpoint(WRIST_EXTENDED_SETPOINT_IN_ROTATIONS);
        System.out.println("Info: Extending wrist");
    }

    public void halfExtendWrist() {
        wristPID.setSetpoint(WRIST_HALF_EXTENDED_SETPOINT_IN_ROTATIONS);
        System.out.println("Info: Extending wrist halfway");
    }

    public void foldWrist() {
        wristPID.setSetpoint(WRIST_FOLDED_SETPOINT_IN_ROTATIONS);
        System.out.println("Info: Folding wrist");
    }

    public double getWristPositionInRotations() {
        return wristEncoder.getPosition();
    }

    public void tickWrist() {
        double PIDOutput = wristPID.calculate(getWristPositionInRotations());
        PIDOutput = min(max(-WRIST_MAX_SPEED, PIDOutput), WRIST_MAX_SPEED);

        wrist.set(PIDOutput);
    }
}
