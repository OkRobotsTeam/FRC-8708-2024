package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Debug;
import frc.robot.InitHelper;
import frc.robot.Constants.Intake.CANIds;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Intake.*;
import static java.lang.Double.*;

import java.text.DecimalFormat;
import java.text.NumberFormat;


public class Intake {
    private final CANSparkMax topIntake = new CANSparkMax(CANIds.TOP_INTAKE, kBrushless);
    private final CANSparkMax bottomIntake = new CANSparkMax(CANIds.BOTTOM_INTAKE, kBrushless);
    private final CANSparkMax wrist = new CANSparkMax(CANIds.WRIST, kBrushless);

    private final RelativeEncoder wristEncoder = wrist.getEncoder();
    private final PIDController wristPID = new PIDController(WRIST_PID_KP, WRIST_PID_KI, WRIST_PID_KD);
    private double lastWristPosition = 0;
    private InitHelper initHelper = new InitHelper("Intake", -0.001, 100, 2000, 50);
    private boolean disabled = true;
    private DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(9);

    
    // private DutyCycleEncoder absoluteEncoder1 = new DutyCycleEncoder(1);
    // private DutyCycleEncoder absoluteEncoder2 = new DutyCycleEncoder(2);
    // private DutyCycleEncoder absoluteEncoder3 = new DutyCycleEncoder(3);
    // private DutyCycleEncoder absoluteEncoder4 = new DutyCycleEncoder(4);
    // private DutyCycleEncoder absoluteEncoder5 = new DutyCycleEncoder(5);
    // private DutyCycleEncoder absoluteEncoder6 = new DutyCycleEncoder(6);
    // private DutyCycleEncoder absoluteEncoder7 = new DutyCycleEncoder(7);
    //private DutyCycleEncoder absoluteEncoder8 = new DutyCycleEncoder(8);
    //private DutyCycleEncoder absoluteEncoder9 = new DutyCycleEncoder(9);


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
        absoluteEncoder.setDistancePerRotation(1);

    }

    public void init() {
        disabled=false;
        wrist.set(-0.2);
        initHelper.start(wristEncoder.getPosition() );
        System.out.println("Intake init starting");
    }

    public void disable() {
        disabled=true;
        topIntake.set(0);
        bottomIntake.set(0);
        wrist.set(0);
    }

    public void enable() {
        disabled=false;
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
        runTopIntake(INTAKE_IN_TOP_POWER);
        runBottomIntake(INTAKE_IN_BOTTOM_POWER);
        System.out.println("Info: Running intake in");
    }

    public void runIntakeOut() {
        runIntake(INTAKE_OUT_POWER);
        System.out.println("Info: Running intake out");
    }

    public void fullSpeedOut() {
        runTopIntake(-1);
        runBottomIntake(-1);
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

    public void periodic() {
        //Debug.debugPrint("Wrist motor encoder: " + Debug.fourPlaces(wristEncoder.getPosition()) + " Wrist alt encoder: "
        //         + " 9:" + Debug.fourPlaces(absoluteEncoder.get()) );


        //System.out.println("Wrist motor power " + fourPlaces.format(PIDOutput) + " : " + wristPID.getSetpoint() + " : " + getWristPositionInRotations());
        //Debug.debugPrint("Wrist motor encoder: " + Debug.fourPlaces(wristEncoder.getPosition()) + " Wrist alt encoder: " + Debug.fourPlaces(absoluteEncoder.getPosition()));


        if (disabled) {

            return;
        }
        if (initHelper.isInitializing(wristEncoder.getPosition())) {
            return;
        }
        if (initHelper.justFinishedInit()) {
            System.out.println("Setting wrist encoder position to " + WRIST_STARTUP_POSITION);
            wrist.set(0);
            wristEncoder.setPosition(WRIST_STARTUP_POSITION);
            wristPID.reset();
            foldWrist();
            wristPID.calculate(WRIST_FOLDED_SETPOINT_IN_ROTATIONS);
            return;
        } 
        double PIDOutput = wristPID.calculate(getWristPositionInRotations());


        PIDOutput = min(max(-WRIST_MAX_SPEED, PIDOutput), WRIST_MAX_SPEED);
        wrist.set(PIDOutput);
    }
}
