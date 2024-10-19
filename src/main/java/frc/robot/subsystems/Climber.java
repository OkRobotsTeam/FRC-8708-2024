package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Debug;
import frc.robot.InitHelper;

import static frc.robot.Constants.Climber.*;
import static java.lang.Double.max;


public class Climber  extends SubsystemBase {
    private final TalonFX leftClimber = new TalonFX(CANIds.LEFT_CLIMBER);
    private final TalonFX rightClimber = new TalonFX(CANIds.RIGHT_CLIMBER);
    private final ProfiledPIDController leftClimberPID = new ProfiledPIDController(CLIMBER_PID_KP, CLIMBER_PID_KI, CLIMBER_PID_KD, new TrapezoidProfile.Constraints(CLIMBER_PID_MAX_SPEED_IN_ROTATIONS_PER_SECOND, CLIMBER_PID_MAX_ACCELERATION_IN_ROTATIONS_PER_SECOND_SQUARED));
    private final ProfiledPIDController rightClimberPID = new ProfiledPIDController(CLIMBER_PID_KP, CLIMBER_PID_KI, CLIMBER_PID_KD, new TrapezoidProfile.Constraints(CLIMBER_PID_MAX_SPEED_IN_ROTATIONS_PER_SECOND, CLIMBER_PID_MAX_ACCELERATION_IN_ROTATIONS_PER_SECOND_SQUARED));
    private boolean climberDoneCalibrating = false;
    private boolean leftClimberMotorCalibrated = false;
    private boolean rightClimberMotorCalibrated = false;
    private double climberCalibrationStartTime = 0.0;
    private XboxController controller;
    private boolean enabled = false;
    private InitHelper leftInitHelper = new InitHelper("Left Climber",-0.01,50,50000);
    private InitHelper rightInitHelper = new InitHelper("Right Climber", -0.01, 50, 50000);
    private DigitalInput leftClimberLimitSwitch = new DigitalInput(LEFT_CLIMBER_DIGITAL_SWITCH_PORT);
    private DigitalInput rightClimberLimitSwitch = new DigitalInput(RIGHT_CLIMBER_DIGITAL_SWITCH_PORT);

    private final double CLIMBER_TRAVEL_LIMIT = 121;
    private boolean isClimberUp = false;

    public Climber(XboxController controller) {
        this.controller = controller;
        leftClimber.setInverted(CLIMBER_LEFT_INVERTED);
        rightClimber.setInverted(CLIMBER_RIGHT_INVERTED);
    }

    public void init() {
        System.out.println("Left Climber Starting Initializing " + leftClimber.getPosition().getValueAsDouble());
        leftInitHelper.start(leftClimber.getPosition().getValueAsDouble());
        // MJB leftClimber.set(-0.075);
        leftClimber.set(CLIMBER_INIT_MOTOR_POWER);
        rightInitHelper.start(rightClimber.getPosition().getValueAsDouble());
        // MJB rightClimber.set(-0.075);
        rightClimber.set(CLIMBER_INIT_MOTOR_POWER);
        leftClimberPID.reset(0);
        rightClimberPID.reset(0);
        enabled=true;
        lowerClimber();

    }

    public void disable() {
        enabled = false;
    }

    private double getLeftEncoderPosition() {
        return leftClimber.getPosition().getValueAsDouble() * LEFT_CLIMBER_GEAR_RATIO;
    }

    private double getRightEncoderPosition() {
        return rightClimber.getPosition().getValueAsDouble() * RIGHT_CLIMBER_GEAR_RATIO;
    }

    private void setLeftEncoderPosition(double position) {
        leftClimber.setPosition(position * LEFT_CLIMBER_GEAR_RATIO);
    }

    private void setRightEncoderPosition(double position) {
        rightClimber.setPosition(position * RIGHT_CLIMBER_GEAR_RATIO);
    }

    private boolean climberStillCalibrating() {
        return leftInitHelper.isInitializing() || rightInitHelper.isInitializing();
    }

    public void periodic() {

        double left = 0;
        double right = 0;
        //System.out.println("Switches: " + leftClimberLimitSwitch.get() +  " : " + rightClimberLimitSwitch.get());
        if (!enabled) {
            leftClimber.set(0);
            rightClimber.set(0);
            return;
        }
        if (leftInitHelper.isInitializing(leftClimber.getPosition().getValueAsDouble())) {
            if (leftClimberLimitSwitch.get() ) {
                System.out.println("Left Climber Limit Switch Triggered");
                leftInitHelper.setDone();
            } else {
                System.out.println("Initializing Left Climber:" + leftClimber.getPosition().getValueAsDouble() + " C: "
                        + leftClimber.getSupplyCurrent());
            }
        } else if (leftInitHelper.justFinishedInit()) {
            System.out.println("Left Climber Done Initialzing");
            if (leftClimberLimitSwitch.get()) {
                leftClimber.setPosition(-1);
            } else {
                leftClimber.setPosition(-5);
            } 
        } else {
            left = leftClimberPID.calculate(getLeftEncoderPosition());
            if (leftClimberLimitSwitch.get()) {
                leftClimber.set(max(left, 0.0));
            } else {
                leftClimber.set(left);
            }
        }
        if (rightInitHelper.isInitializing(rightClimber.getPosition().getValueAsDouble())) {
            if (rightClimberLimitSwitch.get() ) {
                System.out.println("Right Climber Limit Switch Triggered");
                rightInitHelper.setDone();
            } else {
                System.out.println("Initializing Right Climber:" + rightClimber.getPosition().getValueAsDouble()
                        + " C: " + rightClimber.getSupplyCurrent());
            }
        } else if (rightInitHelper.justFinishedInit()) {
            System.out.println("Right Climber Done Initialzing");
            if (rightClimberLimitSwitch.get()) {
                rightClimber.setPosition(-1);
            } else {
                rightClimber.setPosition(-5);
            } 
        } else {
            right = rightClimberPID.calculate(getRightEncoderPosition());
            if (rightClimberLimitSwitch.get()) {
                rightClimber.set(max(right, 0.0));
            } else {
                rightClimber.set(right);
            }
        }

        // Debug.debugPrint("Climber",
        //         "Left Power: " + fmt(left)
        //                 + " Right Power" + fmt(right)
        //                 + " leftPosition: " + fmt(leftClimber.getPosition())
        //                 + " rightPosition: " + fmt(rightClimber.getPosition())
        //                 + " leftCurrent: " + fmt(leftClimber.getSupplyCurrent())
        //                 + " rightCurrent: " + fmt(rightClimber.getSupplyCurrent())
        //                 );

    }

    public String fmt(StatusSignal<Double> num) {
        return fmt(num.getValueAsDouble());
    }

    public String fmt(double num) {
        return Debug.fourPlaces(num);
    }

    public void toggleClimber() {
        System.out.println("Toggleing Climber");
        if (isClimberUp()) {
            lowerClimber();
        } else {
            raiseClimber();
        }
    
    }

    public boolean isClimberUp() {
        return isClimberUp;
    }

    public void raiseClimber() {
        if (climberStillCalibrating()) {
            System.out.println("Warning: Climber will not move until it is calibrated!");
        }
        isClimberUp=true;
        leftClimberPID.setGoal(CLIMBER_UP_POSITION);
        rightClimberPID.setGoal(CLIMBER_UP_POSITION);
        System.out.println("Info: Raising climber");
    }

    public void lowerClimber() {
        if (climberStillCalibrating()) {
            System.out.println("Warning: Climber will not move until it is calibrated!");
        }
        isClimberUp=false;
        leftClimberPID.setGoal(CLIMBER_DOWN_POSITION);
        rightClimberPID.setGoal(CLIMBER_DOWN_POSITION);
        System.out.println("Info: Lowering climber");
    }

    public void recalibrateClimber() {
        climberDoneCalibrating = leftClimberMotorCalibrated = rightClimberMotorCalibrated = false;
        climberCalibrationStartTime = System.currentTimeMillis();
        System.out.println("Info: Recalibrating climber");
    }
}