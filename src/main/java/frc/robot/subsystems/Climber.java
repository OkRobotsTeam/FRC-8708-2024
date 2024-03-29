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


public class Climber  extends SubsystemBase {
    private final TalonFX leftClimber = new TalonFX(CANIds.LEFT_CLIMBER, "CTRE");
    private final TalonFX rightClimber = new TalonFX(CANIds.RIGHT_CLIMBER, "CTRE");
    private final ProfiledPIDController leftClimberPID = new ProfiledPIDController(CLIMBER_PID_KP, CLIMBER_PID_KI, CLIMBER_PID_KD, new TrapezoidProfile.Constraints(CLIMBER_PID_MAX_SPEED_IN_ROTATIONS_PER_SECOND, CLIMBER_PID_MAX_ACCELERATION_IN_ROTATIONS_PER_SECOND_SQUARED));
    private final ProfiledPIDController rightClimberPID = new ProfiledPIDController(CLIMBER_PID_KP, CLIMBER_PID_KI, CLIMBER_PID_KD, new TrapezoidProfile.Constraints(CLIMBER_PID_MAX_SPEED_IN_ROTATIONS_PER_SECOND, CLIMBER_PID_MAX_ACCELERATION_IN_ROTATIONS_PER_SECOND_SQUARED));
    private boolean climberDoneCalibrating = false;
    private boolean leftClimberMotorCalibrated = false;
    private boolean rightClimberMotorCalibrated = false;
    private double climberCalibrationStartTime = 0.0;
    private XboxController controller;
    private boolean enabled = false;
    private InitHelper leftInitHelper = new InitHelper("Left Climber",-0.01,50,50000);
    private InitHelper rightInitHelper = new InitHelper("Left Climber", -0.01, 50, 50000);
    private DigitalInput leftClimberLimitSwitch = new DigitalInput(LEFT_CLIMBER_DIGITAL_SWITCH_PORT);
    private DigitalInput rightClimberLimitSwitch = new DigitalInput(RIGHT_CLIMBER_DIGITAL_SWITCH_PORT);

    private final double CLIMBER_TRAVEL_LIMIT = 121;

    public Climber(XboxController controller) {
        this.controller = controller;
        leftClimber.setInverted(CLIMBER_LEFT_INVERTED);
        rightClimber.setInverted(CLIMBER_RIGHT_INVERTED);
    }

    public void init() {
        System.out.println("Left Climber Starting Initialzing " + leftClimber.getPosition().getValueAsDouble());
        leftInitHelper.start(leftClimber.getPosition().getValueAsDouble());
        leftClimber.set(-0.05);
        rightInitHelper.start(leftClimber.getPosition().getValueAsDouble());
        rightClimber.set(-0.05);
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
            leftClimber.set(left);
        }
        if (rightInitHelper.isInitializing(rightClimber.getPosition().getValueAsDouble())) {
            if (rightClimberLimitSwitch.get() ) {
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
            rightClimber.set(right);
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

    public void raiseClimber() {
        if (climberStillCalibrating()) {
            System.out.println("Warning: Climber will not move until it is calibrated!");
        }

        leftClimberPID.setGoal(CLIMBER_UP_POSITION);
        rightClimberPID.setGoal(CLIMBER_UP_POSITION);
        System.out.println("Info: Raising climber");
    }

    public void lowerClimber() {
        if (climberStillCalibrating()) {
            System.out.println("Warning: Climber will not move until it is calibrated!");
        }

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