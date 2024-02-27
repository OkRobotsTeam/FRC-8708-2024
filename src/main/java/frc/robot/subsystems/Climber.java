package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static frc.robot.Constants.Climber.*;


public class Climber {
    private final TalonFX leftClimber = new TalonFX(CANIds.LEFT_CLIMBER, "CTRE");
    private final TalonFX rightClimber = new TalonFX(CANIds.RIGHT_CLIMBER, "CTRE");
    private final ProfiledPIDController leftClimberPID = new ProfiledPIDController(CLIMBER_PID_KP, CLIMBER_PID_KI, CLIMBER_PID_KD, new TrapezoidProfile.Constraints(CLIMBER_PID_MAX_SPEED_IN_ROTATIONS_PER_SECOND, CLIMBER_PID_MAX_ACCELERATION_IN_ROTATIONS_PER_SECOND_SQUARED));
    private final ProfiledPIDController rightClimberPID = new ProfiledPIDController(CLIMBER_PID_KP, CLIMBER_PID_KI, CLIMBER_PID_KD, new TrapezoidProfile.Constraints(CLIMBER_PID_MAX_SPEED_IN_ROTATIONS_PER_SECOND, CLIMBER_PID_MAX_ACCELERATION_IN_ROTATIONS_PER_SECOND_SQUARED));
    private boolean climberDoneCalibrating = false;
    private boolean leftClimberMotorCalibrated = false;
    private boolean rightClimberMotorCalibrated = false;
    private double climberCalibrationStartTime = 0.0;


    public Climber() {
        leftClimber.setInverted(CLIMBER_LEFT_INVERTED);
        rightClimber.setInverted(CLIMBER_RIGHT_INVERTED);
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


    public boolean climberStillCalibrating() {
        return !climberDoneCalibrating;
    }

    public void tickClimber() {
        if (!climberDoneCalibrating) {
            if (System.currentTimeMillis() - climberCalibrationStartTime > CALIBRATION_DELAY_MS) {
                if (!leftClimberMotorCalibrated && rightClimber.getStatorCurrent().getValueAsDouble() > 4.0) {
                    setLeftEncoderPosition(LEFT_CLIMBER_STARTUP_POSITION);
                    leftClimberPID.setGoal(LEFT_CLIMBER_STARTUP_POSITION);
                    leftClimber.set(0);
                    leftClimberMotorCalibrated = true;
                    System.out.println("Info: Left climber calibrated");
                }
                if (!rightClimberMotorCalibrated && rightClimber.getStatorCurrent().getValueAsDouble() > 4.0) {
                    setRightEncoderPosition(RIGHT_CLIMBER_STARTUP_POSITION);
                    rightClimberPID.setGoal(RIGHT_CLIMBER_STARTUP_POSITION);
                    rightClimber.set(0);
                    rightClimberMotorCalibrated = true;
                    System.out.println("Info: Right climber calibrated");
                }
                if (leftClimberMotorCalibrated && rightClimberMotorCalibrated) {
                    climberDoneCalibrating = true;
                    System.out.println("Info: Climber calibrated");
                }
            }
        } else {
            leftClimber.set(leftClimberPID.calculate(getLeftEncoderPosition()));
            rightClimber.set(rightClimberPID.calculate(getRightEncoderPosition()));
        }
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