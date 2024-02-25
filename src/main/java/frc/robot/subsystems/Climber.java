package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Climber.*;


public class Climber {
    private final CANSparkMax leftClimber = new CANSparkMax(CANIds.LEFT_CLIMBER, kBrushless);
    private final CANSparkMax rightClimber = new CANSparkMax(CANIds.RIGHT_CLIMBER, kBrushless);
    private final RelativeEncoder leftClimberEncoder = leftClimber.getEncoder();
    private final RelativeEncoder rightClimberEncoder = rightClimber.getEncoder();
    private final ProfiledPIDController leftClimberPID = new ProfiledPIDController(CLIMBER_PID_KP, CLIMBER_PID_KI, CLIMBER_PID_KD, new TrapezoidProfile.Constraints(CLIMBER_PID_MAX_SPEED_IN_ROTATIONS_PER_SECOND, CLIMBER_PID_MAX_ACCELERATION_IN_ROTATIONS_PER_SECOND_SQUARED));
    private final ProfiledPIDController rightClimberPID = new ProfiledPIDController(CLIMBER_PID_KP, CLIMBER_PID_KI, CLIMBER_PID_KD, new TrapezoidProfile.Constraints(CLIMBER_PID_MAX_SPEED_IN_ROTATIONS_PER_SECOND, CLIMBER_PID_MAX_ACCELERATION_IN_ROTATIONS_PER_SECOND_SQUARED));
    private boolean climberDoneCalibrating = false;
    private boolean leftClimberMotorCalibrated = false;
    private boolean rightClimberMotorCalibrated = false;


    public Climber() {
        leftClimber.setInverted(CLIMBER_LEFT_INVERTED);
        rightClimber.setInverted(CLIMBER_RIGHT_INVERTED);

        leftClimberEncoder.setPositionConversionFactor(LEFT_CLIMBER_GEAR_RATIO);
        leftClimberEncoder.setVelocityConversionFactor(LEFT_CLIMBER_GEAR_RATIO);

        rightClimberEncoder.setPositionConversionFactor(RIGHT_CLIMBER_GEAR_RATIO);
        rightClimberEncoder.setVelocityConversionFactor(RIGHT_CLIMBER_GEAR_RATIO);
    }

    public boolean climberStillCalibrating() {
        return !climberDoneCalibrating;
    }

    public void tickClimber() {
        if (!climberDoneCalibrating) {
            if (!leftClimberMotorCalibrated && leftClimber.getOutputCurrent() > 4.0) {
                leftClimberEncoder.setPosition(LEFT_CLIMBER_STARTUP_POSITION);
                leftClimberPID.setGoal(LEFT_CLIMBER_STARTUP_POSITION);
                leftClimber.set(0);
                leftClimberMotorCalibrated = true;
                System.out.println("Info: Left climber calibrated");
            }
            if (!rightClimberMotorCalibrated && rightClimber.getOutputCurrent() > 4.0) {
                rightClimberEncoder.setPosition(RIGHT_CLIMBER_STARTUP_POSITION);
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
        System.out.println("Info: Lowereing climber");
    }

    public void recalibrateClimber() {
        climberDoneCalibrating = leftClimberMotorCalibrated = rightClimberMotorCalibrated = false;
        System.out.println("Info: Recalibrating climber");
    }
}