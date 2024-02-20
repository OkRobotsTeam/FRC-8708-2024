package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;

public class Climber {
    private final CANSparkMax leftClimber = new CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightClimber = new CANSparkMax(2, CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder leftClimberEncoder = leftClimber.getEncoder();
    private final RelativeEncoder rightClimberEncoder = rightClimber.getEncoder();
    private final PIDController leftClimberPID = new PIDController(1.0, 0.0, 0.0);
    private final PIDController rightClimberPID = new PIDController(1.0, 0.0, 0.0);
    private boolean climberDoneCalibrating = false;
    private boolean climberLefMotorCalibrated = false;
    private boolean climberRightMotorCalibrated = false;


    public Climber() {
        leftClimberEncoder.setPosition(0);
        rightClimberEncoder.setPosition(0);

        leftClimberEncoder.setPositionConversionFactor(8.0 / 54.0);
        leftClimberEncoder.setVelocityConversionFactor(8.0 / 54.0);

        leftClimberPID.setSetpoint(0);
        rightClimberPID.setSetpoint(0);
    }

    public boolean climberIsCalibrated() {
        return climberDoneCalibrating;
    }

    public void tickClimber() {
        if (!climberDoneCalibrating) {
            if (leftClimber.getOutputCurrent() > 4.0) {
                leftClimberEncoder.setPosition(0);
            }
        }
    }



}
