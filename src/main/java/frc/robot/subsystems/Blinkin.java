package frc.robot.subsystems;

import static frc.robot.Constants.Intake.WRIST_HALF_EXTENDED_SETPOINT_IN_ROTATIONS;

import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blinkin extends SubsystemBase {
    public static double RED = 0.61;
    public static double BLUE = 0.87;
    public static double STROBE_WHITE = -0.05;
    public static double ORANGE = 0.65;
    private Spark blinkin = new Spark(9);
    Intake intake;
    Shooter shooter;

    public Blinkin(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
    }

    public void init() {

    }
    public void periodic() {
        
        if (intake.getWristPosition() > WRIST_HALF_EXTENDED_SETPOINT_IN_ROTATIONS) {
            if (intake.getBottomOutputCurrent() > 7) {
                blinkin.set(STROBE_WHITE);
            } else {
                blinkin.set(0.71);
            }
        } else {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == DriverStation.Alliance.Red) {
                    blinkin.set(RED);
                } else {
                    blinkin.set(BLUE);
                }
            } else {
                blinkin.set(ORANGE);
            }

        }
    }
}

