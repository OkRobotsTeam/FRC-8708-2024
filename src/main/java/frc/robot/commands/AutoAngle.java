package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.BetterPoseEstimator;

public class AutoAngle extends Command {
    BetterPoseEstimator poseEstimator;
    private Shooter shooter;
    public AutoAngle(BetterPoseEstimator poseEstimator, Shooter shooter) {
        this.poseEstimator = poseEstimator;
        this.shooter = shooter;
        addRequirements(shooter);  
    }
    @Override
    public void initialize() {
        
    }

    @Override 
    public void execute() {
        shooter.autoAngle(poseEstimator);
    }
}
