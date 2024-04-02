package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;

public class ShootWithAngle extends SequentialCommandGroup {
    public ShootWithAngle(
            SwerveDrivetrain drivetrain,
            Intake intake,
            Limelight limelight,
            Shooter shooter,
            double angle,
            BetterPoseEstimator poseEstimator
    ) {
        addCommands(
                new InstantCommand(() -> drivetrain.stop()),
                
                new WaitCommand(0.25),
                new InstantCommand(intake::foldWrist),
                new InstantCommand(() -> shooter.setTargetShooterDegreesFromHorizon(angle)),
                new InstantCommand(shooter::runShooterForward),
                new InstantCommand(intake::stopIntake),

                new WaitCommand(0.1),
                new InstantCommand(intake::runIntakeIn),
                new WaitCommand(0.25),
                new InstantCommand(intake::stopIntake),
                new WaitCommand(0.5),
                new InstantCommand(intake::runIntakeOut),
                new WaitCommand(0.5),
                // new InstantCommand(intake::runIntakeIn),
                // new WaitCommand(1),
                // new InstantCommand(intake::runIntakeOut),
                // new WaitCommand(2),
                //new InstantCommand(shooter::stopShooter),
                new InstantCommand(intake::stopIntake)
        );
    }

}
