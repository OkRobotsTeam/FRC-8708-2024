package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;

public class Shoot extends SequentialCommandGroup {
    public Shoot(
            SwerveDrivetrain drivetrain,
            Intake intake,
            Limelight limelight,
            Shooter shooter,
            BetterPoseEstimator poseEstimator
    ) {
        addCommands(
                new InstantCommand(drivetrain::stop),
                new InstantCommand(() -> shooter.setTargetShooterDegreesFromHorizon(45)),
                new InstantCommand(shooter::runShooterForward),
                new WaitCommand(1),
                new InstantCommand(intake::runIntakeOut),
                new WaitCommand(1),
                // new InstantCommand(intake::runIntakeIn),
                // new WaitCommand(1),
                // new InstantCommand(intake::runIntakeOut),
                // new WaitCommand(2),
                //new InstantCommand(shooter::stopShooter),
                new InstantCommand(intake::stopIntake)
        );
    }
}
