package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;

public class TwoRingAutonomous extends SequentialCommandGroup {
    public TwoRingAutonomous(
            SwerveDrivetrain drivetrain,
            Intake intake,
            Shooter shooter
    ) {
        addCommands(
                new InstantCommand(drivetrain::resetGyro, drivetrain),
                new InstantCommand(drivetrain::resetOdometry),

                // Shoot
                new InstantCommand(() -> shooter.setTargetShooterDegreesFromHorizon(30)),
                new InstantCommand(shooter::runShooterForward),
                new WaitCommand(0.8),
                new InstantCommand(intake::runIntakeOut),
                new WaitCommand(0.5),
                new InstantCommand(intake::runIntakeIn),
                new WaitCommand(0.5),
                new InstantCommand(intake::runIntakeOut),
                new InstantCommand(shooter::stopShooter),
                new InstantCommand(shooter::shooterRotationReset),

                // Go pickup second disk
                new InstantCommand(intake::extendWrist),
                new InstantCommand(intake::runIntakeIn),
//                new MoveToPosition(0, new Translation2d(1,0), 0.05, drivetrain, false, true),
//                new MoveToPosition(0, new Translation2d(0.5,0), 0.05, drivetrain, false, true),
                new InstantCommand(intake::foldWrist),
                new InstantCommand(intake::stopIntake),
//                new MoveToPosition(0, new Translation2d(0,0), 0.05, drivetrain, false, true),

                // Shoot
                new InstantCommand(() -> shooter.setTargetShooterDegreesFromHorizon(30)),
                new InstantCommand(shooter::runShooterForward),
                new WaitCommand(0.8),
                new InstantCommand(intake::runIntakeOut),
                new WaitCommand(0.5),
                new InstantCommand(intake::runIntakeIn),
                new WaitCommand(0.5),
                new InstantCommand(intake::runIntakeOut),
                new InstantCommand(shooter::stopShooter),
                new InstantCommand(shooter::shooterRotationReset)
        );
    }

}
