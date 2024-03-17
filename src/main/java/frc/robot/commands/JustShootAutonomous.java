package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;

public class JustShootAutonomous extends SequentialCommandGroup {
    public JustShootAutonomous(
            SwerveDrivetrain drivetrain,
            Intake intake,
            Limelight limelight,
            Shooter shooter
    ) {
        addCommands(
                new InstantCommand(drivetrain::resetGyro, drivetrain),
                new InstantCommand(drivetrain::resetOdometry),

                // Shoot
                new InstantCommand(() -> shooter.setTargetShooterDegreesFromHorizon(65)),
                new InstantCommand(shooter::runShooterForward),
                new WaitCommand(0.8),
                new InstantCommand(intake::runIntakeOut),
                new WaitCommand(0.5),
                new InstantCommand(intake::runIntakeIn),
                new WaitCommand(0.5),
                new InstantCommand(intake::runIntakeOut),
                new WaitCommand(0.5),
                new InstantCommand(shooter::stopShooter),
                new InstantCommand(intake::stopIntake),
                new InstantCommand(shooter::shooterRotationReset)
        );
    }

}
