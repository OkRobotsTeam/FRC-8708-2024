package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;

public class AutonomousTest extends SequentialCommandGroup {
    public AutonomousTest(
            SwerveDrivetrain drivetrain,
            Intake intake,
            Limelight limelight,
            Shooter shooter,
            PoseEstimator poseEstimator
    ) {
        addCommands(
                new InstantCommand(drivetrain::resetGyro, drivetrain),
                new InstantCommand(drivetrain::resetOdometry),
//                new InstantCommand(intake::runIntakeIn),
//                new InstantCommand(intake::extendWrist),
                new MoveToPosition(90, new Translation2d(0,0), 0.05, drivetrain, false, true)

//                new MoveToPosition(0, new Translation2d(-10,-10), 0.05, drivetrain, false, false),
//                new MoveToPosition(0, new Translation2d(10, 10), 0.05, drivetrain, false, false)
//                new InstantCommand(intake::extendWrist),
//                new InstantCommand(intake::runIntakeIn),
//                new WaitCommand(2.0),
//                new InstantCommand(intake::stopIntake),
//                new InstantCommand(intake::foldWrist)
        );
    }

}
