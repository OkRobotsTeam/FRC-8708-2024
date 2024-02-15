package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class AutonomousTest extends SequentialCommandGroup {
    public AutonomousTest(
            SwerveDrivetrain drivetrain
    ) {
        addCommands(
                new InstantCommand(drivetrain::resetGyro, drivetrain),
                // new TurnTo(90, 0.6, drive),
                // new DriveForTick(0, 20, 0.7, drive, false, 20, 1),
                // new WaitCommand(2),
                new WaitCommand(2)
        );
    }

}
