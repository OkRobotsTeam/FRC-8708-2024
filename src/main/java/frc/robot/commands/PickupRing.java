package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;

public class PickupRing extends SequentialCommandGroup {
    Intake intake;
    public PickupRing(Intake intake) {
        this.intake = intake;
        addCommands(
                new InstantCommand(intake::extendWrist),
                new WaitCommand(0.25),
                new InstantCommand(intake::runIntakeIn), 
                new WaitCommand(1).handleInterrupt(this::interrupt),
                new InstantCommand(intake::foldWrist),
                new WaitCommand(0.25).handleInterrupt(this::interrupt),
                new InstantCommand(intake::stopIntake)
        );
        

    }
    public void interrupt() {
        intake.stopIntake();
        intake.foldWrist();
    }
    // @Override
    // public void end(boolean interrupted) {
    //     super.end(interrupted);
    //     System.out.println("Cancel called");
    //     intake.stopIntake();
    //     intake.foldWrist();
    // }
}
