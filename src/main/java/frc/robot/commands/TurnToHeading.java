package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;


public class TurnToHeading extends Command {

    MoveToPosition move_command;

    public TurnToHeading(double targetHeading, SwerveDrivetrain drivetrain, boolean precise) {
        move_command = new MoveToPosition(targetHeading, drivetrain.getOdometryPosition(), 0.5, drivetrain, precise, false);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        move_command.initialize();
    }

    @Override
    public void execute() {
        move_command.execute();
    }

    @Override
    public boolean isFinished() {
        return move_command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        move_command.end(interrupted);
        System.out.println("TurnToHeading: Done");
    }
}
