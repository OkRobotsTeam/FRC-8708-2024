package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;

@Deprecated
public class DriveWithController extends Command {
    SwerveDrivetrain drivetrain;
    XboxController controller; 
    SendableChooser<Double> driveSpeed;
    SendableChooser<Double> turnSpeed;

    int i;
    double speed;
    double tspeed;

    public DriveWithController(SwerveDrivetrain drivetrain,  XboxController controller, SendableChooser<Double> driveSpeed, SendableChooser<Double> turnSpeed) {
         this.drivetrain = drivetrain;
         this.controller = controller;
         this.driveSpeed = driveSpeed;
         this.turnSpeed = turnSpeed;
         addRequirements(drivetrain);
        

    }

    public void execute() {
        drivetrain.driveWithController(controller, driveSpeed, turnSpeed);
    }

    public boolean isFinished() {
        return false;
    }
}


