package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveDrivetrain;

public class DriveWithController extends Command {
    SwerveDrivetrain drivetrain;
    CommandXboxController controller; 
    SendableChooser<Double> driveSpeed;
    SendableChooser<Double> turnSpeed;

    int i;
    double speed;
    double tspeed;

    public DriveWithController(SwerveDrivetrain drivetrain,  CommandXboxController controller, SendableChooser<Double> driveSpeed, SendableChooser<Double> turnSpeed) {
         this.drivetrain = drivetrain;
         this.controller = controller;
         this.driveSpeed = driveSpeed;
         this.turnSpeed = turnSpeed;
         addRequirements(drivetrain);

    }

    public void execute() {
        if (i++%20 == 0 ) {
            speed = driveSpeed.getSelected();
            tspeed = turnSpeed.getSelected();
        }
  
        drivetrain.driveWithController(controller, speed, tspeed, false);

    }

    public boolean isFinished() {
        return false;
    }
}


