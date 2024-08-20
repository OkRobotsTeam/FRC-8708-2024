package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;



    // This function is run when the robot is first started up
    @Override
    public void robotInit() {

        robotContainer = new RobotContainer();
        robotContainer.robotInit();        
    }

    // This function is called every 20 ms, no matter the mode.
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.robotPeriodic();
    }

    // This function is called once each time the robot enters Disabled mode.
    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        robotContainer.disable();
    }
    @Override
    public void disabledPeriodic() {
        robotContainer.periodic();
     }

    @Override
    public void teleopInit() {
        // Cancels the autonomous task when teleop starts
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        robotContainer.teleopInit();
    }

    @Override
    public void teleopPeriodic() {
        robotContainer.periodic();
        robotContainer.teleopPeriodic();
    }

    @Override
    public void autonomousInit() {
        robotContainer.autonomousInit();
        autonomousCommand = robotContainer.getAutonomousCommand();
        // if a valid command was received, schedule it
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        robotContainer.periodic();
    }



    @Override
    public void testInit() {
        // Cancels the autonomous task when teleop starts
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        robotContainer.testInit();
    }

    @Override
    public void testPeriodic() {
        robotContainer.periodic();
    }
}