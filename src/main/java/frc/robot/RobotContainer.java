package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutonomousTest;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrivetrain;


public class RobotContainer {
    // Controllers
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController manipulatorController = new CommandXboxController(1);

    // Subsystems
    private final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain();
    private final Intake intake = new Intake();

    // Shuffleboard
    private final SendableChooser<Boolean> driveMode = new SendableChooser<>();
    private final SendableChooser<Double> maxDriveSpeed = new SendableChooser<>();
    private final SendableChooser<Command> autonomousSelector = new SendableChooser<>();



    public RobotContainer() {
        setupShuffleboard();
        configureControllerBindings();
    }

    private void setupShuffleboard() {
        ShuffleboardTab drivingTab = Shuffleboard.getTab("Driving");

        autonomousSelector.setDefaultOption("Nothing", new InstantCommand());
        autonomousSelector.addOption("Test", new AutonomousTest(swerveDrivetrain));

        maxDriveSpeed.setDefaultOption("100%", 1.0);
        maxDriveSpeed.addOption("50%", 0.5);
        maxDriveSpeed.addOption("25%", 0.25);
        maxDriveSpeed.addOption("10%", 0.1);

        driveMode.setDefaultOption("Competition Mode", false);
        driveMode.setDefaultOption("Demonstration Mode", true);

        drivingTab.add(autonomousSelector).withPosition(4, 0).withSize(2, 1);
        drivingTab.add(driveMode).withPosition(4, 3).withSize(2, 1);

        Shuffleboard.selectTab("Driving");
        Shuffleboard.update();
    }
//                new InstantCommand(() -> intake.runIntakeIn(), intake)
//        );
        private void configureControllerBindings() {
            swerveDrivetrain.setDefaultCommand(getSwerveDriveCommand());

//        manipulatorController.a().onTrue(

        }

    public Command getSwerveDriveCommand() {
        return new InstantCommand(() -> swerveDrivetrain.driveWithController(driveController), swerveDrivetrain);
    }

    public Command getAutonomousCommand() {
        return autonomousSelector.getSelected();
    }

    public void robotInit() {
        swerveDrivetrain.stop();
        swerveDrivetrain.resetGyro();
    }

    public void teleopInit() {}

    public void autonomousInit() {}
}
