package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutonomousTest;
import frc.robot.subsystems.SwerveDrivetrain;


public class RobotContainer {
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController manipulatorController = new CommandXboxController(1);
    private final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain();
    private final SendableChooser<Command> autonomousSelector = new SendableChooser<>();


    public RobotContainer() {
        setupShuffleboard();
        configureControllerBindings();
    }

    private void setupShuffleboard() {
        ShuffleboardTab drivingTab = Shuffleboard.getTab("Driving");
        SendableChooser<Double> maxDriveSpeed = new SendableChooser<>();
        SendableChooser<Boolean> driveMode = new SendableChooser<>();

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
//        m_objectInIntake = drivingTab.add("ObjectInIntake", false).withPosition(4, 1).withSize(2, 2).getEntry();

        Shuffleboard.selectTab("Driving");
        Shuffleboard.update();
    }

    private void configureControllerBindings() {
        swerveDrivetrain.setDefaultCommand(getSwerveDriveCommand());

//        manipulatorController.a().onTrue(
//                new InstantCommand(() -> m_arm.manualAdjustTarget(1.0), m_arm)
//        );
    }

    public Command getSwerveDriveCommand() {
        return new InstantCommand(() -> swerveDrivetrain.driveWithController(driveController), swerveDrivetrain);
    }

    public Command getAutonomousCommand() {
        return autonomousSelector.getSelected();
    }


    public void robotInit() {
        // Initialize things here
    }

    public void teleopInit() {
//        m_arm.init();
//        m_lights.init(m_safety_mode.getSelected());
    }

    public void autonomousInit() {
//        m_arm.init();
//        m_lights.init(m_safety_mode.getSelected());
    }
}
