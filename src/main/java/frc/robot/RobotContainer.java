package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutonomousTest;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivetrain;


public class RobotContainer {
    // Controllers
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController manipulatorController = new CommandXboxController(1);

    // Subsystems
    private final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain();
//    private final Climber climber = new Climber();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();

    // Shuffleboard
    private final SendableChooser<Boolean> driveMode = new SendableChooser<>();
    private final SendableChooser<Double> maxDriveSpeed = new SendableChooser<>();
    private final SendableChooser<Command> autonomousSelector = new SendableChooser<>();


    public RobotContainer() {
        setupShuffleboard();
        swerveDrivetrain.setDefaultCommand(getSwerveDriveCommand());
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
        drivingTab.add(maxDriveSpeed).withPosition(6, 3).withSize(2, 1);

        Shuffleboard.selectTab("Driving");
        Shuffleboard.update();
    }

    private void configureControllerBindings() {
        swerveDrivetrain.setDefaultCommand(getSwerveDriveCommand());

        manipulatorController.a().onTrue(Commands.runOnce(intake::runIntakeIn));
        manipulatorController.a().onFalse(Commands.runOnce(intake::stopIntake));

        manipulatorController.b().onTrue(Commands.runOnce(intake::runIntakeOut));
        manipulatorController.b().onFalse(Commands.runOnce(intake::stopIntake));

        manipulatorController.x().onTrue(Commands.runOnce(intake::extendWrist));
        manipulatorController.x().onFalse(Commands.runOnce(intake::foldWrist));

        manipulatorController.y().onTrue(Commands.runOnce(shooter::runShooterForward));
        manipulatorController.y().onFalse(Commands.runOnce(shooter::stopShooter));

        manipulatorController.povUp().onTrue(Commands.runOnce(shooter::shooterManualAdjustUp));
        manipulatorController.povDown().onFalse(Commands.runOnce(shooter::shooterManualAdjustDown));

        driveController.a().onTrue(Commands.runOnce(swerveDrivetrain::resetGyro));

    }

    public Command getSwerveDriveCommand() {
        return new InstantCommand(() -> swerveDrivetrain.driveWithController(driveController), swerveDrivetrain);
//        return Commands.runOnce(() -> swerveDrivetrain.driveWithController(driveController));
    }

    public Command getAutonomousCommand() {
        return autonomousSelector.getSelected();
    }

    public void robotInit() {
        swerveDrivetrain.stop();
        swerveDrivetrain.resetGyro();
    }

    public void teleopInit() {
//        climber.recalibrateClimber();
    }

    public void autonomousInit() {
    }

    public void periodic() {
        swerveDrivetrain.updateOdometry();
//        climber.tickClimber();
        shooter.tickShooterRotation();
        intake.tickWrist();
    }
}
