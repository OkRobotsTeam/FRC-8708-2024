package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutonomousTest;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivetrain;

import static frc.robot.Constants.SwerveDrivetrain.*;


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
//    private final SendableChooser<Boolean> driveMode = new SendableChooser<>();
    private final SendableChooser<Double> driveSpeed = new SendableChooser<>();
    private final SendableChooser<Double> turnSpeed = new SendableChooser<>();
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

        driveSpeed.setDefaultOption("100%", 1.0);
        driveSpeed.addOption("50%", 0.5);
        driveSpeed.addOption("25%", 0.25);
        driveSpeed.addOption("10%", 0.1);

        turnSpeed.setDefaultOption("100%", 1.0);
        turnSpeed.addOption("50%", 0.5);
        turnSpeed.addOption("25%", 0.25);
        turnSpeed.addOption("10%", 0.1);

//        driveMode.setDefaultOption("Competition Mode", false);
//        driveMode.addOption("Demonstration Mode", true);

        drivingTab.add(autonomousSelector).withPosition(4, 0).withSize(2, 1);
//        drivingTab.add(driveMode).withPosition(4, 3).withSize(2, 1);
        drivingTab.add(driveSpeed).withPosition(6, 3).withSize(2, 1);
        drivingTab.add(turnSpeed).withPosition(8, 3).withSize(2, 1);


        SmartDashboard.putNumber("Shooter Angle", shooter.getTargetShooterDegreesFromHorizon());

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

        manipulatorController.rightBumper().onTrue(Commands.runOnce(intake::halfExtendWrist));
        manipulatorController.rightBumper().onFalse(Commands.runOnce(intake::foldWrist));

        manipulatorController.y().onTrue(Commands.runOnce(shooter::runShooterForward));
        manipulatorController.y().onFalse(Commands.runOnce(shooter::stopShooter));

        manipulatorController.povUp().onTrue(Commands.runOnce(shooter::shooterManualAdjustUp));
        manipulatorController.povDown().onFalse(Commands.runOnce(shooter::shooterManualAdjustDown));

        manipulatorController.povRight().onFalse(Commands.runOnce(shooter::shooterRotationReset));

        manipulatorController.povLeft().onTrue(Commands.runOnce(intake::fullSpeedOut));
        manipulatorController.povLeft().onFalse(Commands.runOnce(intake::stopIntake));

        manipulatorController.leftBumper().onTrue(Commands.runOnce(shooter::runShooterSlow));
        manipulatorController.leftBumper().onFalse(Commands.runOnce(shooter::stopShooter));

        driveController.a().onTrue(
                Commands.runOnce(swerveDrivetrain::resetGyro).andThen(
                Commands.runOnce(swerveDrivetrain::resetOdometry)
                )
        );

    }

    public Command getSwerveDriveCommand() {
        return new InstantCommand(() -> swerveDrivetrain.driveWithController(driveController, driveSpeed.getSelected(), turnSpeed.getSelected()), swerveDrivetrain);
    }

    public Command getAutonomousCommand() {
        return autonomousSelector.getSelected();
    }

    public void robotInit() {
        swerveDrivetrain.stop();
        swerveDrivetrain.resetGyro();
    }

    public void teleopInit() {
        // Reset the braking state in case autonomous exited uncleanly
        swerveDrivetrain.setBraking(BRAKING_DURING_TELEOP);
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
