package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootFromFarther;
import frc.robot.subsystems.*;

import java.util.Map;
import java.util.Optional;

public class RobotContainer {
    // Controllers
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController manipulatorController = new CommandXboxController(1);

    // Subsystems

    //    private final Climber climber = new Climber();
    private final Intake intake = new Intake();
    private Shooter shooter;
    private final Limelight limelight = new Limelight();
    private final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain(shooter, limelight);
    private final PoseEstimator poseEstimator = new PoseEstimator(swerveDrivetrain, limelight);
    private final USBCameraVision USBDriverCamera = new USBCameraVision();
    final HttpCamera httpCamera = new HttpCamera("Camera", "http://10.14.44.5:8080/?action=stream", HttpCamera.HttpCameraKind.kMJPGStreamer);


    // Shuffleboard
    private final SendableChooser<Double> driveSpeed = new SendableChooser<>();
    private final SendableChooser<Double> turnSpeed = new SendableChooser<>();
    private final SendableChooser<Command> autonomousSelector;

    private final Field2d cameraPositioningField = new Field2d();
    private final Field2d odometryField = new Field2d();
    private final Field2d poseEstimatorField = new Field2d();
    private boolean enabled;

    public RobotContainer() {

        USBDriverCamera.start();

        ShuffleboardTab drivingTab = Shuffleboard.getTab("Driving");


        GenericEntry ShooterAngleEntry = drivingTab.add("Shooter angle", 0).withPosition(0, 0).withSize(2, 1).getEntry();


        drivingTab.add(httpCamera);

        shooter = new Shooter(ShooterAngleEntry);

        // Register Named Commands
        NamedCommands.registerCommand("shoot", new Shoot(swerveDrivetrain, intake, limelight, shooter, poseEstimator));
        NamedCommands.registerCommand("shootFromFarther", new ShootFromFarther(swerveDrivetrain, intake, limelight, shooter, poseEstimator));
        NamedCommands.registerCommand("extendWrist", new InstantCommand(intake::extendWrist));
        NamedCommands.registerCommand("foldWrist", new InstantCommand(intake::foldWrist));
        NamedCommands.registerCommand("runIntakeIn", new InstantCommand(intake::runIntakeIn));
        NamedCommands.registerCommand("runIntakeOut", new InstantCommand(intake::runIntakeOut));
        NamedCommands.registerCommand("stopIntake", new InstantCommand(intake::stopIntake));
        NamedCommands.registerCommand("runShooterForward", new InstantCommand(shooter::runShooterForward));
        NamedCommands.registerCommand("runShooterBackward", new InstantCommand(shooter::runShooterBackward));
        NamedCommands.registerCommand("stopShooter", new InstantCommand(shooter::stopShooter));
        NamedCommands.registerCommand("shooterTo50degrees", new InstantCommand(() -> shooter.setTargetShooterDegreesFromHorizon(50)));
        NamedCommands.registerCommand("shooterTo55degrees", new InstantCommand(() -> shooter.setTargetShooterDegreesFromHorizon(55)));
        NamedCommands.registerCommand("shooterTo60degrees", new InstantCommand(() -> shooter.setTargetShooterDegreesFromHorizon(60)));
        NamedCommands.registerCommand("shooterTo65degrees", new InstantCommand(() -> shooter.setTargetShooterDegreesFromHorizon(65)));
        NamedCommands.registerCommand("shooterManualAdjustUp", new InstantCommand(shooter::shooterManualAdjustUp));
        NamedCommands.registerCommand("shooterManualAdjustDown", new InstantCommand(shooter::shooterManualAdjustDown));
        NamedCommands.registerCommand("done", new InstantCommand(() -> System.out.println("Done autonomous")));

        autonomousSelector = AutoBuilder.buildAutoChooser();

        drivingTab.add( new HttpCamera("limelight",
                        NetworkTableInstance.getDefault().getEntry("limelight_Stream").getString("http://limelight.local:5800/stream.mjpg"),
                        HttpCamera.HttpCameraKind.kMJPGStreamer))
                .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
                .withPosition(2,1)
                .withSize(8,4);

        setupShuffleboard(drivingTab);
        swerveDrivetrain.setDefaultCommand(getSwerveDriveCommand());
        configureControllerBindings();
    }

    private void setupShuffleboard(ShuffleboardTab drivingTab) {

        driveSpeed.setDefaultOption("100%", 1.0);
        driveSpeed.addOption("50%", 0.5);
        driveSpeed.addOption("25%", 0.25);
        driveSpeed.addOption("10%", 0.1);


        turnSpeed.setDefaultOption("100%", 1.0);
        turnSpeed.addOption("50%", 0.5);
        turnSpeed.addOption("25%", 0.25);
        turnSpeed.addOption("10%", 0.1);


        drivingTab.add("Autonomous", autonomousSelector).withPosition(2, 0).withSize(2, 1);
        drivingTab.add("Drive Speed", driveSpeed).withPosition(4, 0).withSize(2, 1);
        drivingTab.add("Turning Speed", turnSpeed).withPosition(6, 0).withSize(2, 1);

        SmartDashboard.putData("Limelight Position", cameraPositioningField);
        SmartDashboard.putData("Odometry Position", odometryField);
        SmartDashboard.putData("Odometry Position", poseEstimatorField);

        Shuffleboard.selectTab("Driving");
        Shuffleboard.update();
    }

    private void configureControllerBindings() {
        swerveDrivetrain.setDefaultCommand(getSwerveDriveCommand());

        manipulatorController.a().onTrue(Commands.runOnce(intake::runIntakeIn));
        manipulatorController.a().onFalse(Commands.runOnce(intake::stopIntake));

        // If the B button is pressed while the wrist should be all the way out, run the intake out at the normal speed
        manipulatorController.b().and(manipulatorController.rightBumper().negate()).onTrue(Commands.runOnce(intake::runIntakeOut));
        // If the B button is pressed while the wrist should be halfway out, run the intake out at full speed
        manipulatorController.b().and(manipulatorController.rightBumper()).onTrue(Commands.runOnce(intake::fullSpeedOut));
        // If the B button is released, stop the intake
        manipulatorController.b().onFalse(Commands.runOnce(intake::stopIntake));

        manipulatorController.x().onTrue(Commands.runOnce(intake::extendWrist));
        manipulatorController.x().onFalse(Commands.runOnce(intake::foldWrist));

        manipulatorController.y().onTrue(Commands.runOnce(shooter::runShooterForward));
        manipulatorController.y().onFalse(Commands.runOnce(shooter::stopShooter));

        manipulatorController.rightBumper().onTrue(Commands.runOnce(intake::halfExtendWrist));
        manipulatorController.rightBumper().onFalse(Commands.runOnce(intake::foldWrist));

        manipulatorController.leftBumper().onTrue(Commands.runOnce(shooter::runShooterSlow));
        manipulatorController.leftBumper().onFalse(Commands.runOnce(shooter::stopShooter));

        manipulatorController.povUp().onTrue(Commands.runOnce(shooter::shooterManualAdjustUp));
        manipulatorController.povDown().onFalse(Commands.runOnce(shooter::shooterManualAdjustDown));
        manipulatorController.povRight().onFalse(Commands.runOnce(shooter::shooterRotationReset));
        manipulatorController.back().onTrue(Commands.runOnce(shooter::init));
        

//        manipulatorController.leftTrigger().onTrue(new InstantCommand(() -> shooter.setTargetShooterDegreesFromHorizon(60)));
        manipulatorController.leftTrigger().onTrue(new InstantCommand(() -> shooter.adjustment = 8).andThen(new InstantCommand(shooter::updateShooterManualAdjustment)));

        manipulatorController.rightTrigger().whileTrue(new InstantCommand(() -> shooter.autoAngle(limelight)));

        driveController.a().onTrue(
                Commands.runOnce(swerveDrivetrain::resetGyro).andThen(
                Commands.runOnce(swerveDrivetrain::resetOdometry)
                )
        );

        driveController.rightBumper().onTrue(Commands.runOnce(swerveDrivetrain::toggleFieldOriented));

    }

    public Command getSwerveDriveCommand() {
        XboxController controller = driveController.getHID();
        return new InstantCommand(() -> swerveDrivetrain.driveWithController(controller, driveSpeed.getSelected(), turnSpeed.getSelected(), driveController.rightBumper().getAsBoolean()), swerveDrivetrain);
    }

    public Command getAutonomousCommand() {
        return autonomousSelector.getSelected();
    }

    public void sleep(double millis) {
      try {
            Thread.sleep(100);
        } catch (Exception e) {
            return;
        }
    }

    public void robotInit() {
        swerveDrivetrain.stop();
        swerveDrivetrain.resetGyro();

    }

    public void teleopInit() {


        // Reset the braking state in case autonomous exited uncleanly
        System.out.println("Starting teleop");
        swerveDrivetrain.init();
//        climber.recalibrateClimber();
        shooter.init();
        intake.init();
    }

    public void testInit() {
        Command test = new InstantCommand(() -> swerveDrivetrain.testWithController(driveController), swerveDrivetrain);
        swerveDrivetrain.setDefaultCommand(test);
        // Reset the braking state in case autonomous exited uncleanly
        System.out.println("Starting test");
        
        swerveDrivetrain.init();
    }

    public void autonomousInit() {
        shooter.init();
        intake.init();
    }



    public void periodic() {
        shooter.periodic();
        intake.periodic();
        odometryField.setRobotPose(swerveDrivetrain.getOdometryPose());
        //Optional<Pose2d> limelightPose = limelight.getRobotPose();
        //limelightPose.ifPresent(cameraPositioningField::setRobotPose);
        //poseEstimatorField.setRobotPose(poseEstimator.getCurrentPose());
    }

    public void enable() {
        shooter.enable();
        intake.enable();
    }

    public void disable() {
        shooter.disable();
        intake.disable();
        swerveDrivetrain.stop();
    }
}
