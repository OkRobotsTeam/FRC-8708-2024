package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;

import static frc.robot.Constants.Drivetrain.*;

public class MoveToPosition extends Command {

    private final double targetHeading;
    private final double speed;
    private final boolean precise;
    private final SwerveDrivetrain drivetrain;
    private final PIDController turningPID = new PIDController(TURNING_KP, TURNING_KI, TURNING_KD);
    private final Translation2d targetPosition;

    public MoveToPosition(double targetHeading, Translation2d targetPosition, double speed, SwerveDrivetrain drivetrain, boolean precise) {
        this.targetHeading = targetHeading;
        this.targetPosition = targetPosition;
        this.speed = speed;
        this.drivetrain = drivetrain;
        this.precise = precise;
        addRequirements(drivetrain);
    }

    double getRemainingDistance() {
        return drivetrain.getOdometryPosition().getDistance(targetPosition);
    }
    @Override
    public void initialize() {
        drivetrain.stop();
        turningPID.enableContinuousInput(-180.0, 180.0);
        turningPID.setSetpoint(targetHeading);
        System.out.println("MoveToPosition: Started moving");
        System.out.println("MoveToPosition: Remaining distance: " + getRemainingDistance());
        System.out.println("MoveToPosition: Current rotation: " + drivetrain.getGyroAngle());
        System.out.println("MoveToPosition: Target rotation: " + turningPID.getSetpoint());
        drivetrain.setBraking(true);
    }

    @Override
    public void execute() {
        Rotation2d moveDirection = targetPosition.minus(drivetrain.getOdometryPosition()).getAngle();
        double xSpeed = moveDirection.getCos() * speed;
        double ySpeed = moveDirection.getSin() * speed;
        drivetrain.drive(xSpeed, ySpeed, turningPID.calculate(drivetrain.getGyroAngle()), true);
    }

    @Override
    public boolean isFinished() {
        double distanceToTarget = getRemainingDistance();
        double angleToTarget = targetPosition.minus(drivetrain.getOdometryPosition()).getAngle().getDegrees();
        System.out.println("MoveToPosition: Remaining distance: " + distanceToTarget);
        System.out.println("MoveToPosition: Remaining rotation: " + angleToTarget);

        boolean movementComplete = (distanceToTarget < ALLOWED_DISTANCE_FROM_TARGET_METERS);
        boolean turningComplete;
        if (precise) {
            turningComplete =  (angleToTarget < ALLOWED_ROTATION_FROM_TARGET_DEGREES);
        } else {
            turningComplete =  (angleToTarget < ALLOWED_ROTATION_FROM_TARGET_PRECISE_DEGREES);
        }

        return movementComplete && turningComplete;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        System.out.println("MoveToPosition: Done");
    }
}
