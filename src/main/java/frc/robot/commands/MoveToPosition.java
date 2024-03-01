package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;

import static frc.robot.Constants.SwerveDrivetrain.*;

public class MoveToPosition extends Command {

    private final double targetHeading;
    private final double speed;
    private final boolean precise;
    private final boolean resetPID;
    private final SwerveDrivetrain drivetrain;
    private final Translation2d targetPosition;

    public MoveToPosition(double targetHeading, Translation2d targetPosition, double speed, SwerveDrivetrain drivetrain, boolean precise, boolean resetPID) {
        this.targetHeading = targetHeading;
        this.targetPosition = targetPosition;
        this.speed = speed;
        this.drivetrain = drivetrain;
        this.precise = precise;
        this.resetPID = resetPID;
        addRequirements(drivetrain);
    }

    double getRemainingDistance() {
        return drivetrain.getOdometryPosition().getDistance(targetPosition);
    }
    @Override
    public void initialize() {
        drivetrain.stop();
        if (resetPID) {
            System.out.println("MoveToPosition: Resetting drivetrain turning PID");
            drivetrain.turningPID.reset();
        } else {
            System.out.println("MoveToPosition: Not resetting drivetrain turning PID");
        }
        drivetrain.turningPID.setSetpoint(targetHeading);

        System.out.println("MoveToPosition: Started moving");
        System.out.println("MoveToPosition: Remaining distance: " + getRemainingDistance());
        System.out.println("MoveToPosition: Current rotation: " + drivetrain.getGyroAngle());
        System.out.println("MoveToPosition: Target rotation: " + drivetrain.turningPID.getSetpoint());
        drivetrain.setBraking(BRAKING_DURING_AUTONOMOUS);
    }

    @Override
    public void execute() {
        Rotation2d moveDirection = targetPosition.minus(drivetrain.getOdometryPosition()).getAngle();
        double xSpeed = moveDirection.getCos() * speed;
        double ySpeed = moveDirection.getSin() * speed;
        double rotationPIDOutput = drivetrain.turningPID.calculate(drivetrain.getOdometryRotation().getDegrees());

        SmartDashboard.putNumber("Rotation PID output: ", rotationPIDOutput * 0.005);

        drivetrain.drive(xSpeed, ySpeed, 0, true);
    }

    @Override
    public boolean isFinished() {
        double distanceToTarget = getRemainingDistance();
        double angleToTarget = targetHeading - drivetrain.getGyroAngle().getDegrees();

        System.out.println("MoveToPosition: Remaining distance: " + distanceToTarget);
        System.out.println("MoveToPosition: Remaining rotation: " + angleToTarget);

        boolean movementComplete = (distanceToTarget < ALLOWED_DISTANCE_FROM_TARGET_IN_METERS);
        boolean turningComplete;
        if (precise) {
            turningComplete =  (Math.abs(angleToTarget) <= ALLOWED_ROTATION_FROM_TARGET_IN_RADIANS);
        } else {
            turningComplete =  (Math.abs(angleToTarget) <= ALLOWED_ROTATION_FROM_TARGET_PRECISE_IN_RADIANS);
        }

        return movementComplete;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        System.out.println("MoveToPosition: Done");
    }
}
