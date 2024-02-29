package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Arrays;
import java.util.Optional;
import java.util.OptionalDouble;
import static frc.robot.Constants.Limelight.*;


public class Limelight {

    public Limelight() {}

    public Optional<Pose2d> getRobotPose() {
        double[] defaultBotpose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(defaultBotpose);

        if (botpose.length == 0) {
            System.out.println("Couldn't get position from limelight");
            return Optional.empty();
        }

        Translation2d robotPosition = new Translation2d(botpose[1] + (FIELD_WIDTH_IN_METERS / 2), botpose[0]);
        Rotation2d robotRotation = Rotation2d.fromDegrees(botpose[5]);

        return Optional.of(new Pose2d(robotPosition, robotRotation));
    }

    public Optional<Translation2d> getRobotPosition() {
        Optional<Pose2d> robotPose = getRobotPose();
        return robotPose.map(Pose2d::getTranslation);
    }

    public Optional<Rotation2d> getRobotRotation() {
        Optional<Pose2d> robotPose = getRobotPose();
        return robotPose.map(Pose2d::getRotation);
    }

    public OptionalDouble getDistanceFromGoalInMeters() {
        Optional<DriverStation.Alliance> ally = DriverStation.getAlliance();

        // goal is 2.2m off the ground

        Translation2d targetPosition = new Translation2d(0, 0);

        if (ally.isPresent()) {
            if (ally.get() == DriverStation.Alliance.Red) {
                targetPosition = new Translation2d(0.457, 2.616);
            }
            if (ally.get() == DriverStation.Alliance.Blue) {
                targetPosition = new Translation2d(0.457, -16.074);
            }
        } else {
            System.out.println("Warning: No alliance Selected, please select alliance");
        }

        Optional<Translation2d> robotPose = getRobotPosition();

        if (robotPose.isPresent()) {
            return OptionalDouble.of(robotPose.get().getDistance(targetPosition));
        } else {
            return OptionalDouble.empty();
        }
    }
}
