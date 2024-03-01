package frc.robot.subsystems;

import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class PoseEstimator {
    private final SwerveDrivetrain drivetrain;
    private final Limelight limelight;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field = new Field2d();

    public PoseEstimator(SwerveDrivetrain drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;

        SmartDashboard.putData("PoseEstimator", field);

        poseEstimator = new SwerveDrivePoseEstimator(
                drivetrain.kinematics,
                drivetrain.getGyroAngle(),
                drivetrain.getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
        );
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginning of
     * a match.
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(
                drivetrain.getGyroAngle(),
                drivetrain.getModulePositions(),
                newPose);
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

    public void periodic() {
        // Update pose estimator with the best visible target

        Optional<Pose2d> visionPose = limelight.getRobotPose();

        visionPose.ifPresent(pose2d -> poseEstimator.addVisionMeasurement(pose2d, System.currentTimeMillis() / 1000.0));

        // Update pose estimator with drivetrain sensors
        poseEstimator.update(
                drivetrain.getGyroAngle(),
                drivetrain.getModulePositions()
        );

        field.setRobotPose(getCurrentPose());
    }
}
