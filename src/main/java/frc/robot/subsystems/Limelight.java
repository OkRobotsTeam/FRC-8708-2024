package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import org.opencv.dnn.Net;

import static frc.robot.Constants.Limelight.*;


public class Limelight extends SubsystemBase {

    private double lastPoseChangeTime = 0.0;
    private long poseChangeMillis = 0;
    private BetterPoseEstimator poseEstimator;
    private Pose2d mostRecentPose = new Pose2d();

    public Limelight(BetterPoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
    }


    public Pose2d getRobotPose() {
        return mostRecentPose;
    }

    public double getPoseChangeTimeMillis() {
        return poseChangeMillis;
    }

    public Translation2d getRobotPosition() {
        return getRobotPose().getTranslation();
    }

    public Rotation2d getRobotRotation() {
        return getRobotPose().getRotation();
    }

    public void update() {
        double[] defaultBotpose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        final NetworkTableEntry botPoseEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose");
        double thisPoseChangeTime = botPoseEntry.getLastChange();

        if (lastPoseChangeTime == thisPoseChangeTime) {
            //nothing has changed. 
            return;
        }
        //Limelight returns valid updated data even if it doesn't detect an image.  tv will be 0 though.
        if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getInteger(0) == 0) {
            //System.out.println("Couldn't get position from limelight");
            return;
        }
        double[] botpose = botPoseEntry.getDoubleArray(defaultBotpose);
        lastPoseChangeTime = thisPoseChangeTime;
        //we have an update
        
        long latency = limelightTable.getEntry("tl").getInteger(0) + limelightTable.getEntry("cl").getInteger(0);
        poseChangeMillis = (System.currentTimeMillis() - latency); 

        Translation2d robotPosition = new Translation2d(botpose[0] + (FIELD_WIDTH_IN_METERS / 2), botpose[1] + (FIELD_HEIGHT_IN_METERS / 2));
        Rotation2d robotRotation = Rotation2d.fromDegrees(botpose[5]);
        mostRecentPose = new Pose2d(robotPosition, robotRotation);
        poseEstimator.newVisionEntry( new Pose2d(robotPosition, robotRotation), poseChangeMillis);
    }

}
