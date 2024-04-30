package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Debug;

public class BetterPoseEstimator extends SubsystemBase {

    private class OdometryHistoryEntry {
        public long time;
        public Pose2d pose;
        OdometryHistoryEntry(long time, Pose2d pose) {
            this.time = time;
            this.pose = pose;
        }
    }

    private class VisionHistoryEntry {
        public OdometryHistoryEntry o;
        public Pose2d pose;
        public double speedDiff;
        public VisionHistoryEntry(Pose2d pose, OdometryHistoryEntry o, double speedDiff) {
            this.pose = pose;
            this.o = o;
            this.speedDiff = speedDiff;
        }
    }

    public Pose2d anchorOdometry = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public Pose2d anchorVision = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public Rotation2d diffR = Rotation2d.fromDegrees(0);
    ArrayList<OdometryHistoryEntry> odometryHistory = new ArrayList<OdometryHistoryEntry>();
    ArrayList<VisionHistoryEntry> visionHistory = new ArrayList<VisionHistoryEntry>();
    public BetterPoseEstimator() {
        odometryHistory.add(0, new OdometryHistoryEntry(System.currentTimeMillis(), new Pose2d(0, 0, Rotation2d.fromDegrees(0))));
        visionHistory.add(0, new VisionHistoryEntry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), odometryHistory.get(0), 0));
    }


    public void newOdometryEntry(Pose2d pose) {
        final OdometryHistoryEntry newEntry = new OdometryHistoryEntry(System.currentTimeMillis(), pose);
        odometryHistory.add(0, newEntry);
        if (odometryHistory.size() > 100) {
            odometryHistory.remove(odometryHistory.size() - 1);
        }
    }

    public void newVisionEntry(Pose2d visionPose, long visionTime) {
        int matching = 0;
        for (int i = 0; i < odometryHistory.size(); i++) {
            if (odometryHistory.get(i).time <= visionTime) {
                matching = i;
                break;
            }
        }
        if (matching == 0) {
            return;
        }

        var matchingOdometry = odometryHistory.get(matching);
        double speedDelta = calculateSpeedDelta(visionPose,odometryHistory.get(matching), visionHistory.get(0));
        visionHistory.add(0,new VisionHistoryEntry(visionPose, odometryHistory.get(matching), speedDelta));

        double totalDiff = 0;
        if (visionHistory.size() > 10) {
            for (int i = 0; i < 10; i++) {
                totalDiff += visionHistory.get(i).speedDiff;
            }
            Debug.debugPrint("TotalDiff", fmt(totalDiff) + "\n");
            if (totalDiff < 50) {
                anchorOdometry = matchingOdometry.pose;
                anchorVision = visionPose;
                diffR = matchingOdometry.pose.getRotation().minus(visionPose.getRotation());
            }
        }
    }

    public double calculateSpeedDelta(Pose2d visionPose, OdometryHistoryEntry thisOdometry,
            VisionHistoryEntry lastVision) {
        double visionMovement = distanceBetween(visionPose, lastVision.pose);
        double visionRotation = Math.abs(visionPose.getRotation().minus(lastVision.pose.getRotation()).getRadians());
        double visionCombined = (visionMovement + visionRotation * 5) * 100;
        //Debug.debugPrint("Vision Math", "Lateral: " + fmt(visionMovement) + " Rotation:" + fmt(visionRotation) + " vx: " + fmt(visionPose.getX()) + " vy: " + fmt(visionPose.getY()));
        OdometryHistoryEntry lastOdometry = lastVision.o;
        double odometryMovement = distanceBetween(thisOdometry.pose, lastOdometry.pose);
        double odometryRotation = Math.abs(thisOdometry.pose.getRotation().minus(lastOdometry.pose.getRotation()).getRadians());
        double odometryCombined = (odometryMovement + odometryRotation * 5) * 100;
        Debug.debugPrint("Vision Movement: " + fmt(visionCombined) + " Odometry Movement: " + fmt(odometryCombined));
        return(Math.abs(visionCombined - odometryCombined));
    }

    public double distanceBetween(Pose2d pose1, Pose2d pose2) {
        return pose1.getTranslation().getDistance(pose2.getTranslation());
    }

    String fmt(double num1, double num2) {
        return String.format("%2.2f,%2.2f", num1, num2);
    }

    String fmt(double num) {
        return String.format("%2.2f", num);
    }

    public Pose2d getRobotPose() {
        return getCurrentPose();
    }

    public Pose2d getCurrentPose() {
        OdometryHistoryEntry currentOdometry = odometryHistory.get(0);
        Transform2d movement = currentOdometry.pose.minus(anchorOdometry);     
        Pose2d newPosition = anchorVision.transformBy(movement);
        //Debug.debugPrint("posemath",  " A:" + anchorVision.toString() + " MV:" + movement.toString() + " NP:" + newPosition.toString() );
        return (newPosition);
    }

    public Translation2d getOffsetFromGoalInMeters() {
        Optional<DriverStation.Alliance> ally = DriverStation.getAlliance();

        // goal is 2.2m off the ground

        Translation2d targetPosition = new Translation2d(0, 0);

        if (ally.isPresent()) {
            if (ally.get() == DriverStation.Alliance.Red) {
                targetPosition = new Translation2d(16.788 - (Units.inchesToMeters(26)),
                        6.013 - (Units.inchesToMeters(20)));
            }
            if (ally.get() == DriverStation.Alliance.Blue) {
                targetPosition = new Translation2d(1.169 - (Units.inchesToMeters(26)),
                        6.013 - (Units.inchesToMeters(20)));
            }
        } else {
            System.out.println("Warning: No alliance Selected, please select alliance");
        }

        Translation2d robotPose = getRobotPose().getTranslation();
        return robotPose.minus(targetPosition);
    }
}
