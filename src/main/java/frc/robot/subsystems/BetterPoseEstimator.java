package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Debug;

public class BetterPoseEstimator extends SubsystemBase {

    private class OdometryHistoryEntry {
        public long time;
        public double x;
        public double y;
        public Rotation2d r;
        OdometryHistoryEntry(long time, double x, double y, Rotation2d r) {
            this.time = time;
            this.x = x;
            this.y = y;
            this.r = r;
        }
    }
    private class VisionHistoryEntry {
        public OdometryHistoryEntry o;
        public double x;
        public double y;
        public Rotation2d r;
        public double speedDiff;
        public VisionHistoryEntry(double x, double y, Rotation2d r, OdometryHistoryEntry o, double speedDiff) {
            this.x = x;
            this.y = y;
            this.r = r;
            this.o = o;
            this.speedDiff = speedDiff;
        }
    }

    public double diffX;
    public double diffY;
    public Rotation2d diffR = Rotation2d.fromDegrees(0);
    ArrayList<OdometryHistoryEntry> odometryHistory = new ArrayList<OdometryHistoryEntry>();
    ArrayList<VisionHistoryEntry> visionHistory = new ArrayList<VisionHistoryEntry>();
    public BetterPoseEstimator() {
        odometryHistory.add(0, new OdometryHistoryEntry(System.currentTimeMillis(), 0, 0, Rotation2d.fromDegrees(0)));
        visionHistory.add(0, new VisionHistoryEntry(0, 0, Rotation2d.fromDegrees(0), odometryHistory.get(0), 0));
    }


    public void newOdometryEntry(Pose2d pose) {
        final OdometryHistoryEntry newEntry = new OdometryHistoryEntry(System.currentTimeMillis(), pose.getX(),
                pose.getY(), pose.getRotation());
        odometryHistory.add(0, newEntry);
        if (odometryHistory.size() > 100) {
            odometryHistory.remove(odometryHistory.size() - 1);
        }
        Translation2d adjustedLocation = translateHistoryToAdjusted(newEntry);
        

    }

    public Translation2d translateHistoryToAdjusted(OdometryHistoryEntry entry) {
        Translation2d translation = new Translation2d(entry.x, entry.y);
        Translation2d newTransation = translation.rotateBy(diffR);
        // Debug.debugPrint("Translation. x " + entry.x + ":" + newTransation.getX() + "
        // y " + entry.y + " : " + newTransation.getY() + " rot:" +
        // Units.radiansToDegrees(diffR));
        return newTransation;
    }

    public Pose2d getAdjustedOdometryPose() {
        Translation2d translated = translateHistoryToAdjusted(odometryHistory.get(0));
        Rotation2d angle = odometryHistory.get(0).r.rotateBy(diffR);
        return new Pose2d(translated, angle);
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
        // matching contains the index of the entry matching the time the camera saw the
        // targets
        Translation2d historyEntry = translateHistoryToAdjusted(odometryHistory.get(matching));
        double newX = visionPose.getX() - historyEntry.getX();
        double newY = visionPose.getY() - historyEntry.getY();
        Rotation2d newR = visionPose.getRotation().minus(odometryHistory.get(matching).r);

        // Adjust all odometry histories since the match by the same difference. Maybe
        // divided by something to remove noise.

        double weighting = 3;
        OdometryHistoryEntry last = odometryHistory.get(0);
        double speedDelta = calculateSpeedDelta(newX, newY, newR,odometryHistory.get(matching), visionHistory.get(0));
        visionHistory.add(0,new VisionHistoryEntry(newX, newY, newR, odometryHistory.get(matching), speedDelta));
        //System.out.println("Speed delta:" + fmt(speedDelta));
        // System.out.println("Munging: x:" + fmt(diffX,newX) + " y:" + fmt(diffY,newY)
        // + " r:" + fmt(diffR,newR));
        // Debug.debugPrint("Rot:" + fmt(visionPose.getRotation().getDegrees()) + " : "
        // + fmt(Units.radiansToDegrees( odometryHistory.get(matching).r)) + " : " +
        // fmt(Units.radiansToDegrees(diffR)));
        double totalDiff = 0;
        if (visionHistory.size() > 10) {
            for (int i = 0; i < 10; i++) {

                totalDiff += visionHistory.get(i).speedDiff;
            }

            //Debug.debugPrint("TotalDiff: " + fmt(totalDiff));

            if (totalDiff < 100) {
                for (int i = 0; i < 10; i++) {
                    var h = visionHistory.get(i);
                    diffX = h.x - h.o.x;

                }
            }
        }
        diffX = ((diffX * weighting) + newX) / (weighting + 1);
        diffY = ((diffY * weighting) + newY) / (weighting + 1);
        diffR = diffR.interpolate(newR, 1.0 / weighting);
        // Debug.debugPrint("x:" + fmt(last.x,last.x+diffX) + " y:" +
        // fmt(last.y,last.y+diffY) + " r:" + fmt(last.r,last.r+diffR));
    }

    public double calculateSpeedDelta(double llx, double lly, Rotation2d llrot, OdometryHistoryEntry thisOdometry,
            VisionHistoryEntry lastHistory) {

        double llMovement = movementMath(llx, lly, lastHistory.x, lastHistory.y);
        double llRotation = Math.abs(llrot.minus(lastHistory.r).getRadians());
        double llCombined = llMovement + llRotation;

        OdometryHistoryEntry lastOdometry = lastHistory.o;
        double odometryMovement = movementMath(thisOdometry.x, thisOdometry.y, lastOdometry.x, lastOdometry.y);
        double odometryRotation = thisOdometry.r.minus(lastOdometry.r).getRadians();
        double odometryCombined = odometryMovement + odometryRotation;
        return((llCombined - odometryCombined) * 100);
    }

    public double movementMath(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
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
        Translation2d adjusted = translateHistoryToAdjusted(odometryHistory.get(0));

        return new Pose2d(
                adjusted.getX() + diffX,
                adjusted.getY() + diffY,
                odometryHistory.get(0).r.plus(diffR));
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
