package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BetterPoseEstimator extends SubsystemBase{
    
    private class OdometryHistoryEntry {
        OdometryHistoryEntry(long time,double x,double y,double r) {
            this.time=time;
            this.x=x;
            this.y=y;
            this.r=r;
        }
        public long time;
        public double x;
        public double y;
        public double r;
        public void update(double diffX, double diffY,double diffR){
            x+=diffX;
            y+=diffY;
            r+=diffR;
        }
        
    }
    public double diffX;
    public double diffY;
    public double diffR;


    public BetterPoseEstimator() {
        
    }

    ArrayList<OdometryHistoryEntry> odometryHistory = new ArrayList<OdometryHistoryEntry>();

    public void newOdometryEntry(Pose2d pose) {
        Translation2d translatedPose = pose.getTranslation().rotateBy(new Rotation2d(diffR));
        odometryHistory.add(0, new OdometryHistoryEntry(System.currentTimeMillis(), translatedPose.getX(), translatedPose.getY(), pose.getRotation().getRadians()));
        if (odometryHistory.size() > 100) {
            odometryHistory.remove(odometryHistory.size() -1);
        }
    }

    public void newVisionEntry(Pose2d visionPose, long visionTime) {
        int matching=0;
        for (int i = 0;  i < odometryHistory.size() ; i++ ) {
            if (odometryHistory.get(i).time <= visionTime) {
                matching = i;
                break;
            }
        }
        if (matching == 0) {
            return;
        }
        //matching contains the index of the entry matching the time the camera saw the targets
        OdometryHistoryEntry historyEntry = odometryHistory.get(matching);
        double newX = visionPose.getX() - historyEntry.x;
        double newY = visionPose.getY() - historyEntry.y;
        double newR = visionPose.getRotation().getRadians() - historyEntry.r;

        //Adjust all odometry histories since the match by the same difference.  Maybe divided by something to remove noise. 
        double weighting = 3;
        OdometryHistoryEntry last = odometryHistory.get(0);
        //System.out.println("Munging:  x:" + fmt(diffX,newX) + " y:" + fmt(diffY,newY) + " r:" + fmt(diffR,newR));
        
        diffX = ((diffX*weighting) + newX ) / (weighting+1);
        diffY = ((diffY*weighting) + newY ) / (weighting+1);
        diffR = ((diffR*weighting) + newR )/ (weighting+1);
        //System.out.println("x:" + fmt(last.x,last.x+diffX) + " y:" + fmt(last.y,last.y+diffY) + " r:" + fmt(last.r,last.r+diffR));

    }
    String fmt(double num1, double num2) {
        return String.format("%2.2f,%2.2f", num1,num2);

    }
    String fmt(double num) {
        return String.format("%2.2f", num);
    }

    public Pose2d getRobotPose() {
        return getCurrentPose();
    }
    
    public Pose2d getCurrentPose() {
        if (odometryHistory.size() < 1) {
            return new Pose2d(0, 0, new Rotation2d(0));
        }
        return new Pose2d(
            odometryHistory.get(0).x + diffX, 
            odometryHistory.get(0).y + diffY, 
            new Rotation2d(odometryHistory.get(0).r+diffR));
    }

    public Translation2d getOffsetFromGoalInMeters() {
        Optional<DriverStation.Alliance> ally = DriverStation.getAlliance();

        // goal is 2.2m off the ground

        Translation2d targetPosition = new Translation2d(0, 0);

        if (ally.isPresent()) {
            if (ally.get() == DriverStation.Alliance.Red) {
                targetPosition = new Translation2d(16.788 - (Units.inchesToMeters(26)), 6.013 - (Units.inchesToMeters(20)));
            }
            if (ally.get() == DriverStation.Alliance.Blue) {
                targetPosition = new Translation2d(1.169 - (Units.inchesToMeters(26)), 6.013 - (Units.inchesToMeters(20)));
            }
        } else {
            System.out.println("Warning: No alliance Selected, please select alliance");
        }

        Translation2d robotPose = getRobotPose().getTranslation();
        return robotPose.minus(targetPosition);
    }


}
