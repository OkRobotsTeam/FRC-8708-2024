package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Debug;
import frc.robot.InitHelper;
import frc.robot.MathUtils;
import frc.robot.Constants.Shooter.CANIds;


import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Shooter.*;


public class Shooter extends SubsystemBase {
    private final TalonFX topShooter = new TalonFX(CANIds.TOP_SHOOTER, "CTRE");
    private final TalonFX bottomShooter = new TalonFX(CANIds.BOTTOM_SHOOTER, "CTRE");
    private final CANSparkMax shooterRotation = new CANSparkMax(CANIds.SHOOTER_ROTATION, kBrushless);

    private final RelativeEncoder shooterRotationEncoder = shooterRotation.getEncoder();
    private PIDController shooterRotationPID = new PIDController(SHOOTER_ROTATION_PID_KP, SHOOTER_ROTATION_PID_KI,
            SHOOTER_ROTATION_PID_KD);
    // Create a new ArmFeedforward with gains kS, kG, kV, and kA
    // ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV, kA);
    private GenericEntry shooterAngleEntry;
    public int adjustment = -1;
    private int resetCount = 0;
    private InitHelper initHelper = new InitHelper("Shooter", -0.001, 80, 3000, 100);
    private boolean disabled = true;

    public Shooter(GenericEntry shooterAngleEntry) {
        this.shooterAngleEntry = shooterAngleEntry;

        topShooter.setInverted(SHOOTER_TOP_INVERTED);
        bottomShooter.setInverted(SHOOTER_BOTTOM_INVERTED);
        shooterRotation.setInverted(SHOOTER_ROTATION_INVERTED);

        shooterRotationEncoder.setPositionConversionFactor(SHOOTER_ROTATION_GEAR_RATIO);
        shooterRotationEncoder.setVelocityConversionFactor(SHOOTER_ROTATION_GEAR_RATIO);

        shooterRotationEncoder.setPosition(SHOOTER_ROTATION_STARTUP_POSITION);
        shooterRotationPID.setSetpoint(SHOOTER_ROTATION_STARTUP_POSITION);

        shooterRotationPID.setTolerance(0.01);

        // SmartDashboard.putData("Rotation PID: ", shooterRotationPID);
    }

    public void init() {
        disabled = false;
        adjustment = -1;
        setTargetShooterDegreesFromHorizon(0.0);
        shooterRotationPID.reset();
        shooterRotationPID.setSetpoint(SHOOTER_ROTATION_STARTUP_POSITION);
        shooterRotation.set(-0.03);
        initHelper.start(shooterRotationEncoder.getPosition());

    }

    public double getShooterRotationPositionInRotations() {
        return shooterRotationEncoder.getPosition();
    }

    public void setTopShooterSpeed(double speed) {
        topShooter.set(speed);
    }

    public void setBottomShooterSpeed(double speed) {
        bottomShooter.set(speed);
    }

    public void setShooterSpeed(double speed) {
        setTopShooterSpeed(speed);
        setBottomShooterSpeed(speed);
    }

    public void runShooterForward() {
        setShooterSpeed(SHOOTER_FORWARD_SPEED);
        System.out.println("Info: Running shooter forward");
    }

    public void runShooterSlow() {
        setShooterSpeed(SHOOTER_FORWARD_SLOW_SPEED);
        System.out.println("Info: Running shooter forward slow");
    }

    public void runShooterBackward() {
        setShooterSpeed(SHOOTER_REVERSE_SPEED);
        System.out.println("Info: Running shooter backward");
    }

    public void stopShooter() {
        setShooterSpeed(0.0);
        System.out.println("Info: Stopping shooter");
    }

    public void setTargetShooterDegreesFromHorizon(double angle) {
        if (disabled) {
            System.out.println("Warning: Disabled");
        }

        double output = angle / 360.0 + SHOOTER_ROTATION_STARTUP_POSITION;
        System.out.println("Setting setpoint to: " + output);

        shooterRotationPID.setSetpoint(output);
        // SmartDashboard.putNumber("Shooter Angle",
        // getTargetShooterDegreesFromHorizon());
    }

    public double getTargetShooterDegreesFromHorizon() {
        return (shooterRotationPID.getSetpoint() - SHOOTER_ROTATION_STARTUP_POSITION) * 360.0;

    }

    public void shooterRotationReset() {
        setTargetShooterDegreesFromHorizon(0);
        adjustment = 0;
    }

    public void shooterManualAdjustUp() {
        adjustment++;
        updateShooterManualAdjustment();
    }

    public void shooterManualAdjustDown() {
        adjustment--;
        updateShooterManualAdjustment();
    }

    public void updateShooterManualAdjustment() {
        if (adjustment < -1) {
            adjustment = -1;
        }
        if (adjustment == -1) {
            setTargetShooterDegreesFromHorizon(0.0);
        } else {
            setTargetShooterDegreesFromHorizon(SHOOTER_ROTATION_MANUAL_ADJUST_START_DEGREES + (adjustment * 5.0));
        }
    }

    public void setShooterRotationBraking(boolean braking) {
        if (braking) {
            shooterRotation.setIdleMode(CANSparkBase.IdleMode.kBrake);
        } else {
            shooterRotation.setIdleMode(CANSparkBase.IdleMode.kCoast);
        }
    }

    public void disable() {
        disabled = true;
        shooterRotation.set(0);
        topShooter.set(0);
        bottomShooter.set(0);
    }

    public void enable() {
        disabled = false;
    }

    public void update() {
        if (initHelper.initializing(shooterRotationEncoder.getPosition())) {
            return;
        }
        if (initHelper.justFinishedInit()) {
            System.out.println("Setting shooter encoder position to " + SHOOTER_ROTATION_STARTUP_POSITION);
            shooterRotation.set(0);
            shooterRotationEncoder.setPosition(SHOOTER_ROTATION_STARTUP_POSITION - 0.01);
            shooterRotationPID.reset();
            shooterRotationPID.setSetpoint(SHOOTER_ROTATION_STARTUP_POSITION);
            shooterRotationPID.calculate(SHOOTER_ROTATION_STARTUP_POSITION);
            return;
        }
        if (disabled) {
            return;
        }
        setShooterRotationBraking(false);
        double PIDOutput = shooterRotationPID.calculate(getShooterRotationPositionInRotations());
        // PIDOutput= PIDOutput * 0.6;
        // PIDOutput = Math.min(0.2,PIDOutput);
        // PIDOutput = Math.max(-0.2,PIDOutput);

        double gravityCompensationCoefficient = Math.sin(Units.degreesToRadians(getTargetShooterDegreesFromHorizon()))
                * 0.07;

        SmartDashboard.putNumber("Gravity compensation: ", (gravityCompensationCoefficient));
        SmartDashboard.putNumber("PID Output: ", PIDOutput);

        // double gravityCompensationCoefficient =
        // Math.sin(Units.degreesToRadians(getTargetShooterDegreesFromHorizon())) *
        // 0.07;
        // SmartDashboard.putNumber("Gravity compensation: ",
        // (gravityCompensationCoefficient));
        // PIDOutput = PIDOutput + gravityCompensationCoefficient;

        // Debug.debugPrint("Shooter motor power " + PIDOutput + " : " +
        // shooterRotationPID.getSetpoint() + " : " +
        // getShooterRotationPositionInRotations() +
        // " Diff: " + (shooterRotationPID.getSetpoint() -
        // getShooterRotationPositionInRotations()) );

        shooterRotation.set(PIDOutput);
        // }

        shooterAngleEntry.setDouble(Math.round(shooterRotationEncoder.getPosition() * 360));

        SmartDashboard.putNumber("Rotation motor position: ", Math.round(shooterRotationEncoder.getPosition() * 360));
        SmartDashboard.putNumber("Rotation PID setpoint: ", Math.round(shooterRotationPID.getSetpoint() * 360));
    }

    public void autoAngle(BetterPoseEstimator poseEstimator) {
        Translation2d goalOffset = poseEstimator.getOffsetFromGoalInMeters();
        Debug.debugPrint("GDM: " + goalOffset.getNorm());
        SmartDashboard.putString("Offset from Goal (in): ", goalOffset.times(39.3701).toString());

        double goalDistanceInMeters = MathUtils.clamp(goalOffset.getNorm(),0,5);
        Rotation2d angleFromRobotToGoal = goalOffset.getAngle();

        double GOAL_HEIGHT_METERS = 2.2;
        double SHOOTER_HEIGHT = 0.4826;

        Rotation2d angleFromShooterToGoal = Rotation2d
                .fromRadians(Math.atan((GOAL_HEIGHT_METERS - SHOOTER_HEIGHT) / goalDistanceInMeters))
                .minus(Rotation2d.fromDegrees(12));

        if (goalDistanceInMeters > 80) {
            angleFromRobotToGoal.times(goalDistanceInMeters / 80);
        }
        if (angleFromRobotToGoal.getDegrees() < 0) {
            angleFromRobotToGoal = new Rotation2d(0);
        }

        SmartDashboard.putNumber("Distance from Goal (in): ", Units.metersToInches(goalDistanceInMeters));
        SmartDashboard.putNumber("Angle to goal from robot (deg): ", angleFromRobotToGoal.getDegrees());
        SmartDashboard.putNumber("Shooter angle (deg): ", angleFromShooterToGoal.getDegrees());

        setTargetShooterDegreesFromHorizon(angleFromShooterToGoal.getDegrees());
    }

}
