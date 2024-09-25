package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Debug;
import frc.robot.InitHelper;
import frc.robot.MathUtils;
import frc.robot.Constants.Shooter.CANIds;


import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Shooter.*;


public class Shooter extends SubsystemBase {
    private static final int SHOOTER_MANUAL_ADJUST_DEGREES_PER_PRESS = 3;
    public final TalonFX topShooter = new TalonFX(CANIds.TOP_SHOOTER);
    private final TalonFX bottomShooter = new TalonFX(CANIds.BOTTOM_SHOOTER);
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
    private DigitalInput shooterLimitSwitch = new DigitalInput(8);
    private boolean disabled = true;
    private double shooterSpeed = 0;
    private boolean shooting;
//    private final double[] angleByDistanceInFeet = {44,40,33,25,21,20,20,16,14};
    private final double[] angleByDistanceInFeet = {43,43,34,28,23,21,20,16,14,13,13};

//    private enum calibrationStates {}
//    private boolean calibrationState = 0

    public Shooter(GenericEntry shooterAngleEntry) {
        this.shooterAngleEntry = shooterAngleEntry;

        topShooter.setInverted(SHOOTER_TOP_INVERTED);
        bottomShooter.setInverted(SHOOTER_BOTTOM_INVERTED);
        shooterRotation.setInverted(SHOOTER_ROTATION_INVERTED);

        topShooter.setNeutralMode(NeutralModeValue.Coast);
        bottomShooter.setNeutralMode(NeutralModeValue.Coast);

        shooterRotationEncoder.setPositionConversionFactor(SHOOTER_ROTATION_GEAR_RATIO);
        shooterRotationEncoder.setVelocityConversionFactor(SHOOTER_ROTATION_GEAR_RATIO);

        shooterRotationEncoder.setPosition(SHOOTER_ROTATION_STARTUP_POSITION);
        shooterRotationPID.setSetpoint(SHOOTER_ROTATION_STARTUP_POSITION);

        shooterRotationPID.setTolerance(0.01);

        // SmartDashboard.putData("Rotation PID: ", shooterRotationPID);
    }

    public void init() {
        disabled = false;
        stopShooter();
        adjustment = -1;
        setTargetShooterDegreesFromHorizon(0.0);
        shooterRotationPID.reset();
        shooterRotationPID.setSetpoint(SHOOTER_ROTATION_STARTUP_POSITION);

        shooterRotation.set(-0.03);
        initHelper.start(shooterRotationEncoder.getPosition());

    }

//    public void calibrate() {
//
//
//        while (shooterLimitSwitch.get()) {
//
//        }
//    }

    public double getShooterRotationPositionInRotations() {
        return shooterRotationEncoder.getPosition();
    }

    public double getShooterRotationPositionInDegreesFromHorizon() {
        return shooterRotationEncoder.getPosition() / 360.0 + SHOOTER_ROTATION_STARTUP_POSITION;
    }

    public void setTopShooterSpeed(double speed) {
         if (speed == 0){
             topShooter.setControl(new NeutralOut());
         } else {
             //topShooter.setControl(new VelocityVoltage(speed * 16));
             topShooter.set(speed);

         }
//        topShooter.set(speed);

    }

    public void setBottomShooterSpeed(double speed) {
         if (speed == 0){
             bottomShooter.setControl(new NeutralOut());
         } else {
//             bottomShooter.setControl(new VoltageOut(speed * 16));
             bottomShooter.set(speed);
         }
//        bottomShooter.set(speed);

    }

    public void setShooterSpeed(double speed) {
        setTopShooterSpeed(speed);
        setBottomShooterSpeed(speed);
    }

    public void runShooterForward() {
        shooting = true;
        shooterSpeed = 0.1;
        //setShooterSpeed(SHOOTER_FORWARD_SPEED);
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
        shooting = false;
        setShooterSpeed(0.0);
        System.out.println("Info: Stopping shooter");
    }

    public void setTargetShooterDegreesFromHorizon(double angle) {
        if (disabled) {
            System.out.println("Warning: Disabled");
        }

        double output = angle / 360.0 + SHOOTER_ROTATION_STARTUP_POSITION;
        //System.out.println("Setting setpoint to: " + output);

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
            setTargetShooterDegreesFromHorizon(SHOOTER_ROTATION_MANUAL_ADJUST_START_DEGREES + (adjustment * SHOOTER_MANUAL_ADJUST_DEGREES_PER_PRESS));
        }
    }

    public void setShooterAngle(double angle) {
        adjustment = (int) ((angle - SHOOTER_ROTATION_MANUAL_ADJUST_START_DEGREES) / SHOOTER_MANUAL_ADJUST_DEGREES_PER_PRESS);
        setTargetShooterDegreesFromHorizon(angle);
    }



    public void lowerShooter() {
        adjustment = -1;
        setTargetShooterDegreesFromHorizon(0);
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
        setShooterSpeed(0);
        shooterRotation.set(0);
    }

    public boolean limitSwitchPressed() {
        //System.out.println("Limit Switch:" + shooterLimitSwitch.get());
        return shooterLimitSwitch.get();
    }

    public void enable() {
        disabled = false;
    }

    public void update() {



        if (shooting) {
            if (shooterSpeed < SHOOTER_FORWARD_SPEED) {
                shooterSpeed = shooterSpeed + 0.1;
            }
            setShooterSpeed(shooterSpeed);
        }

       if (initHelper.isInitializing(shooterRotationEncoder.getPosition())) {
           if (limitSwitchPressed()) {
               initHelper.setDone();
           } else {
               return;
           }
       }
       if (initHelper.justFinishedInit()) {
           shooterRotation.set(0);
           if (limitSwitchPressed()) {
               System.out.println("Setting shooter encoder position to " + SHOOTER_ROTATION_STARTUP_POSITION);
               shooterRotationEncoder.setPosition(SHOOTER_ROTATION_STARTUP_POSITION);
           } else {
               System.out.println("Setting shooter encoder position to " + (SHOOTER_ROTATION_STARTUP_POSITION - 0.02));
               shooterRotationEncoder.setPosition(SHOOTER_ROTATION_STARTUP_POSITION - 0.02);
           }
           shooterRotationPID.reset();
           shooterRotationPID.calculate(SHOOTER_ROTATION_STARTUP_POSITION);
           return;
       }
       if (disabled) {
           return;
       }

        setShooterRotationBraking(false);
        double PIDOutput = shooterRotationPID.calculate(getShooterRotationPositionInRotations());
        shooterRotation.set(MathUtils.clamp(PIDOutput, -0.2, 0.2));
        // Debug.debugPrint("ShooterPID Status "
        //         + " Position: " + fmt(getShooterRotationPositionInRotations())
        //         + " Setpoint: " + fmt(shooterRotationPID.getSetpoint())
        //         + " Error: " + fmt(shooterRotationPID.getPositionError())
        //         + " Output: " + fmt(PIDOutput));

        shooterAngleEntry.setDouble(Math.round(shooterRotationEncoder.getPosition() * 360));
        SmartDashboard.putNumber("PID Output: ", PIDOutput);
        SmartDashboard.putNumber("Rotation motor position: ", Math.round(shooterRotationEncoder.getPosition() * 360));
        SmartDashboard.putNumber("Rotation PID setpoint: ", Math.round(shooterRotationPID.getSetpoint() * 360));
    }

    public String fmt(double number) {
        return Debug.fourPlaces(number);
    }

    private double interpolate(double start_value, double end_value, double t) {
        /*
        Perform linear interpolation between two values.
        Args:
            start_value: The value to start at.
            end_value: The value to end at.
            t: How far between the two values to interpolate. With zero corresponding to start_value and 1 corresponding to end_value.
            This is clamped to the range [0, 1].

        Returns:
            The interpolated value.
        */

        return start_value + (end_value - start_value) * MathUtils.clamp(t, 0, 1);
    }

    public void autoAngle(BetterPoseEstimator poseEstimator) {
        Translation2d goalOffset = poseEstimator.getOffsetFromGoalInMeters();
        double offsetInFeet = Units.metersToFeet(goalOffset.getNorm());
        offsetInFeet = offsetInFeet - 3;
        System.out.println("autoAim"+  "Disance to goal in feet " + fmt(offsetInFeet)); 
        int below = (int) Math.floor(offsetInFeet);
        int above = (int) Math.ceil(offsetInFeet);
        below = MathUtils.clamp(below, 0, angleByDistanceInFeet.length - 1);
        above = MathUtils.clamp(above, 0, angleByDistanceInFeet.length - 1);
        double factor = offsetInFeet - below;
        

        double angle = interpolate(angleByDistanceInFeet[below], angleByDistanceInFeet[above], factor);
        
        
        Debug.debugPrint("autoAngle", " Distance from goal in feet: " + fmt(offsetInFeet) 
            + " below: " + below 
            + " above: " + above 
            + " factor " + factor 
            + " Resulting Angle: " + angle);
        
        setTargetShooterDegreesFromHorizon(angle);


        //SmartDashboard.putString("Offset from Goal (in): ", goalOffset.times(39.3701).toString());

        // double goalDistanceInMeters = MathUtils.clamp(goalOffset.getNorm(),0,5);
        // Rotation2d angleFromRobotToGoal = goalOffset.getAngle();

        // double GOAL_HEIGHT_METERS = 2.2;
        // double SHOOTER_HEIGHT = 0.4826;

        // Rotation2d angleFromShooterToGoal = Rotation2d
        //         .fromRadians(Math.atan((GOAL_HEIGHT_METERS - SHOOTER_HEIGHT) / goalDistanceInMeters))
        //         .minus(Rotation2d.fromDegrees(12));

        // if (goalDistanceInMeters > 80) {
        //     angleFromRobotToGoal.times(goalDistanceInMeters / 80);
        // }
        // if (angleFromRobotToGoal.getDegrees() < 0) {
        //     angleFromRobotToGoal = new Rotation2d(0);
        // }

        // SmartDashboard.putNumber("Distance from Goal (in): ", Units.metersToInches(goalDistanceInMeters));
        // SmartDashboard.putNumber("Angle to goal from robot (deg): ", angleFromRobotToGoal.getDegrees());
        // SmartDashboard.putNumber("Shooter angle (deg): ", angleFromShooterToGoal.getDegrees());

        // setTargetShooterDegreesFromHorizon(angleFromShooterToGoal.getDegrees());
    }

}
