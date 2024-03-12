package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.pathplanner.lib.auto.AutoBuilder;

import java.util.Arrays;
import java.util.Optional;

import static frc.robot.Constants.SwerveDrivetrain.*;
import static frc.robot.MathUtils.cubicFilter;


/**
 * Represents a swerve drive style drivetrain.
 */
public class SwerveDrivetrain extends SubsystemBase {
    private final Translation2d frontLeftLocation = new Translation2d(WHEELBASE_IN_METERS / 2, WHEELBASE_IN_METERS / 2);
    private final Translation2d frontRightLocation = new Translation2d(WHEELBASE_IN_METERS / 2, -WHEELBASE_IN_METERS / 2);
    private final Translation2d backLeftLocation = new Translation2d(-WHEELBASE_IN_METERS / 2, WHEELBASE_IN_METERS / 2);
    private final Translation2d backRightLocation = new Translation2d(-WHEELBASE_IN_METERS / 2, -WHEELBASE_IN_METERS / 2);
    public final SwerveModule frontLeft = new SwerveModule(CANIds.FRONT_LEFT_DRIVE_MOTOR, CANIds.FRONT_LEFT_ROTATION_MOTOR, CANIds.FRONT_LEFT_ROTATION_ENCODER, "FL");
    private final SwerveModule frontRight = new SwerveModule(CANIds.FRONT_RIGHT_DRIVE_MOTOR, CANIds.FRONT_RIGHT_ROTATION_MOTOR, CANIds.FRONT_RIGHT_ROTATION_ENCODER, "FR");
    private final SwerveModule backLeft = new SwerveModule(CANIds.BACK_LEFT_DRIVE_MOTOR, CANIds.BACK_LEFT_ROTATION_MOTOR, CANIds.BACK_LEFT_ROTATION_ENCODER, "BL");
    private final SwerveModule backRight = new SwerveModule(CANIds.BACK_RIGHT_DRIVE_MOTOR, CANIds.BACK_RIGHT_ROTATION_MOTOR, CANIds.BACK_RIGHT_ROTATION_ENCODER, "BR");
    //private final AHRS gyro = new AHRS(I2C.Port.kMXP);
    private final AHRS gyro = new AHRS();

    public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
    public final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});
    // Slew rate limiters to make joystick inputs less abrupt
    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(MOVEMENT_MAX_ACCELERATION_IN_METERS_PER_SECOND_SQUARED);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(MOVEMENT_MAX_ACCELERATION_IN_METERS_PER_SECOND_SQUARED);
    private final SlewRateLimiter rotateLimiter = new SlewRateLimiter(TURNING_MAX_ANGULAR_ACCELERATION_IN_RADIANS_PER_SECOND_SQUARED);

    private Shooter shooter;
    private Limelight limelight;
    private boolean fieldOriented = false;

    public SwerveDrivetrain(Shooter shooter, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight;

        resetGyro();
        resetOdometry();

        AutoBuilder.configureHolonomic(
                this::getOdometryPose, // Robot pose supplier
                this::setOdometryPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::pathPlannerDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                HOLONOMIC_PATH_FOLLOWER_CONFIG,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        System.out.println("getRobotRelativeSpeeds returning:" + kinematics.toChassisSpeeds(getModuleStates()).toString());
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public void setBraking(boolean braking) {
        frontLeft.setDriveMotorBraking(braking);
        frontRight.setDriveMotorBraking(braking);
        backLeft.setDriveMotorBraking(braking);
        backRight.setDriveMotorBraking(braking);
    }

    public void init() {
        fieldOriented = false;
        setBraking(BRAKING_DURING_TELEOP);
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void toggleFieldOriented() {
        fieldOriented = !fieldOriented;
    }

    public void resetOdometry() {
        setOdometryPose(new Pose2d());
    }

    public void setOdometryPose(Pose2d pose) {

        System.out.println("Just set odometry pose to: " + pose.toString());
        odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }

    public Optional<Rotation2d> getGoalAngle(Limelight limelight) {
        Optional<Translation2d> goalOffset = limelight.getOffsetFromGoalInMeters();

        if (goalOffset.isPresent()) {
            Rotation2d angleFromRobotToGoal = goalOffset.get().getAngle();
            return Optional.of(angleFromRobotToGoal);
        }
        return Optional.empty();
    }

    public void driveWithController(CommandXboxController controller, double driveSpeedScalar, double rotationSpeedScalar, boolean autoAdjust) {
        boolean fast = controller.getRightTriggerAxis() > 0.25;
        boolean slow = controller.getLeftTriggerAxis() > 0.25;
        boolean wheelsCrossed = controller.leftBumper().getAsBoolean();

        //System.out.println("Gyro: " + gyro.getAngle());

        double leftStickYWithDeadzone = MathUtil.applyDeadband(controller.getLeftY(), CONTROLLER_DEADZONE);
        double leftStickXWithDeadzone = MathUtil.applyDeadband(controller.getLeftX(), CONTROLLER_DEADZONE);
        double rightStickXWithDeadzone = MathUtil.applyDeadband(controller.getRightX(), CONTROLLER_DEADZONE);

        double leftStickYWithCubicFilter = cubicFilter(leftStickYWithDeadzone, CONTROLLER_CUBIC_LINEARITY);
        double leftStickXWithCubicFilter = cubicFilter(leftStickXWithDeadzone, CONTROLLER_CUBIC_LINEARITY);
        double rightStickXWithCubicFilter = cubicFilter(rightStickXWithDeadzone, CONTROLLER_CUBIC_LINEARITY);

        double leftStickYWithRateLimit = xSpeedLimiter.calculate(leftStickYWithCubicFilter);
        double leftStickXWithRateLimit = ySpeedLimiter.calculate(leftStickXWithCubicFilter);
        double rightStickXWithRateLimit = rotateLimiter.calculate(rightStickXWithCubicFilter);


        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        double xSpeed = -leftStickYWithRateLimit * MOVEMENT_MAX_SPEED_IN_METERS_PER_SECOND;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. The Xbox controller
        // returns positive values when you pull to the right by default.
        double ySpeed = -leftStickXWithRateLimit * MOVEMENT_MAX_SPEED_IN_METERS_PER_SECOND;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). The Xbox controller returns positive values when you pull to
        // the right by default.
        double rot = -rightStickXWithRateLimit * TURNING_MAX_ANGULAR_VELOCITY_IN_RADIANS_PER_SECOND;

        if (autoAdjust) {
            Optional<Rotation2d> targetRotation = getGoalAngle(limelight);

            if (targetRotation.isPresent()) {
                rot = targetRotation.get().minus(getOdometryRotation()).getRotations();
            } else {
                System.out.println("Warning: autoAdjust failed! Can the limelight see any april tags?");
            }
        }

        // Apply the drive speed selector from ShuffleBoard
        xSpeed *= driveSpeedScalar;
        ySpeed *= driveSpeedScalar;

        // Apply the rotation speed selector from ShuffleBoard
        rot *= rotationSpeedScalar;

        if (slow) {
            // Slow down movements if the slow flag is true
            xSpeed *= 0.5;
            ySpeed *= 0.5;
            rot *= 0.5;
        } else if (fast) {
            // Speed up movements if the fast flag is true and the slow flag is false
            xSpeed *= 2;
            ySpeed *= 2;
        }

        if (wheelsCrossed) {
            // If the wheelsCrossed flag is true then stop all movement and force the wheels into a diamond shape for more traction
            // This is useful if we are being pushed
            frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
            frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
            backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
            backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));

        } else {
            // Otherwise drive normally
            drive(xSpeed, ySpeed, rot, fieldOriented);
        }
    }


    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rotation      Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
//        if (fieldRelative) {
//            System.out.println("Gyro: " + gyro.getRotation2d().getDegrees());
//        }
        var swerveModuleStates = kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, gyro.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MOVEMENT_MAX_SPEED_IN_METERS_PER_SECOND);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    public void pathPlannerDrive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 0.1);

        System.out.println("pathPlannerDrive:" + Arrays.toString(kinematics.toSwerveModuleStates(chassisSpeeds)));

//        swerveModuleStates[0].speedMetersPerSecond = Math.min(Math.max(swerveModuleStates[0].speedMetersPerSecond, -0.1), 0.1);
//        swerveModuleStates[1].speedMetersPerSecond = Math.min(Math.max(swerveModuleStates[1].speedMetersPerSecond, -0.1), 0.1);
//        swerveModuleStates[2].speedMetersPerSecond = Math.min(Math.max(swerveModuleStates[2].speedMetersPerSecond, -0.1), 0.1);
//        swerveModuleStates[3].speedMetersPerSecond = Math.min(Math.max(swerveModuleStates[3].speedMetersPerSecond, -0.1), 0.1);

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Gets the current drivetrain position, as reported by the modules themselves.
     * @return current drivetrain state. Array orders are frontLeft, frontRight, backLeft, backRight
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()};
    }

    /**
     * Updates the field relative position of the robot.
     */
    public void updateOdometry() {
        odometry.update(gyro.getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("Odometry X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry Y", odometry.getPoseMeters().getY());
    }

    public Translation2d getOdometryPosition() {
        return odometry.getPoseMeters().getTranslation();
    }

    public Rotation2d getOdometryRotation() {
        return odometry.getPoseMeters().getRotation();
    }

    public Pose2d getOdometryPose() {
//        System.out.println("getOdometryPose: " + odometry.getPoseMeters().toString());
        return odometry.getPoseMeters();
    }

    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }
}

