package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class Robot extends TimedRobot {
    private final CommandXboxController controller = new CommandXboxController(0);
    private final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain();

    // Slew rate limiters to make joystick inputs less abrupt



    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(1d / Constants.MOVEMENT_MAX_ACCELERATION_METERS_PER_SECOND);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(1d / Constants.MOVEMENT_MAX_ACCELERATION_METERS_PER_SECOND);
    private final SlewRateLimiter rotateLimiter = new SlewRateLimiter(1d / Constants.TURNING_MAX_ACCELERATION_RADIANS_PER_SECOND);


    @Override
    public void autonomousPeriodic() {
    }


    @Override
    public void teleopPeriodic() {
        driveWithJoystick();
    }


    private void driveWithJoystick() {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -xSpeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftX(), Constants.CONTROLLER_DEADBAND)) * SwerveDrivetrain.MAX_SPEED_METERS_PER_SECOND;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. The Xbox controller
        // returns positive values when you pull to the right by default.
        final var ySpeed = -ySpeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), Constants.CONTROLLER_DEADBAND)) * SwerveDrivetrain.MAX_SPEED_METERS_PER_SECOND;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). The Xbox controller returns positive values when you pull to
        // the right by default.
        final var rot = -rotateLimiter.calculate(MathUtil.applyDeadband(controller.getRightX(), Constants.CONTROLLER_DEADBAND)) * SwerveDrivetrain.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        swerveDrivetrain.drive(xSpeed, ySpeed, rot, Constants.FIELD_ORIENTED);



    }
}
