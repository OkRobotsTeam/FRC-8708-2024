package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class Robot extends TimedRobot {
    private final CommandXboxController controller = new CommandXboxController(0);

    // Slew rate limiters to make joystick inputs less abrupt
    TalonFX m1 = new TalonFX(2);
    TalonFX m2 = new TalonFX(4);
    TalonFX m3 = new TalonFX(6);
    TalonFX m4 = new TalonFX(8);
    
    CANcoder e1 = new CANcoder(10);
    CANcoder e2 = new CANcoder(11);
    CANcoder e3 = new CANcoder(12);
    CANcoder e4 = new CANcoder(13);
    int i=0;


    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(1d / Constants.MOVEMENT_MAX_ACCELERATION_METERS_PER_SECOND);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(1d / Constants.MOVEMENT_MAX_ACCELERATION_METERS_PER_SECOND);
    private final SlewRateLimiter rotateLimiter = new SlewRateLimiter(1d / Constants.TURNING_MAX_ACCELERATION_RADIANS_PER_SECOND);


    @Override
    public void autonomousPeriodic() {
    }


    @Override
    public void teleopPeriodic() {
        driveWithJoystick();
        if (i++%10==0) {
            System.out.println(
            String.format(" %.2f", e1.getAbsolutePosition().getValueAsDouble()) +
            String.format(" %.2f", e2.getAbsolutePosition().getValueAsDouble()) +
            String.format(" %.2f", e3.getAbsolutePosition().getValueAsDouble()) +
            String.format(" %.2f", e4.getAbsolutePosition().getValueAsDouble()));

            //e1.getAbsolutePosition() + ", E2: " + e2.getAbsolutePosition() + "E3: " + e1.getAbsolutePosition() + ", E4: " + e2.getAbsolutePosition());
        }
          // System.out.printf("E1: %0.2f E2: %0.2f E3: %0.2f E4: %0.2f \n",
        //     e1.getAbsolutePosition(),
        //     e2.getAbsolutePosition(),
        //     e3.getAbsolutePosition(),
        //     e4.getAbsolutePosition());
    }


    private void driveWithJoystick() {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -xSpeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), Constants.CONTROLLER_DEADBAND)) * SwerveDrivetrain.MAX_SPEED_METERS_PER_SECOND;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. The Xbox controller
        // returns positive values when you pull to the right by default.
        final var ySpeed = -ySpeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftX(), Constants.CONTROLLER_DEADBAND)) * SwerveDrivetrain.MAX_SPEED_METERS_PER_SECOND;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). The Xbox controller returns positive values when you pull to
        // the right by default.
        final var rot = -rotateLimiter.calculate(MathUtil.applyDeadband(controller.getRightX(), Constants.CONTROLLER_DEADBAND)) * SwerveDrivetrain.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
        final var delme = -controller.getRightY();

        m1.set(controller.getLeftX());
        m2.set(controller.getLeftY());

        m3.set(controller.getRightX());

        m4.set(controller.getRightY());


    }
}
