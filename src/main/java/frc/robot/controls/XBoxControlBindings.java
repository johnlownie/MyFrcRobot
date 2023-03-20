package frc.robot.controls;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.TeleopConstants;

public class XBoxControlBindings implements ControlBindings {
    private final CommandXboxController driverController = new CommandXboxController(0);;

    @Override
    public Optional<Trigger> driveToCube() {
        return Optional.of(driverController.povCenter());
    }

    @Override
    public Optional<Trigger> driveToPoleLeft() {
        return Optional.of(driverController.leftBumper());
    }

    @Override
    public Optional<Trigger> driveToPoleRight() {
        return Optional.of(driverController.rightBumper());
    }

    @Override
    public Optional<Trigger> driveTeleop() {
        return Optional.of(driverController.povDown());
    }

    @Override
    public Optional<Trigger> driveType() {
        return Optional.of(driverController.povUp());
    }

    @Override
    public Optional<Trigger> reseedSteerMotors() {
        return Optional.of(driverController.start());
    }
  
    @Override
    public Optional<Trigger> resetPose() {
        return Optional.of(driverController.back());
    }
  
    @Override
    public Optional<Trigger> xStance() {
        return Optional.of(driverController.x());
    }
    
    @Override
    public DoubleSupplier translationX() {
        return () ->-modifyAxis(driverController.getLeftY()) * SwerveModuleConstants.MAX_VELOCITY_METERS_PER_SECOND;
    }
    @Override
    public DoubleSupplier translationY() {
        return () -> -modifyAxis(driverController.getLeftX()) * SwerveModuleConstants.MAX_VELOCITY_METERS_PER_SECOND;
    }
    
    @Override
    public DoubleSupplier omega() {
        return () -> -modifyAxis(driverController.getRightX()) * SwerveModuleConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 2;
    }
  
    @Override
    public Supplier<Optional<Rotation2d>> heading() {
        return () -> {
            final var thetaX = -modifyAxis(this.driverController.getLeftY()) * SwerveModuleConstants.MAX_VELOCITY_METERS_PER_SECOND;
            final var thetaY = -modifyAxis(this.driverController.getLeftX()) * SwerveModuleConstants.MAX_VELOCITY_METERS_PER_SECOND;
            final var centered = MathUtil.applyDeadband(thetaX, TeleopConstants.DEADBAND) == 0 && MathUtil.applyDeadband(thetaY, TeleopConstants.DEADBAND) == 0;

            if (centered) {
                // Hold heading when stick is centered
                return Optional.empty();
            }

            // Calculate heading from Y-Axis to X, Y coordinates
            return Optional.of(new Rotation2d(thetaX, thetaY));
        };
    }

    @Override
    public BooleanSupplier driverWantsControl() {
      return () -> modifyAxis(this.driverController.getLeftY()) != 0.0  || modifyAxis(this.driverController.getLeftX()) != 0.0
          || modifyAxis(this.driverController.getLeftY()) != 0.0 || modifyAxis(this.driverController.getLeftX()) != 0.0;
    }
        
    /**
     * 
     */
    private static double modifyAxis(double value) {
        // Deadband
        value = MathUtil.applyDeadband(value, TeleopConstants.DEADBAND);
    
        // Square the axis
        value = Math.copySign(value * value, value);
    
        return value;
    }
}
