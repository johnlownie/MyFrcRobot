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

/**
 * 
 */
public class XBoxControlBindings implements ControlBindings {
    private final CommandXboxController driverController = new CommandXboxController(0);;
    private final CommandXboxController operatorController = new CommandXboxController(1);;
 
    /**
     * Driver Controls
     */

    @Override
    public Optional<Trigger> driveToCube() {
        return Optional.of(this.driverController.povCenter());
    }

    @Override
    public Optional<Trigger> driveToPoleLeft() {
        return Optional.of(this.driverController.leftBumper());
    }

    @Override
    public Optional<Trigger> driveToPoleRight() {
        return Optional.of(this.driverController.rightBumper());
    }

    @Override
    public Optional<Trigger> driveTeleop() {
        return Optional.of(this.driverController.povDown());
    }

    @Override
    public Optional<Trigger> driveType() {
        return Optional.of(this.driverController.povUp());
    }

    @Override
    public Optional<Trigger> reseedSteerMotors() {
        return Optional.of(this.driverController.start());
    }
  
    @Override
    public Optional<Trigger> resetPose() {
        return Optional.of(this.driverController.back());
    }
  
    @Override
    public Optional<Trigger> xStance() {
        return Optional.of(this.driverController.x());
    }
 
    /**
     * Operator Controls
     */
     @Override
     public Optional<Trigger> closeGripper() {
         return Optional.of(this.operatorController.a());
     }

     @Override
     public Optional<Trigger> driveToStationLeft() {
         return Optional.of(this.operatorController.leftBumper());
     }
 
     @Override
     public Optional<Trigger> driveToStationRight() {
         return Optional.of(this.operatorController.rightBumper());
     }
 
     @Override
     public Optional<Trigger> extendDrawer() {
         return Optional.of(this.operatorController.x());
     }
  
     @Override
     public Optional<Trigger> openGripper() {
         return Optional.of(this.operatorController.y());
     }

     @Override
     public Optional<Trigger> retractDrawer() {
         return Optional.of(this.operatorController.b());
     }
 
     /**
      * 
      */
    @Override
    public DoubleSupplier translationX() {
        return () ->-modifyAxis(this.driverController.getLeftY()) * SwerveModuleConstants.MAX_VELOCITY_METERS_PER_SECOND;
    }
    @Override
    public DoubleSupplier translationY() {
        return () -> -modifyAxis(this.driverController.getLeftX()) * SwerveModuleConstants.MAX_VELOCITY_METERS_PER_SECOND;
    }
    
    @Override
    public DoubleSupplier omega() {
        return () -> -modifyAxis(this.driverController.getRightX()) * SwerveModuleConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 2;
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

    @Override
    public BooleanSupplier operatorWantsControl() {
      return () -> modifyAxis(this.operatorController.getLeftY()) != 0.0  || modifyAxis(this.operatorController.getLeftX()) != 0.0
          || modifyAxis(this.operatorController.getLeftY()) != 0.0 || modifyAxis(this.operatorController.getLeftX()) != 0.0;
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
