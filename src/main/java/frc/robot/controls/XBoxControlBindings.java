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
public class XBoxControlBindings implements IControlBindings {
    private final int DRIVER_PORT_ID = 0;
    private final int OPERATOR_PORT_ID = 1;
    
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_PORT_ID);;
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_PORT_ID);;
 
    /**
     * Game Specific Driver Controls
     */

     @Override
     public Optional<Trigger> alignAndShoot() {
         return Optional.of(this.driverController.a());
     }
 
    @Override
    public Optional<Trigger> driveToAmp() {
        return Optional.of(this.driverController.povCenter());
    }

    @Override
    public Optional<Trigger> driveToSpeakerRight() {
        return Optional.of(this.driverController.povRight());
    }

    @Override
    public Optional<Trigger> driveToSpeakerCenter() {
        return Optional.of(this.driverController.povUp());
    }

    @Override
    public Optional<Trigger> driveToSpeakerLeft() {
        return Optional.of(this.driverController.povLeft());
    }

    @Override
    public Optional<Trigger> driveTeleop() {
        return Optional.of(this.driverController.povDown());
    }

    @Override
    public Optional<Trigger> reseedSteerMotors() {
        return Optional.of(this.driverController.start());
    }
  
    @Override
    public Optional<Trigger> resetPose() {
        return Optional.of(this.driverController.back());
    }
 
    /**
     * Game Specific Operator Controls
     */
 
    /**
     * Generic Driver Controls
     */
    @Override
    public Optional<Trigger> driverA() {
        return Optional.of(this.driverController.a());
    }

    @Override
    public Optional<Trigger> driverB() {
        return Optional.of(this.driverController.b());
    }

    @Override
    public Optional<Trigger> driverX() {
        return Optional.of(this.driverController.x());
    }

    @Override
    public Optional<Trigger> driverY() {
        return Optional.of(this.driverController.y());
    }

    @Override
    public Optional<Trigger> driverStart() {
        return Optional.of(this.driverController.start());
    }

    @Override
    public Optional<Trigger> driverBack() {
        return Optional.of(this.driverController.back());
    }

    @Override
    public Optional<Trigger> driverLeftTrigger() {
        return Optional.of(this.driverController.leftTrigger());
    }

    @Override
    public Optional<Trigger> driverRightTrigger() {
        return Optional.of(this.driverController.rightTrigger());
    }

    @Override
    public Optional<Trigger> driverLeftBumper() {
        return Optional.of(this.driverController.leftTrigger());
    }

    @Override
    public Optional<Trigger> driverRightBumper() {
        return Optional.of(this.driverController.rightTrigger());
    }

    @Override
    public Optional<Trigger> driverPovLeft() {
        return Optional.of(this.driverController.povLeft());
    }

    @Override
    public Optional<Trigger> driverPovRight() {
        return Optional.of(this.driverController.povRight());
    }

    @Override
    public Optional<Trigger> driverPovUp() {
        return Optional.of(this.driverController.povUp());
    }

    @Override
    public Optional<Trigger> driverPovDown() {
        return Optional.of(this.driverController.povDown());
    }
 
    /**
     * Generic Operator Controls
     */
    @Override
    public Optional<Trigger> operatorA() {
        return Optional.of(this.operatorController.a());
    }

    @Override
    public Optional<Trigger> operatorB() {
        return Optional.of(this.operatorController.b());
    }

    @Override
    public Optional<Trigger> operatorX() {
        return Optional.of(this.operatorController.x());
    }

    @Override
    public Optional<Trigger> operatorY() {
        return Optional.of(this.operatorController.y());
    }

    @Override
    public Optional<Trigger> operatorStart() {
        return Optional.of(this.operatorController.start());
    }

    @Override
    public Optional<Trigger> operatorBack() {
        return Optional.of(this.operatorController.back());
    }

    @Override
    public Optional<Trigger> operatorLeftTrigger() {
        return Optional.of(this.operatorController.leftTrigger());
    }

    @Override
    public Optional<Trigger> operatorRightTrigger() {
        return Optional.of(this.operatorController.rightTrigger());
    }

    @Override
    public Optional<Trigger> operatorLeftBumper() {
        return Optional.of(this.operatorController.leftTrigger());
    }

    @Override
    public Optional<Trigger> operatorRightBumper() {
        return Optional.of(this.operatorController.rightTrigger());
    }

    @Override
    public Optional<Trigger> operatorPovLeft() {
        return Optional.of(this.operatorController.povLeft());
    }

    @Override
    public Optional<Trigger> operatorPovRight() {
        return Optional.of(this.operatorController.povRight());
    }

    @Override
    public Optional<Trigger> operatorPovUp() {
        return Optional.of(this.operatorController.povUp());
    }

    @Override
    public Optional<Trigger> operatorPovDown() {
        return Optional.of(this.operatorController.povDown());
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
