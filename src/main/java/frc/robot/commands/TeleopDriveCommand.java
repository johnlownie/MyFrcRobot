package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TeleopConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Command for teleop driving where translation is field oriented and rotation velocity is controlled by the driver.
 * 
 * Translation is specified on the field-relative coordinate system. The Y-axis runs parallel to the alliance wall, left
 * is positive. The X-axis runs down field toward the opposing alliance wall, away from the alliance wall is positive.
 */
public class TeleopDriveCommand extends CommandBase {
    private final SwerveDriveSubsystem swerveDrive;
    private final Supplier<Rotation2d> robotAngleSupplier;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
  
    private final SlewRateLimiter translateXRateLimiter = new SlewRateLimiter(TeleopConstants.X_RATE_LIMIT);
    private final SlewRateLimiter translateYRateLimiter = new SlewRateLimiter(TeleopConstants.Y_RATE_LIMIT);
    private final SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(TeleopConstants.ROTATION_RATE_LIMIT);

    /**
     * Constructor
     * @param swerveDrive drivetrain
     * @param robotAngleSupplier supplier for the current angle of the robot
     * @param translationXSupplier supplier for translation X component, in meters per second
     * @param translationYSupplier supplier for translation Y component, in meters per second
     * @param rotationSupplier supplier for rotation component, in radians per second
     */
    public TeleopDriveCommand(SwerveDriveSubsystem swerveDrive, Supplier<Rotation2d> robotAngleSupplier, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        this.swerveDrive = swerveDrive;
        this.robotAngleSupplier = robotAngleSupplier;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(swerveDrive);
    }

    @Override
    public void end(boolean interrupted) {
        this.swerveDrive.stop();

        Logger.getInstance().recordOutput("ActiveCommands/TeleopDriveCommand", false);
    }

    @Override
    public void execute() {
        Rotation2d angle = this.robotAngleSupplier.get();
        double xVelocity = this.translateXRateLimiter.calculate(translationXSupplier.getAsDouble());
        double yVelocity = this.translateYRateLimiter.calculate(translationYSupplier.getAsDouble());
        double rVelocity = this.rotationRateLimiter.calculate(rotationSupplier.getAsDouble());

        this.swerveDrive.drive(xVelocity, yVelocity, rVelocity, angle, true);

        Logger.getInstance().recordOutput("TeleopDriveCommand/xVelocity", xVelocity);
        Logger.getInstance().recordOutput("TeleopDriveCommand/yVelocity", yVelocity);
        Logger.getInstance().recordOutput("TeleopDriveCommand/rVelocity", rVelocity);
        Logger.getInstance().recordOutput("TeleopDriveCommand/angle", angle.getDegrees());
    }
    
    @Override
    public void initialize() {
        // use swerve module pid controllers
        this.swerveDrive.resetPIDControllers();

        Logger.getInstance().recordOutput("ActiveCommands/TeleopDriveCommand", true);
    }
}