package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.led.LEDController;
import frc.lib.led.LEDPreset;
import frc.lib.util.ProfiledPIDController;
import frc.robot.Constants.TeleopConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TargetLockedTeleopDriveCommand extends Command {
    private final SwerveDriveSubsystem swerveDrive;
    private final VisionSubsystem visionSubsystem;
    private final Supplier<Pose2d> poseProvider;
    private final boolean fromFrontCamera;

    private final Supplier<Rotation2d> robotAngleSupplier;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    private final ProfiledPIDController xController = TeleopConstants.xController;
    private final ProfiledPIDController yController = TeleopConstants.yController;
    private final ProfiledPIDController omegaController = TeleopConstants.omegaController;
  
    private final SlewRateLimiter translateXRateLimiter = new SlewRateLimiter(TeleopConstants.X_RATE_LIMIT);
    private final SlewRateLimiter translateYRateLimiter = new SlewRateLimiter(TeleopConstants.Y_RATE_LIMIT);
    private final SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(TeleopConstants.ROTATION_RATE_LIMIT);

    /**
     * 
     */
    public TargetLockedTeleopDriveCommand(SwerveDriveSubsystem swerveDrive, VisionSubsystem visionSubsystem, Supplier<Pose2d> poseProvider, Supplier<Rotation2d> robotAngleSupplier, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier, boolean fromFrontCamera) {
        this.swerveDrive = swerveDrive;
        this.visionSubsystem = visionSubsystem;
        this.poseProvider = poseProvider;
        this.robotAngleSupplier = robotAngleSupplier;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.fromFrontCamera = fromFrontCamera;

        this.xController.setTolerance(0.2);
        this.yController.setTolerance(0.2);
        this.omegaController.setTolerance(Units.degreesToRadians(3));
        this.omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.swerveDrive);
        addRequirements(this.visionSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        this.swerveDrive.stop();

        Logger.recordOutput("Commands/Active Command", "");
    }

    @Override
    public void execute() {
        PhotonTrackedTarget target = this.visionSubsystem.getBestTarget(this.fromFrontCamera);

        if (target == null) {
            // LED indicate no target and drive regular teleop
            LEDController.set(LEDPreset.Solid.kRed);

            Rotation2d angle = this.robotAngleSupplier.get();
            double xVelocity = this.translateXRateLimiter.calculate(translationXSupplier.getAsDouble());
            double yVelocity = this.translateYRateLimiter.calculate(translationYSupplier.getAsDouble());
            double rVelocity = this.rotationRateLimiter.calculate(rotationSupplier.getAsDouble());

            this.swerveDrive.drive(xVelocity, yVelocity, rVelocity, angle, true);
        }
        else {
            // LED indicate and rotate robot to always face target
            LEDController.set(LEDPreset.Solid.kGreen);

            Rotation2d angle = this.robotAngleSupplier.get();
            double xVelocity = this.translateXRateLimiter.calculate(translationXSupplier.getAsDouble());
            double yVelocity = this.translateYRateLimiter.calculate(translationYSupplier.getAsDouble());

            Pose2d robotPose = this.poseProvider.get();
            double goalRotation = robotPose.getRotation().getRadians() - Units.degreesToRadians(target.getYaw());
            this.omegaController.setGoal(goalRotation);
            
            double omegaSpeed = this.omegaController.calculate(robotPose.getRotation().getRadians());
            if (this.omegaController.atGoal()) omegaSpeed = 0;

            this.swerveDrive.drive(xVelocity, yVelocity, omegaSpeed, angle, true);
        }

        // Logger.recordOutput("Commands/TargetLockedTeleopDriveCommand/xVelocity", xVelocity);
        // Logger.recordOutput("Commands/TargetLockedTeleopDriveCommand/yVelocity", yVelocity);
        // Logger.recordOutput("Commands/TargetLockedTeleopDriveCommand/rVelocity", rVelocity);
        // Logger.recordOutput("Commands/TargetLockedTeleopDriveCommand/angle", angle.getDegrees());
    }
    
    @Override
    public void initialize() {
        resetPIDControllers();

        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    /**
     * 
     */
    private void resetPIDControllers() {
        Pose2d robotPose = this.poseProvider.get();

        this.xController.reset(robotPose.getX());
        this.yController.reset(robotPose.getY());
        this.omegaController.reset(robotPose.getRotation().getRadians());
    }
}
