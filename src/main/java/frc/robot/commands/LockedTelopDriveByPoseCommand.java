package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.led.LEDController;
import frc.lib.led.LEDPreset;
import frc.robot.Constants.TeleopConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class LockedTelopDriveByPoseCommand extends Command {
    private final SwerveDriveSubsystem swerveDrive;
    private final Supplier<Pose2d> poseProvider;
    private final Supplier<Pose3d> targetPoseProvider;
    private final Supplier<Rotation2d> robotAngleSupplier;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;

    private final ProfiledPIDController omegaController = TeleopConstants.omegaController;
  
    private final SlewRateLimiter translateXRateLimiter = new SlewRateLimiter(TeleopConstants.X_RATE_LIMIT);
    private final SlewRateLimiter translateYRateLimiter = new SlewRateLimiter(TeleopConstants.Y_RATE_LIMIT);

    private Pose3d targetPose;

    /**
     * 
     */
    public LockedTelopDriveByPoseCommand(SwerveDriveSubsystem swerveDrive, Supplier<Pose2d> poseProvider, Supplier<Pose3d> targetPoseProvider, Supplier<Rotation2d> robotAngleSupplier, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        this.swerveDrive = swerveDrive;
        this.poseProvider = poseProvider;
        this.targetPoseProvider = targetPoseProvider;
        this.robotAngleSupplier = robotAngleSupplier;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;

        this.omegaController.setTolerance(Units.degreesToRadians(2));
        this.omegaController.enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(this.swerveDrive);
    }    

    @Override
    public void end(boolean interrupted) {
        this.swerveDrive.stop();

        Logger.recordOutput("Commands/Active Command", "");
    }

    @Override
    public void execute() {
        Rotation2d angle = this.robotAngleSupplier.get();
        double xVelocity = this.translateXRateLimiter.calculate(translationXSupplier.getAsDouble());
        double yVelocity = this.translateYRateLimiter.calculate(translationYSupplier.getAsDouble());

        Pose2d robotPose = this.poseProvider.get();

        // this.omegaController.reset(robotPose.getRotation().getRadians());
        double goalRotation = -getRadiansToTarget();
        this.omegaController.setGoal(goalRotation);
            
        double omegaSpeed = this.omegaController.calculate(robotPose.getRotation().getRadians());
        if (this.omegaController.atGoal()) omegaSpeed = 0;

        this.swerveDrive.drive(xVelocity, yVelocity, omegaSpeed, angle, true);

        Logger.recordOutput("Commands/LockedTelopDriveByPoseCommand/xVelocity", xVelocity);
        Logger.recordOutput("Commands/LockedTelopDriveByPoseCommand/yVelocity", yVelocity);
        Logger.recordOutput("Commands/LockedTelopDriveByPoseCommand/omegaSpeed", omegaSpeed);
        Logger.recordOutput("Commands/LockedTelopDriveByPoseCommand/angle", angle.getDegrees());
        Logger.recordOutput("Commands/LockedTelopDriveByPoseCommand/RadiansToTarget", getRadiansToTarget());
        Logger.recordOutput("Commands/LockedTelopDriveByPoseCommand/AngleToTarget", Units.radiansToDegrees(getRadiansToTarget()));
        Logger.recordOutput("Commands/LockedTelopDriveByPoseCommand/TargetPose", this.targetPose);
    }
    
    @Override
    public void initialize() {
        this.targetPose = this.targetPoseProvider.get();
        Pose2d robotPose = this.poseProvider.get();
        this.omegaController.reset(robotPose.getRotation().getRadians());

        LEDController.set(this.targetPose == null ? LEDPreset.Solid.kRed : LEDPreset.Solid.kGreen);

        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    /**
     * 
     */
    private double getRadiansToTarget() {
        if (this.targetPose == null) return 0.0;

        Pose2d robotPose = this.poseProvider.get();

        double b = this.targetPose.getY() - robotPose.getY();
        double c = getHypotenuse();

        double sina = (Math.sin(90.0) * b) / c;

        return Math.asin(sina);
    }

    /**
     * 
     */
    private double getHypotenuse() {
        Pose2d robotPose = this.poseProvider.get();

        double a = robotPose.getX();
        double b = this.targetPose.getY() - robotPose.getY();

        double c = (a * a) + (b * b);

        return Math.sqrt(c);
    }
}
