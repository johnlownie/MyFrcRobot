package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TeleopConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class LockedTelopDriveByPoseCommand extends Command {
    private final SwerveDriveSubsystem swerveDrive;
    private final Supplier<Pose2d> poseProvider;
    private final Supplier<PhotonTrackedTarget> targetSupplier;
    private final Supplier<Rotation2d> robotAngleSupplier;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;

    private final ProfiledPIDController omegaController = TeleopConstants.omegaController;
  
    private final SlewRateLimiter translateXRateLimiter = new SlewRateLimiter(TeleopConstants.X_RATE_LIMIT);
    private final SlewRateLimiter translateYRateLimiter = new SlewRateLimiter(TeleopConstants.Y_RATE_LIMIT);

    private PhotonTrackedTarget selectedTarget;
    private boolean lostTarget;

    /**
     * 
     */
    public LockedTelopDriveByPoseCommand(SwerveDriveSubsystem swerveDrive, Supplier<Pose2d> poseProvider, Supplier<PhotonTrackedTarget> targetSupplier, Supplier<Rotation2d> robotAngleSupplier, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
        this.swerveDrive = swerveDrive;
        this.poseProvider = poseProvider;
        this.targetSupplier = targetSupplier;
        this.robotAngleSupplier = robotAngleSupplier;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;

        this.omegaController.setTolerance(Units.degreesToRadians(2));
        this.omegaController.enableContinuousInput(-Math.PI, Math.PI);

        this.selectedTarget = null;
        this.lostTarget = false;
        
        addRequirements(this.swerveDrive);
    }    

    @Override
    public void end(boolean interrupted) {
        this.swerveDrive.stop();
        this.selectedTarget = null;

        Logger.recordOutput("Commands/Active Command", "");
    }

    @Override
    public void execute() {
        // settings for standard telop drive
        Rotation2d angle = this.robotAngleSupplier.get();
        double xVelocity = this.translateXRateLimiter.calculate(translationXSupplier.getAsDouble());
        double yVelocity = this.translateYRateLimiter.calculate(translationYSupplier.getAsDouble());

        // check for loss of target
        setAndCheckTarget();
        if (this.selectedTarget == null) return;

        // this.omegaController.reset(robotPose.getRotation().getRadians());
        Pose2d robotPose = this.poseProvider.get();

        double goalRotation = angle.getRadians() - Units.degreesToRadians(this.selectedTarget.getYaw());
        this.omegaController.setGoal(goalRotation);
        
        double omegaSpeed = this.omegaController.calculate(robotPose.getRotation().getRadians());
        if (this.omegaController.atGoal()) omegaSpeed = 0;

        this.swerveDrive.drive(xVelocity, yVelocity, omegaSpeed, angle, true);

        Logger.recordOutput("Commands/LockedTelopDriveByPoseCommand/xVelocity", xVelocity);
        Logger.recordOutput("Commands/LockedTelopDriveByPoseCommand/yVelocity", yVelocity);
        Logger.recordOutput("Commands/LockedTelopDriveByPoseCommand/omegaSpeed", omegaSpeed);
        Logger.recordOutput("Commands/LockedTelopDriveByPoseCommand/angle", angle.getDegrees());
        Logger.recordOutput("Commands/LockedTelopDriveByPoseCommand/RadiansToTarget", goalRotation);
        Logger.recordOutput("Commands/LockedTelopDriveByPoseCommand/AngleToTarget", Units.radiansToDegrees(goalRotation));
    }
    
    @Override
    public void initialize() {
        this.lostTarget = false;
        setAndCheckTarget();

        Pose2d robotPose = this.poseProvider.get();
        this.omegaController.reset(robotPose.getRotation().getRadians());

        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    @Override
    public boolean isFinished() {
        return this.lostTarget;
    }

    /**
     * 
     */
    private void setAndCheckTarget() {
        PhotonTrackedTarget target = this.targetSupplier.get();

        if (target == null && this.selectedTarget != null) {
            this.lostTarget = true;
        }
        else if (this.selectedTarget != null && this.selectedTarget.getFiducialId() != target.getFiducialId()) {
            this.lostTarget = true;
        }
        else {
            this.selectedTarget = target;
        }
    }
}
