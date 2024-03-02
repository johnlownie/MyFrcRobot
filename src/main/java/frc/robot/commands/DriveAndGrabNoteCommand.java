package frc.robot.commands;

import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.ProfiledPIDController;
import frc.robot.Constants.TeleopConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveAndGrabNoteCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final SwerveDriveSubsystem swerveDrive;
    private final Supplier<Rotation2d> robotAngleSupplier;
    private final DoubleSupplier rotationSupplier;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final Supplier<PhotonTrackedTarget> targetSupplier;


    private final ProfiledPIDController xController = TeleopConstants.xController;
    private final ProfiledPIDController yController = TeleopConstants.yController;
    private final ProfiledPIDController omegaController = TeleopConstants.omegaController;
  
    private final SlewRateLimiter translateXRateLimiter = new SlewRateLimiter(TeleopConstants.X_RATE_LIMIT);
    private final SlewRateLimiter translateYRateLimiter = new SlewRateLimiter(TeleopConstants.Y_RATE_LIMIT);
    private final SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(TeleopConstants.ROTATION_RATE_LIMIT);

    private PhotonTrackedTarget selectedTarget;
    private boolean lostTarget;

    /**
     * 
     */
    public DriveAndGrabNoteCommand(SwerveDriveSubsystem swerveDrive, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, Supplier<Rotation2d> robotAngleSupplier, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier, Supplier<PhotonTrackedTarget> target) {
        this.swerveDrive = swerveDrive;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.robotAngleSupplier = robotAngleSupplier;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.targetSupplier = target;

        this.xController.setTolerance(0.2);
        this.yController.setTolerance(0.2);
        this.omegaController.setTolerance(Units.degreesToRadians(3));
        this.omegaController.enableContinuousInput(-Math.PI, Math.PI);

        this.selectedTarget = null;
        this.lostTarget = false;

        addRequirements(this.swerveDrive);
    }

    @Override
    public void end(boolean interrupted) {
        this.swerveDrive.stop();
        this.intakeSubsystem.addAction(IntakeSubsystem.Action.IDLE);
        this.shooterSubsystem.addAction(ShooterSubsystem.Action.IDLE);
        this.selectedTarget = null;
            
        Logger.recordOutput("Commands/Active Command", "");
    }

    @Override
    public void execute() {
        // settings for standard telop drive
        Rotation2d angle = this.robotAngleSupplier.get();
        double xVelocity = this.translateXRateLimiter.calculate(translationXSupplier.getAsDouble());
        double yVelocity = this.translateYRateLimiter.calculate(translationYSupplier.getAsDouble());
        double rVelocity = this.rotationRateLimiter.calculate(rotationSupplier.getAsDouble());

        setTarget();

        // check for visible note
        if (this.selectedTarget != null && this.selectedTarget.getYaw() != 0) {
            // turn on intake
            if (Math.abs(this.selectedTarget.getYaw()) < 10) {
                this.intakeSubsystem.addAction(IntakeSubsystem.Action.INTAKE);
                this.shooterSubsystem.addAction(ShooterSubsystem.Action.INTAKE);
            }

            // reduce yVelocity as yaw gets closer to 0
            yVelocity = Math.min(xVelocity, xVelocity * (this.selectedTarget.getYaw() / 15));
            yVelocity *= -1;

            // increase xVelocity as yaw gets closer to 0
            xVelocity -= xVelocity * (this.selectedTarget.getYaw() / 15) * 0.5;

            // Limit the velocity values
            xVelocity = this.translateXRateLimiter.calculate(xVelocity);
            yVelocity = this.translateYRateLimiter.calculate(yVelocity);
            }

        this.swerveDrive.drive(xVelocity, yVelocity, rVelocity, angle, true);

        Logger.recordOutput("Commands/DriveAndGrabNoteCommand/targetId", this.selectedTarget != null ? this.selectedTarget.getFiducialId() : 0);
        Logger.recordOutput("Commands/DriveAndGrabNoteCommand/targetYaw", this.selectedTarget != null ? this.selectedTarget.getYaw() : 0.0);
        Logger.recordOutput("Commands/DriveAndGrabNoteCommand/xVelocity", xVelocity);
        Logger.recordOutput("Commands/DriveAndGrabNoteCommand/yVelocity", yVelocity);
        Logger.recordOutput("Commands/DriveAndGrabNoteCommand/rVelocity", rVelocity);
        Logger.recordOutput("Commands/DriveAndGrabNoteCommand/angle", angle.getDegrees());
    }
  
    @Override
    public void initialize() {
        this.lostTarget = false;
        setTarget();

        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    @Override
    public boolean isFinished() {
        return this.shooterSubsystem.hasNote() || this.lostTarget;
    }

    /**
     * 
     */
    private void setTarget() {
        PhotonTrackedTarget target = this.targetSupplier.get();

        if (this.selectedTarget != null && this.selectedTarget.getFiducialId() != target.getFiducialId()) {
            this.lostTarget = true;
        }
        else {
            this.selectedTarget = target;
        }
    }
}
