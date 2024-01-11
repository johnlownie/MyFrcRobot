package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.ProfiledPIDController;
import frc.robot.Constants.TeleopConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Command to drive to a pose.
 */
public class DriveFromPoseCommand extends Command {
    private final SwerveDriveSubsystem swerveDrive;
    private final Supplier<Pose2d> poseProvider;

    private final double xDeltaInMeters, yDeltaInMeters, zDeltaInRadians;

    private final ProfiledPIDController xController = TeleopConstants.xController;
    private final ProfiledPIDController yController = TeleopConstants.yController;
    private final ProfiledPIDController omegaController = TeleopConstants.omegaController;

    /**
     * Drive to x, y, z delta from current position
     */
    public DriveFromPoseCommand(SwerveDriveSubsystem swerveDrive, Supplier<Pose2d> poseProvider, double xDeltaInMeters, double yDeltaInMeters, double zDeltaInRadians) {
        this.swerveDrive = swerveDrive;
        this.poseProvider = poseProvider;
        this.xDeltaInMeters = xDeltaInMeters;
        this.yDeltaInMeters = yDeltaInMeters;
        this.zDeltaInRadians = zDeltaInRadians;

        this.xController.setTolerance(0.2);
        this.yController.setTolerance(0.2);
        this.omegaController.setTolerance(Units.degreesToRadians(3));
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
        Pose2d robotPose = this.poseProvider.get();

        double xSpeed = this.xController.calculate(robotPose.getX());
        double ySpeed = this.yController.calculate(robotPose.getY());
        double omegaSpeed = this.omegaController.calculate(robotPose.getRotation().getRadians());

        if (this.xController.atGoal()) xSpeed = 0;
        if (this.yController.atGoal()) ySpeed = 0;
        if (this.omegaController.atGoal()) omegaSpeed = 0;

        this.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()), false);
    }

    @Override
    public void initialize() {
        resetPIDControllers();

        Pose2d currentPose = this.poseProvider.get();
        Pose2d goalPose = new Pose2d(
            currentPose.getX() + this.xDeltaInMeters, 
            currentPose.getY() + this.yDeltaInMeters, 
            new Rotation2d(currentPose.getRotation().getRadians() + this.zDeltaInRadians)
        );

        this.xController.setGoal(goalPose.getX());
        this.yController.setGoal(goalPose.getY());
        this.omegaController.setGoal(goalPose.getRotation().getRadians());

        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    /**
     * 
     */
    private boolean isAtGoal() {
        return this.xController.atGoal() && this.yController.atGoal() && this.omegaController.atGoal();
    }

    @Override
    public boolean isFinished() {
        return isAtGoal();
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
