package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TeleopConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Command to drive to a pose.
 */
public class DriveFromPoseCommand extends CommandBase {
    private final SwerveDriveSubsystem swerveDrive;
    private final Supplier<Pose2d> poseProvider;

    private final ProfiledPIDController xController = TeleopConstants.xController;
    private final ProfiledPIDController yController = TeleopConstants.yController;
    private final ProfiledPIDController omegaController = TeleopConstants.omegaController;

    private double xDeltaInMeters, yDeltaInMeters, zDeltaInRadians;

    /**
     * Drive to x, y, z delta from current position
     */
    public DriveFromPoseCommand(SwerveDriveSubsystem swerveDrive, Supplier<Pose2d> poseProvider, double xDeltaInMeters, double yDeltaInMeters, double zDeltaInRadians) {
        this.swerveDrive = swerveDrive;
        this.poseProvider = poseProvider;
        this.xDeltaInMeters = xDeltaInMeters;
        this.yDeltaInMeters = yDeltaInMeters;
        this.zDeltaInRadians = zDeltaInRadians;

        addRequirements(this.swerveDrive);
    }

    @Override
    public void end(boolean interrupted) {
        this.swerveDrive.stop();
            
        Logger.getInstance().recordOutput("ActiveCommands/DriveFromPoseCommand", false);
    }

    @Override
    public void execute() {
        Pose2d robotPose = poseProvider.get();

        double xSpeed = xController.calculate(robotPose.getX());
        double ySpeed = yController.calculate(robotPose.getY());
        double omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());

        if (xController.atGoal()) xSpeed = 0;
        if (yController.atGoal()) ySpeed = 0;
        if (omegaController.atGoal()) omegaSpeed = 0;

        this.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
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

        this.xController.reset(goalPose.getX());
        this.yController.reset(goalPose.getY());
        this.omegaController.reset(goalPose.getRotation().getRadians());

        Logger.getInstance().recordOutput("ActiveCommands/DriveFromPoseCommand", true);
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
