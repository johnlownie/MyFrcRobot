package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathPlannerTrajectory;

import frc.lib.pathplanner.PPSwerveControllerCommand;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class DrivePathCommand extends PPSwerveControllerCommand {
    private final SwerveDriveSubsystem swerveDrive;
    private final PoseEstimatorSubsystem poseEstimator;
    private final PathPlannerTrajectory trajectory;
    private final boolean isInitialPath;

    /**
     * 
     */
    public DrivePathCommand(SwerveDriveSubsystem swerveDrive, PoseEstimatorSubsystem poseEstimator, PathPlannerTrajectory trajectory, boolean isInitialPath) {
        super(trajectory, poseEstimator::getCurrentPose, swerveDrive.getKinematics(),
            swerveDrive.getXController().getController(), swerveDrive.getYController().getController(), swerveDrive.getOmegaController().getController(),
            swerveDrive::setModuleStates, swerveDrive, poseEstimator);

        this.swerveDrive = swerveDrive;
        this.poseEstimator = poseEstimator;
        this.trajectory = trajectory;
        this.isInitialPath = isInitialPath;
    }

    @Override
    public void end(boolean interrupted) {
        this.swerveDrive.stop();
        super.end(interrupted);
    }
  
    @Override
    public void initialize() {
        super.initialize();

        // don't use swerve module pid controllers
        this.swerveDrive.zeroPIDControllers();

        if (this.isInitialPath) {
            // reset odometry to the starting pose of the trajectory
            this.poseEstimator.resetOdometry(this.trajectory.getInitialState());
        }

        // reset the theta controller such that old accumulated ID values aren't used with the new path
        //      this doesn't matter if only the P value is non-zero, which is the current behavior
        this.swerveDrive.getXController().reset(this.trajectory.getInitialPose().getX());
        this.swerveDrive.getYController().reset(this.trajectory.getInitialPose().getY());
        this.swerveDrive.getOmegaController().reset(this.trajectory.getInitialPose().getRotation().getRadians());

        Logger.getInstance().recordOutput("DriveToPathCommand/Trajectory Size", this.trajectory.getStates().size());
        Logger.getInstance().recordOutput("DriveToPathCommand/Trajectory Total Time", this.trajectory.getTotalTimeSeconds());
    }
}
