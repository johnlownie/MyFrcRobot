package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import frc.lib.util.PPSwerveControllerCommand;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class DrivePathCommand extends PPSwerveControllerCommand {
    /**
     * 
     */
    public DrivePathCommand(SwerveDriveSubsystem swerveDrive, PoseEstimatorSubsystem poseEstimator, PathPlannerTrajectory trajectory, boolean initialPath) {
        super(trajectory, poseEstimator::getCurrentPose, swerveDrive.getKinematics(),
            swerveDrive.getXController().getController(), swerveDrive.getYController().getController(), swerveDrive.getOmegaController().getController(),
            swerveDrive::setModuleStates, swerveDrive, poseEstimator);
    }
}
