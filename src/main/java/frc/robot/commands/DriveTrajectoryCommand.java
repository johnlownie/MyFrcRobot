package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathPlannerTrajectory;

import frc.lib.pathplanner.PPSwerveControllerCommand;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class DriveTrajectoryCommand extends PPSwerveControllerCommand {
    private final SwerveDriveSubsystem swerveDrive;
    private final PoseEstimatorSubsystem poseEstimator;
    private final PathPlannerTrajectory trajectory;
    private final boolean isInitialPath;

    /**
     * 
     */
    public DriveTrajectoryCommand(SwerveDriveSubsystem swerveDrive, PoseEstimatorSubsystem poseEstimator, PathPlannerTrajectory trajectory, boolean isInitialPath) {
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
        
        Logger.getInstance().recordOutput("Commands/Active Command", "");
    }
    
    @Override
    public void initialize() {
        super.initialize();
        
        Logger.getInstance().recordOutput("Commands/DrivePathCommand/Trajectory", this.trajectory);

        if (this.isInitialPath) {
            // reset odometry to the starting pose of the trajectory
            this.poseEstimator.resetOdometry(this.trajectory.getInitialState());
            this.swerveDrive.setGyroOffset(this.trajectory.getInitialState().holonomicRotation.getDegrees());
        }

        this.swerveDrive.getXController().reset(this.trajectory.getInitialPose().getX());
        this.swerveDrive.getYController().reset(this.trajectory.getInitialPose().getY());
        this.swerveDrive.getOmegaController().reset(this.trajectory.getInitialPose().getRotation().getRadians());

        this.swerveDrive.getXController2().reset();
        this.swerveDrive.getYController2().reset();
        this.swerveDrive.getOmegaController2().reset();

        Logger.getInstance().recordOutput("Commands/Active Command", this.getName());
    }
}
