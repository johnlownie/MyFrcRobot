package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.lib.util.ProfiledPIDController;
import frc.lib.util.Timer;
import frc.robot.Constants.TeleopConstants;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class DrivePathCommand extends PPSwerveControllerCommand {
    private final SwerveDriveSubsystem swerveDrive;
    private final PoseEstimatorSubsystem poseEstimator;
    private final PathPlannerTrajectory trajectory;
    private final boolean initialPath;

    private Timer timer = new Timer();
    
    /**
     * 
     */
    public DrivePathCommand(SwerveDriveSubsystem swerveDrive, PoseEstimatorSubsystem poseEstimator, PathPlannerTrajectory trajectory, boolean initialPath) {
        super(trajectory, poseEstimator::getCurrentPose, swerveDrive.getKinematics(),
            swerveDrive.getXController().getController(), swerveDrive.getYController().getController(), swerveDrive.getOmegaController().getController(),
            swerveDrive::setModuleStates, swerveDrive, poseEstimator);

        this.swerveDrive = swerveDrive;
        this.poseEstimator = poseEstimator;
        this.trajectory = trajectory;
        this.initialPath = initialPath;

        this.swerveDrive.getXController().setTolerance(0.2);
        this.swerveDrive.getYController().setTolerance(0.2);
        this.swerveDrive.getOmegaController().setTolerance(Units.degreesToRadians(3));
        this.swerveDrive.getOmegaController().enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        this.swerveDrive.stop();
        this.timer.stop();

        Logger.getInstance().recordOutput("ActiveCommands/DrivePathCommand", false);
        Logger.getInstance().recordOutput("DrivePathCommand/End Pose", this.poseEstimator.getCurrentPose());
        Logger.getInstance().recordOutput("DrivePathCommand/Elapsed Time", this.timer.get());
     }

    @Override
    public void execute() {
        super.execute();
        
        Pose2d robotPose = this.poseEstimator.getCurrentPose();
        // double xSpeed = this.xController.calculate(robotPose.getX());

        Logger.getInstance().recordOutput("DrivePathCommand/Current Pose", robotPose);
        // Logger.getInstance().recordOutput("DrivePathCommand/xSpeed", xSpeed);
    }

    @Override
    public void initialize() {
        super.initialize();
        
        resetPIDControllers();
        
        this.timer.reset();
        this.timer.start();

        if (this.initialPath) {
            this.poseEstimator.resetOdometry(this.trajectory.getInitialState());
        }

        Logger.getInstance().recordOutput("ActiveCommands/DrivePathCommand", true);
        Logger.getInstance().recordOutput("DrivePathCommand/Initial Pose", this.poseEstimator.getCurrentPose());
        Logger.getInstance().recordOutput("DrivePathCommand/Goal Pose", 7.10);
    }

    /**
     * 
     */
    private void resetPIDControllers() {
        Pose2d robotPose = this.poseEstimator.getCurrentPose();

        this.swerveDrive.getXController().reset(robotPose.getX());
        this.swerveDrive.getYController().reset(robotPose.getY());
        this.swerveDrive.getOmegaController().reset(robotPose.getRotation().getRadians());
    }
}
