package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
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

    private final ProfiledPIDController xController = TeleopConstants.xController;
    private final ProfiledPIDController yController = TeleopConstants.yController;
    private final ProfiledPIDController omegaController = TeleopConstants.omegaController;

    private Timer timer = new Timer();

    /**
     * 
     */
    public DrivePathCommand(SwerveDriveSubsystem swerveDrive, PoseEstimatorSubsystem poseEstimator, PathPlannerTrajectory trajectory, boolean initialPath) {
        super(trajectory, poseEstimator::getCurrentPose, swerveDrive.getKinematics(),
            new PIDController(2, 0, 0), new PIDController(2, 0, 0), new PIDController(2, 0, 0),
            swerveDrive::setModuleStates, swerveDrive, poseEstimator);

        this.swerveDrive = swerveDrive;
        this.poseEstimator = poseEstimator;
        this.trajectory = trajectory;
        this.initialPath = initialPath;

        // this.xController.setTolerance(0.2);
        // this.yController.setTolerance(0.2);
        // this.omegaController.setTolerance(Units.degreesToRadians(3));
        // this.omegaController.enableContinuousInput(-Math.PI, Math.PI);
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
        
        // resetPIDControllers();
        
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

        this.xController.reset(robotPose.getX());
        this.yController.reset(robotPose.getY());
        this.omegaController.reset(robotPose.getRotation().getRadians());
    }
}
