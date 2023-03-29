package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TeleopConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DriveToBestTagCommand extends CommandBase {
    private final SwerveDriveSubsystem swerveDrive;
    private final PhotonCamera photonCamera;
    private final Supplier<Pose2d> poseProvider;

    private final ProfiledPIDController xController = TeleopConstants.xController;
    private final ProfiledPIDController yController = TeleopConstants.yController;
    private final ProfiledPIDController omegaController = TeleopConstants.omegaController;
    private final Transform3d FIELD_TO_TAG;

    private PhotonTrackedTarget lastTarget;
 
    /**
     * 
     */
    public DriveToBestTagCommand(SwerveDriveSubsystem swerveDrive, PhotonCamera photonCamera, Supplier<Pose2d> poseProvider) {
        this.swerveDrive = swerveDrive;
        this.photonCamera = photonCamera;
        this.poseProvider = poseProvider;

        this.xController.setTolerance(0.2);
        this.yController.setTolerance(0.2);
        this.omegaController.setTolerance(Units.degreesToRadians(3));
        this.omegaController.enableContinuousInput(-Math.PI, Math.PI);

        this.FIELD_TO_TAG = new Transform3d(new Translation3d(0.76, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));
   
        addRequirements(this.swerveDrive);
    }

    @Override
    public void end(boolean interrupted) {
        this.swerveDrive.stop();
            
        Logger.getInstance().recordOutput("ActiveCommands/DriveToTagCommand", false);
    }

    @Override
    public void execute() {
        Pose2d robotPose2d = this.poseProvider.get();
        Pose3d robotPose = new Pose3d( robotPose2d.getX(), robotPose2d.getY(), 0.0, new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
        
        PhotonPipelineResult results = this.photonCamera.getLatestResult();
        if (results.hasTargets()) {
            // This is new target data, so recalculate the goal
            PhotonTrackedTarget target = results.getBestTarget();
            this.lastTarget = target;
            
            // Transform the robot's pose to find the camera's pose
            var cameraPose = robotPose.transformBy(VisionConstants.ROBOT_TO_REAR_CAMERA);

            // Trasnform the camera's pose to the target's pose
            var camToTarget = target.getBestCameraToTarget();
            var targetPose = cameraPose.transformBy(camToTarget);
            
            // Transform the tag's pose to set our goal
            var goalPose = targetPose.transformBy(FIELD_TO_TAG).toPose2d();

            // Drive
            this.xController.setGoal(goalPose.getX());
            this.yController.setGoal(goalPose.getY());
            this.omegaController.setGoal(goalPose.getRotation().getRadians());
       }
        
        if (this.lastTarget == null) {
            // No target has been visible
            this.swerveDrive.stop();
        } else {
            // Drive to the target
            double xSpeed = this.xController.calculate(robotPose.getX());
            double ySpeed = this.yController.calculate(robotPose.getY());
            double omegaSpeed = this.omegaController.calculate(robotPose2d.getRotation().getRadians());

            if (this.xController.atGoal()) xSpeed = 0;
            if (this.yController.atGoal()) ySpeed = 0;
            if (this.omegaController.atGoal()) omegaSpeed = 0;

            this.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()));
        }
    }
  
    @Override
    public void initialize() {
        resetPIDControllers();
        
        this.lastTarget = null;
            
        Logger.getInstance().recordOutput("ActiveCommands/DriveToTagCommand", true);
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
