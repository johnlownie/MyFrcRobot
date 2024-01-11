package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.ProfiledPIDController;
import frc.robot.Constants.TeleopConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.modules.vision.VisionModule;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DriveToBestTagCommand extends Command {
    private final SwerveDriveSubsystem swerveDrive;
    private final VisionModule visionModule;
    private final Supplier<Pose2d> poseProvider;
    private final boolean fromFrontCamera;

    private final ProfiledPIDController xController = TeleopConstants.xController;
    private final ProfiledPIDController yController = TeleopConstants.yController;
    private final ProfiledPIDController omegaController = TeleopConstants.omegaController;
    private final Transform3d FIELD_TO_TAG;

    /**
     * 
     */
    public DriveToBestTagCommand(SwerveDriveSubsystem swerveDrive, VisionModule visionModule, Supplier<Pose2d> poseProvider, boolean fromFrontCamera) {
        this.swerveDrive = swerveDrive;
        this.visionModule = visionModule;
        this.poseProvider = poseProvider;
        this.fromFrontCamera = fromFrontCamera;

        this.xController.setTolerance(0.2);
        this.yController.setTolerance(0.2);
        this.omegaController.setTolerance(Units.degreesToRadians(3));
        this.omegaController.enableContinuousInput(-Math.PI, Math.PI);

        double xMetersFromTag = fromFrontCamera ? 0.4 : 0.8;
        double cameraYaw = fromFrontCamera ? Math.PI : 0.0;

        this.FIELD_TO_TAG = new Transform3d(new Translation3d(xMetersFromTag, 0.0, 0.0), new Rotation3d(0.0, 0.0, cameraYaw));
   
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

    /**
     * 
     */
    private Pose2d getBestTagPose(Pose3d currentPose) {
        PhotonTrackedTarget target = this.visionModule.getBestTarget(this.fromFrontCamera);

        if (target == null) return null;
        
        Pose3d cameraPose = currentPose.transformBy(this.fromFrontCamera ? VisionConstants.ROBOT_TO_FRONT_CAMERA : VisionConstants.ROBOT_TO_REAR_CAMERA);

        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d targetPose = cameraPose.transformBy(camToTarget);
        
        return targetPose.transformBy(FIELD_TO_TAG).toPose2d();
    }
  
    @Override
    public void initialize() {
        resetPIDControllers();

        Pose2d robotPose2d = this.poseProvider.get();
        Pose3d robotPose = new Pose3d(robotPose2d.getX(), robotPose2d.getY(), 0.0, new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        Pose2d goalPose = getBestTagPose(robotPose);
        if (goalPose == null) goalPose = robotPose2d;

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
