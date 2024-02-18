package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.led.LEDController;
import frc.lib.led.LEDPreset;
import frc.lib.util.ProfiledPIDController;
import frc.lib.util.Timer;
import frc.robot.Constants.TeleopConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignAndShootTagCommand extends Command {
    private final SwerveDriveSubsystem swerveDrive;
    private final ArmSubsystem armSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final Supplier<Pose2d> poseProvider;
    private final boolean fromFrontCamera;

    private final ProfiledPIDController xController = TeleopConstants.xController;
    private final ProfiledPIDController yController = TeleopConstants.yController;
    private final ProfiledPIDController omegaController = TeleopConstants.omegaController;

    private final Timer timer = new Timer();

    private Transform3d ROBOT_TO_TAG;

    /**
     * 
     */
    public AlignAndShootTagCommand(SwerveDriveSubsystem swerveDrive, ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, Supplier<Pose2d> poseProvider, boolean fromFrontCamera) {
        this.swerveDrive = swerveDrive;
        this.armSubsystem = armSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.poseProvider = poseProvider;
        this.fromFrontCamera = fromFrontCamera;

        this.xController.setTolerance(0.2);
        this.yController.setTolerance(0.2);
        this.omegaController.setTolerance(Units.degreesToRadians(3));
        this.omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.swerveDrive);
        addRequirements(this.armSubsystem);
        addRequirements(this.shooterSubsystem);
        addRequirements(this.visionSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        this.timer.stop();
        
        Logger.recordOutput("Commands/Active Command", "");
    }

    @Override
    public void execute() {

    }

    @Override
    public void initialize() {
        resetPIDControllers();

        Pose2d robotPose2d = this.poseProvider.get();
        Pose3d robotPose = new Pose3d(robotPose2d.getX(), robotPose2d.getY(), 0.0, new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        PhotonTrackedTarget target = this.visionSubsystem.getBestTarget(this.fromFrontCamera);

        Pose2d goalPose = getBestTagPose(target, robotPose);
        if (goalPose == null) {
            goalPose = robotPose2d;
            LEDController.set(LEDPreset.Solid.kRed);
        }
        else {
            LEDController.set(LEDPreset.Solid.kGreen);
        }

        this.xController.setGoal(goalPose.getX());
        this.yController.setGoal(goalPose.getY());
        this.omegaController.setGoal(goalPose.getRotation().getRadians());
        
        this.timer.reset();
        this.timer.start();

        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    @Override
    public boolean isFinished() {
        return this.timer.hasElapsed(5.0);
    }

    /**
     * 
     */
    private Pose2d getBestTagPose(PhotonTrackedTarget target, Pose3d currentPose) {
        if (target == null) return null;

        double cameraYaw = this.fromFrontCamera ? Math.PI : 0.0;
        this.ROBOT_TO_TAG = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, cameraYaw));
        
        Pose3d cameraPose = currentPose.transformBy(this.fromFrontCamera ? VisionConstants.ROBOT_TO_FRONT_CAMERA : VisionConstants.ROBOT_TO_REAR_CAMERA);
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d targetPose = cameraPose.transformBy(camToTarget);
        
        return targetPose.transformBy(ROBOT_TO_TAG).toPose2d();
    }

    /**
     * 
     */
    private boolean isAtGoal() {
        return this.xController.atGoal() && this.yController.atGoal() && this.omegaController.atGoal();
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