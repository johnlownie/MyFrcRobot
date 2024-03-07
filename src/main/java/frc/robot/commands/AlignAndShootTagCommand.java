package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.led.LEDController;
import frc.lib.led.LEDPreset;
import frc.lib.util.Timer;
import frc.robot.Constants.TeleopConstants;
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
    private final String cameraName;

    private final ProfiledPIDController xController = TeleopConstants.xController;
    private final ProfiledPIDController yController = TeleopConstants.yController;
    private final ProfiledPIDController omegaController = TeleopConstants.omegaController;

    private final Timer timer = new Timer();

    /**
     * 
     */
    public AlignAndShootTagCommand(SwerveDriveSubsystem swerveDrive, ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, Supplier<Pose2d> poseProvider, String cameraName) {
        this.swerveDrive = swerveDrive;
        this.armSubsystem = armSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.poseProvider = poseProvider;
        this.cameraName = cameraName;

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

        Pose2d robotPose2d = this.poseProvider.get();
        Pose3d robotPose = new Pose3d(robotPose2d.getX(), robotPose2d.getY(), 0.0, new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        PhotonTrackedTarget target = this.visionSubsystem.getBestTarget(this.cameraName);
        Pose2d goalPose = robotPose2d;

        if (target == null) {
            LEDController.set(LEDPreset.Solid.kRed);
        }
        else {
            goalPose = getBestTagPose(robotPose, target);
            
            LEDController.set(LEDPreset.Solid.kGreen);
        }

        this.xController.setGoal(goalPose.getX());
        this.yController.setGoal(goalPose.getY());
        this.omegaController.setGoal(goalPose.getRotation().getRadians());
        
        this.timer.reset();
        this.timer.start();

        Logger.recordOutput("Commands/Goal Pose", goalPose);
        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    @Override
    public boolean isFinished() {
        return isAtGoal() || this.timer.hasElapsed(5.0);
    }

    /**
     * Set arm angle based on target pitch
     * Pitch and arm angle are zero relative to the horizon
     * Pitch angle is positive for targets above the horizon
     * Arm angle is negative for targets above the horizon 
     */
    private double getBestArmAngle(PhotonTrackedTarget target) {
        if (target == null) return 0.0;

        if (isBetween(target.getPitch(), 5.0, 20.0)) {
            return 0.0;
        }
        return 0.0;
    }

    /**
     * 
     */
    private Pose2d getBestTagPose(Pose3d currentPose, PhotonTrackedTarget target) {

        Pose3d cameraPose = currentPose.transformBy(this.visionSubsystem.getRobotToCamera(this.cameraName));
        
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d targetPose = cameraPose.transformBy(camToTarget);

        Translation3d goalTranslation = new Translation3d(currentPose.getX(), currentPose.getY(), 0.0);
        Rotation3d goalRotation = new Rotation3d(0.0, 0.0, this.visionSubsystem.getCameraYaw(this.cameraName));
        Transform3d robotToTag = new Transform3d(goalTranslation, goalRotation);

        return targetPose.transformBy(robotToTag).toPose2d();
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
    private boolean isBetween(double value, double lower, double upper) {
        return lower <= value && value <= upper;
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
