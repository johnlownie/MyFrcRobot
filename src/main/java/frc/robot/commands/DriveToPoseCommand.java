package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Command to drive to a pose.
 */
public class DriveToPoseCommand extends CommandBase {
    private final SwerveDriveSubsystem swerveDrive;
    private final Supplier<Pose2d> poseProvider;
    private final Pose2d goalPose;

    private final ProfiledPIDController xController = TeleopConstants.xController;
    private final ProfiledPIDController yController = TeleopConstants.yController;
    private final ProfiledPIDController omegaController = TeleopConstants.omegaController;

    private double xDelta, yDelta, zDelta;

    /**
     * 
     */
    public DriveToPoseCommand(SwerveDriveSubsystem swerveDrive, Supplier<Pose2d> poseProvider, Pose2d goalPose) {
        this.swerveDrive = swerveDrive;
        this.poseProvider = poseProvider;
        this.goalPose = goalPose;

        addRequirements(this.swerveDrive);
    }

    /**
     * Drive to x, y, z delta from current position
     */
    public DriveToPoseCommand(SwerveDriveSubsystem swerveDrive, Supplier<Pose2d> poseProvider, double xDelta, double yDelta, double zDelta) {
        this.swerveDrive = swerveDrive;
        this.poseProvider = poseProvider;
        this.goalPose = null;
        this.xDelta = xDelta;
        this.yDelta = yDelta;
        this.zDelta = zDelta;

        addRequirements(this.swerveDrive);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
            
        Logger.getInstance().recordOutput("ActiveCommands/DriveToPoseCommand", false);
    }

    @Override
    public void execute() {
        // Drive to the goal
        var robotPose = poseProvider.get();

        var xSpeed = xController.calculate(robotPose.getX());
        if (xController.atGoal()) xSpeed = 0;

        var ySpeed = yController.calculate(robotPose.getY());
        if (yController.atGoal()) ySpeed = 0;

        var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
        if (omegaController.atGoal()) omegaSpeed = 0;

        this.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
    }

    @Override
    public void initialize() {
        resetPIDControllers();
        Pose2d pose = goalPose;

        if (pose == null) {
            Pose2d currentPose = this.poseProvider.get();
            pose = new Pose2d(currentPose.getX() + xDelta, currentPose.getY() + yDelta, new Rotation2d(currentPose.getRotation().getRadians() + zDelta));
        }

        this.xController.setGoal(pose.getX());
        this.yController.setGoal(pose.getY());
        this.omegaController.setGoal(pose.getRotation().getRadians());
            
        Logger.getInstance().recordOutput("ActiveCommands/DriveToPoseCommand", true);
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
        Pose2d robotPose = poseProvider.get();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }
}
