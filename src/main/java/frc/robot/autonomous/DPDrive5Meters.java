package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class DPDrive5Meters extends SequentialCommandGroup {
    /**
     * 
     */
    public DPDrive5Meters(SwerveDriveSubsystem swerveDriveSubsystem, PoseEstimatorSubsystem poseEstimatorSubsystem) {
        Pose2d fiveMeterPose = new Pose2d(FieldConstants.POSE_X + 5.0, FieldConstants.POSE_Y[0], Rotation2d.fromDegrees(0));

        addCommands(
            Commands.print("*** Starting DPDrive5Meters ***"),
            new DriveToPoseCommand(swerveDriveSubsystem, poseEstimatorSubsystem::getCurrentPose, fiveMeterPose),
            Commands.print("*** Finished DPDrive5Meters ***")
        );

        addRequirements(swerveDriveSubsystem);
        addRequirements(poseEstimatorSubsystem);
    }
}
