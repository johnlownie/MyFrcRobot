package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class Drive5Meters extends SequentialCommandGroup {
    /**
     * 
     */
    public Drive5Meters(SwerveDriveSubsystem swerveDriveSubsystem, PoseEstimatorSubsystem poseEstimatorSubsystem) {
        Pose2d fiveMeterPose = new Pose2d(7.10, 4.42, Rotation2d.fromDegrees(0));

        addCommands(
            Commands.print("*** Starting DPDrive5Meters ***"),
            new DriveToPoseCommand(swerveDriveSubsystem, poseEstimatorSubsystem::getCurrentPose, fiveMeterPose),
            Commands.print("*** Finished DPDrive5Meters ***")
        );

        addRequirements(swerveDriveSubsystem);
        addRequirements(poseEstimatorSubsystem);
    }
}
