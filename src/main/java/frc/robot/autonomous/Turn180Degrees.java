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
public class Turn180Degrees extends SequentialCommandGroup {
    /**
     * 
     */
    public Turn180Degrees(SwerveDriveSubsystem swerveDriveSubsystem, PoseEstimatorSubsystem poseEstimatorSubsystem) {
        Pose2d oneEightyDegreesPose = new Pose2d(FieldConstants.POSE_X, FieldConstants.POSE_Y[0], Rotation2d.fromDegrees(180));

        addCommands(
            Commands.print("*** Starting DPTurn180Degrees ***"),
            new DriveToPoseCommand(swerveDriveSubsystem, poseEstimatorSubsystem::getCurrentPose, oneEightyDegreesPose),
            Commands.print("*** Finished DPTurn180Degrees ***")
        );

        addRequirements(swerveDriveSubsystem);
        addRequirements(poseEstimatorSubsystem);
    }
}
