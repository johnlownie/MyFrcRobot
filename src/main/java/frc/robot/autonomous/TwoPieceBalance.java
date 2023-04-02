package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.AutoLevelCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.ArmSubsystem.Action;
import frc.robot.utils.AllianceFlipUtil;

/**
 * 
 */
public class TwoPieceBalance extends SequentialCommandGroup {
    /**
     * 
     */
    public TwoPieceBalance(SwerveDriveSubsystem swerveDriveSubsystem, PoseEstimatorSubsystem poseEstimatorSubsystem, ArmSubsystem armSubsystem, Command drivePoleToPiece) {
        Pose2d deployPose = AllianceFlipUtil.apply(FieldConstants.POLE_POSES[0][0]);
        Pose2d firstPiecePose = FieldConstants.GAME_PIECE_POSES[0];
        Pose2d goalPose = AllianceFlipUtil.apply(firstPiecePose);
        Pose2d stationEdge = AllianceFlipUtil.apply(FieldConstants.CHARGE_STATION_EDGE);

        addCommands(
            Commands.print("*** Starting DPTwoPieceBalance ***"),
            new InstantCommand(() -> {
                armSubsystem.addAction(Action.MOVE_TO_DRAWER);
                armSubsystem.addAction(Action.GRAB);
                armSubsystem.addAction(Action.MOVE_TO_MID_NODE);
                armSubsystem.addAction(Action.RELEASE);
                armSubsystem.addAction(Action.PAUSE);
                armSubsystem.addAction(Action.MOVE_TO_GROUND);
            }),
            new WaitUntilCommand(armSubsystem::isReleased),
            new DriveToPoseCommand(swerveDriveSubsystem, poseEstimatorSubsystem::getCurrentPose, firstPiecePose),
            new InstantCommand(() -> {
                armSubsystem.addAction(Action.GRAB);
                armSubsystem.addAction(Action.MOVE_TO_MID_NODE);
            }),
            new DriveToPoseCommand(swerveDriveSubsystem, poseEstimatorSubsystem::getCurrentPose, deployPose),
            new InstantCommand(() -> {
                armSubsystem.addAction(Action.RELEASE);
                armSubsystem.addAction(Action.PAUSE);
                armSubsystem.addAction(Action.MOVE_TO_GROUND);
            }),
            new WaitUntilCommand(armSubsystem::isReleased),
            drivePoleToPiece,
            new InstantCommand(() -> {
                armSubsystem.addAction(Action.GRAB);
                armSubsystem.addAction(Action.MOVE_TO_DRAWER);
            }),
            new DriveToPoseCommand(swerveDriveSubsystem, poseEstimatorSubsystem::getCurrentPose, stationEdge),
            new AutoLevelCommand(swerveDriveSubsystem),
            Commands.print("*** Finished DPTwoPieceBalance ***")
        );

        addRequirements(swerveDriveSubsystem);
        addRequirements(poseEstimatorSubsystem);
        addRequirements(armSubsystem);
    }
}
