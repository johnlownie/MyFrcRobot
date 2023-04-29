package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
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
public class DPTwoPieceBalance extends SequentialCommandGroup {
    /**
     * 
     */
    public DPTwoPieceBalance(SwerveDriveSubsystem swerveDriveSubsystem, PoseEstimatorSubsystem poseEstimatorSubsystem, ArmSubsystem armSubsystem) {
        /* In simulation these poses are not updated when switching stations */
        // Pose2d deployPose = AllianceFlipUtil.apply(FieldConstants.POLE_POSES[0][0]);
        Pose2d deployPose = FieldConstants.POLE_POSES[0][0];
        Pose2d firstPiecePose = FieldConstants.GAME_PIECE_POSES[0];
        Pose2d secondPiecePose = FieldConstants.GAME_PIECE_POSES[1];
        Pose2d stationEdge = FieldConstants.CHARGE_STATION_EDGE;


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
                armSubsystem.addAction(Action.MOVE_TO_HIGH_NODE);
            }),
            new DriveToPoseCommand(swerveDriveSubsystem, poseEstimatorSubsystem::getCurrentPose, deployPose),
            new InstantCommand(() -> {
                armSubsystem.addAction(Action.MOVE_TO_MID_NODE);
                armSubsystem.addAction(Action.RELEASE);
                armSubsystem.addAction(Action.PAUSE);
                armSubsystem.addAction(Action.MOVE_TO_GROUND);
            }),
            new WaitUntilCommand(armSubsystem::isReleased),
            new DriveToPoseCommand(swerveDriveSubsystem, poseEstimatorSubsystem::getCurrentPose, secondPiecePose),
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
