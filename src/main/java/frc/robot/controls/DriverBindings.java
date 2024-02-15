package frc.robot.controls;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.DeployGamePieceCommand;
import frc.robot.commands.DriveFromBestTagCommand;
import frc.robot.commands.DriveToBestTagCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriverBindings {
    /* Subsystems */
    private final PoseEstimatorSubsystem poseEstimator;
    private final ArmSubsystem armSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final SwerveDriveSubsystem swerveDrive;
    private final VisionSubsystem visionSubsystem;

    /* Commands */
    private final TeleopDriveCommand teleopDriveCommand;

    /**
     * 
     */
    public DriverBindings(SwerveDriveSubsystem swerveDrive, PoseEstimatorSubsystem poseEstimator, ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, TeleopDriveCommand teleopDriveCommand) {
        this.swerveDrive = swerveDrive;
        this.poseEstimator = poseEstimator;
        this.armSubsystem = armSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.teleopDriveCommand = teleopDriveCommand;
    }

    /**
     * 
     */
    public void configureButtonBindings(XBoxControlBindings driverController) {
        // Teleop Drive
        driverController.driveTeleop().ifPresent(
            trigger -> trigger.onTrue(runOnce(() -> this.swerveDrive.setDefaultCommand(this.teleopDriveCommand))
                .andThen(new ScheduleCommand(this.teleopDriveCommand))));

        // reset the robot pose
        // driverController.resetPose().ifPresent(trigger -> trigger.onTrue(runOnce(this::resetPose)));

        // Start button reseeds the steer motors to fix dead wheel
        driverController.reseedSteerMotors()
            .ifPresent(trigger -> trigger.onTrue(this.swerveDrive.runOnce(this.swerveDrive::reseedSteerMotorOffsets)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)));

        // Align and Shoot
        driverController.alignAndShoot().ifPresent(
            trigger -> trigger.onTrue(
                new DriveToBestTagCommand(
                    this.swerveDrive,
                    this.visionSubsystem,
                    this.poseEstimator::getCurrentPose,
                    false)
                .andThen(
                    new DeployGamePieceCommand(this.armSubsystem, this.shooterSubsystem, this.visionSubsystem, false)
                    .until(driverController.driverWantsControl())
                )
                .until(driverController.driverWantsControl())
            )
        );

        // Drive to Speaker Pose examples
        driverController.driveToSpeakerRight().ifPresent(
            trigger -> trigger.onTrue(
                new DriveFromBestTagCommand(
                    this.swerveDrive,
                    this.visionSubsystem,
                    this.poseEstimator::getCurrentPose,
                    FieldConstants.SPEAKER_POSE_TRANSLATIONS[0],
                    FieldConstants.SPEAKER_POSE_ROTATIONS[0],
                    false)
                .until(driverController.driverWantsControl())
            )
        );

        // 
        driverController.driveToSpeakerCenter().ifPresent(
            trigger -> trigger.onTrue(
                new DriveFromBestTagCommand(
                    this.swerveDrive,
                    this.visionSubsystem,
                    this.poseEstimator::getCurrentPose,
                    FieldConstants.SPEAKER_POSE_TRANSLATIONS[1],
                    FieldConstants.SPEAKER_POSE_ROTATIONS[1],
                    false)
                .until(driverController.driverWantsControl())
            )
        );

        // 
        driverController.driveToSpeakerLeft().ifPresent(
            trigger -> trigger.onTrue(
                new DriveFromBestTagCommand(
                    this.swerveDrive,
                    this.visionSubsystem,
                    this.poseEstimator::getCurrentPose,
                    FieldConstants.SPEAKER_POSE_TRANSLATIONS[2],
                    FieldConstants.SPEAKER_POSE_ROTATIONS[2],
                    false)
                .until(driverController.driverWantsControl())
            )
        );
    }
}
