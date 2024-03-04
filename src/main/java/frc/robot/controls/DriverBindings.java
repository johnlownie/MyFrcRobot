package frc.robot.controls;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.commands.DeployGamePieceCommand;
import frc.robot.commands.DriveAndGrabNoteCommand;
import frc.robot.commands.DriveFromBestTagCommand;
import frc.robot.commands.LockedTelopDriveByPoseCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriverBindings {
    /* Subsystems */
    private final PoseEstimatorSubsystem poseEstimator;
    private final ArmSubsystem armSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final SwerveDriveSubsystem swerveDrive;
    private final VisionSubsystem visionSubsystem;

    /* Commands */
    private final TeleopDriveCommand teleopDriveCommand;

    /**
     * 
     */
    public DriverBindings(SwerveDriveSubsystem swerveDrive, PoseEstimatorSubsystem poseEstimator, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, TeleopDriveCommand teleopDriveCommand) {
        this.swerveDrive = swerveDrive;
        this.poseEstimator = poseEstimator;
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.teleopDriveCommand = teleopDriveCommand;
    }

    /**
     * 
     */
    public void configureButtonBindings(XBoxControlBindings controller) {
        // switch from robot relative to field relative
        controller.driveType().ifPresent(
            trigger -> trigger.toggleOnTrue(either(
                runOnce(this.swerveDrive::disableFieldRelative, this.swerveDrive),
                runOnce(this.swerveDrive::enableFieldRelative, this.swerveDrive),
                this.swerveDrive::isFieldRelative)
            )
        );
        
        // Teleop Drive
        controller.driveTeleop().ifPresent(
            trigger -> trigger.onTrue(
                runOnce(() -> this.swerveDrive.setDefaultCommand(this.teleopDriveCommand))
                .andThen(new ScheduleCommand(this.teleopDriveCommand))
            )
        );
        
        // Drive Slower
        controller.slowMode().ifPresent(
            trigger -> trigger.whileTrue(
                runOnce(() -> this.swerveDrive.setSpeedModifier(TeleopConstants.SPEED_MODIFIER_THIRTY))
            )
            .onFalse(
                runOnce(() -> this.swerveDrive.setSpeedModifier(TeleopConstants.SPEED_MODIFIER_ONE_HUNDRED))
            )
        );

        // Teleop Drive but Grab Note if Visible    
        controller.driverA().ifPresent(
            trigger -> trigger.whileTrue(
                new DriveAndGrabNoteCommand(
                    this.swerveDrive,
                    this.intakeSubsystem,
                    this.shooterSubsystem,
                    this.poseEstimator.getCurrentPose()::getRotation,
                    controller.translationX(),
                    controller.translationY(),
                    controller.omega(),
                    this.visionSubsystem::getBestNoteTarget
                )
                .until(this.shooterSubsystem::hasNote)
                // .andThen(new ScheduleCommand(this.teleopDriveCommand))
            )
            // trigger -> trigger.onTrue(
            //     new InstantCommand(() -> {
            //         this.shooterSubsystem.addAction(ShooterSubsystem.Action.SHOOT_SPEAKER);
            //     })
            //     .until(controller.driverWantsControl())
            // )
        );
        
        // Toggle Intake
        controller.toggleIntake().ifPresent(
            trigger -> trigger.whileTrue(
                sequence(
                    runOnce(() -> {
                        this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_INTAKE);
                    }),
                    waitUntil(this.armSubsystem::isAtAngle),
                    runOnce(() -> {
                        this.intakeSubsystem.addAction(IntakeSubsystem.Action.INTAKE);
                        this.shooterSubsystem.addAction(ShooterSubsystem.Action.INTAKE);
                    })
                )
            )
        );
        
        // Toggle Outtake
        controller.toggleOuttake().ifPresent(
            trigger -> trigger.whileTrue(
                runOnce(() -> this.intakeSubsystem.addAction(IntakeSubsystem.Action.EJECT))
            )
        );
        
        // Reset Gyroscope
        controller.zeroGyro().ifPresent(
            trigger -> trigger.onTrue(
                runOnce(() -> this.swerveDrive.zeroGyroscope())
                .andThen(new ScheduleCommand(this.teleopDriveCommand))
            )
        );

        // reset the robot pose
        // driverController.resetPose().ifPresent(trigger -> trigger.onTrue(runOnce(this::resetPose)));

        // Align and Shoot
        // controller.alignAndShoot().ifPresent(
        //     trigger -> trigger.onTrue(
        //         new DriveToBestTagCommand(
        //             this.swerveDrive,
        //             this.visionSubsystem,
        //             this.poseEstimator::getCurrentPose,
        //             false)
        //         .andThen(
        //             new DeployGamePieceCommand(this.armSubsystem, this.shooterSubsystem, this.visionSubsystem, false)
        //             .until(controller.driverWantsControl())
        //         )
        //         .until(controller.driverWantsControl())
        //     )
        // );

        // Drive to Speaker Pose examples
        controller.driveToSpeakerRight().ifPresent(
            trigger -> trigger.onTrue(
                new DriveFromBestTagCommand(
                    this.swerveDrive,
                    this.visionSubsystem,
                    this.poseEstimator::getCurrentPose,
                    FieldConstants.SPEAKER_POSE_TRANSLATIONS[0],
                    FieldConstants.SPEAKER_POSE_ROTATIONS[0],
                    "RearArduCam"
                )
                .andThen(
                    new DeployGamePieceCommand(
                        this.armSubsystem,
                        this.shooterSubsystem,
                        this.visionSubsystem,
                        "RearArduCam")
                    .until(controller.driverWantsControl())
                )
                .until(controller.driverWantsControl())
            )
            .onFalse(this.teleopDriveCommand)
        );

        // 
        controller.driveToSpeakerCenter().ifPresent(
            trigger -> trigger.onTrue(
                new DriveFromBestTagCommand(
                    this.swerveDrive,
                    this.visionSubsystem,
                    this.poseEstimator::getCurrentPose,
                    FieldConstants.SPEAKER_POSE_TRANSLATIONS[1],
                    FieldConstants.SPEAKER_POSE_ROTATIONS[1],
                    "RearArduCam"
                )
                .andThen(
                    new DeployGamePieceCommand(
                        this.armSubsystem,
                        this.shooterSubsystem,
                        this.visionSubsystem,
                        "RearArduCam"
                        )
                    .until(controller.driverWantsControl())
                )
                .until(controller.driverWantsControl())
            )
        );

        // 
        controller.driveToSpeakerLeft().ifPresent(
            trigger -> trigger.onTrue(
                new DriveFromBestTagCommand(
                    this.swerveDrive,
                    this.visionSubsystem,
                    this.poseEstimator::getCurrentPose,
                    FieldConstants.SPEAKER_POSE_TRANSLATIONS[2],
                    FieldConstants.SPEAKER_POSE_ROTATIONS[2],
                    "RearArduCam"
                    )
                .andThen(
                    new DeployGamePieceCommand(
                        this.armSubsystem,
                        this.shooterSubsystem,
                        this.visionSubsystem,
                        "RearArduCam"
                        )
                    .until(controller.driverWantsControl())
                )
                .until(controller.driverWantsControl())
            )
        );

        // Drive with Target Locked
        controller.driverRightStick().ifPresent(
            trigger -> trigger.onTrue(
                new LockedTelopDriveByPoseCommand(
                    this.swerveDrive,
                    () -> this.poseEstimator.getCurrentPose(),
                    () -> this.visionSubsystem.getBestTarget("RearArduCam"),
                    () -> this.poseEstimator.getCurrentPose().getRotation(),
                    controller.translationX(),
                    controller.translationY()
                )
                .until(controller.driverWantsControlRight())
            )
        );
    }
}
