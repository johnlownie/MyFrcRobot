package frc.robot.controls;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AlignAndShootTagCommand;
import frc.robot.commands.DriveAndGrabNoteCommand;
import frc.robot.commands.DriveFromBestTagCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TuningBindings {
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
    public TuningBindings(SwerveDriveSubsystem swerveDrive, PoseEstimatorSubsystem poseEstimator, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, TeleopDriveCommand teleopDriveCommand) {
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
    public void configureDriverButtonBindings(XBoxControlBindings controller) {
        // Teleop Drive
        controller.driveTeleop().ifPresent(
            trigger -> trigger.onTrue(runOnce(() -> this.swerveDrive.setDefaultCommand(this.teleopDriveCommand))
                .andThen(new ScheduleCommand(this.teleopDriveCommand))
            )
        );

        //        
        controller.driverA().ifPresent(
            trigger -> trigger.onTrue(
                new AlignAndShootTagCommand(
                    this.swerveDrive,
                    this.armSubsystem,
                    this.shooterSubsystem,
                    this.visionSubsystem, this.poseEstimator::getCurrentPose,
                    VisionConstants.DATA_FROM_CAMERA
                )
                .until(controller.driverWantsControl())
            )
        );

        //        
        controller.driverB().ifPresent(
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

        // 
        controller.driverY().ifPresent(
            trigger -> trigger.onTrue(
                new DriveFromBestTagCommand(
                    this.swerveDrive,
                    this.visionSubsystem,
                    this.poseEstimator::getCurrentPose,
                    FieldConstants.SPEAKER_POSE_TRANSLATIONS[2],
                    FieldConstants.SPEAKER_POSE_ROTATIONS[2],
                    VisionConstants.DATA_FROM_CAMERA
                )
                // .andThen(
                    // new DeployGamePieceCommand(this.armSubsystem, this.shooterSubsystem, this.visionSubsystem, false)
                    // .until(controller.driverWantsControl())
                // )
                .until(controller.driverWantsControl())
            )
        );
    }

    /**
     * 
     */
    public void configureOperatorButtonBindings(XBoxControlBindings controller) {
        // Arm Bindings
        controller.operatorA().ifPresent(
            trigger -> trigger.onTrue(
                new InstantCommand(() -> {
                    this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_INTAKE);
                })
                .until(controller.operatorWantsControl())
            )
        );

        controller.operatorB().ifPresent(
            trigger -> trigger.onTrue(
                new InstantCommand(() -> {
                    this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_AMP);
                })
                .andThen(
                    new WaitUntilCommand(this.armSubsystem::isAtAngle)
                )
                .andThen(
                    new InstantCommand(() -> {
                        this.shooterSubsystem.addAction(ShooterSubsystem.Action.SHOOT_AMP);
                    })
                )
                .andThen(
                    new WaitUntilCommand(this.shooterSubsystem::hasShot)
                )
                .andThen(
                    new InstantCommand(() -> {
                        this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_INTAKE);
                    })
                )
                .until(controller.operatorWantsControl())
            )
        );

        controller.operatorX().ifPresent(
            trigger -> trigger.onTrue(
                new InstantCommand(() -> {
                    this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_SPEAKER);
                })
                .andThen(
                    new WaitUntilCommand(this.armSubsystem::isAtAngle)
                )
                .andThen(
                    new InstantCommand(() -> {
                        this.shooterSubsystem.addAction(ShooterSubsystem.Action.SHOOT_SPEAKER);
                    })
                )
                .andThen(
                    new WaitUntilCommand(this.shooterSubsystem::hasShot)
                )
                .andThen(
                    new InstantCommand(() -> {
                        this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_INTAKE);
                    })
                )
                .until(controller.operatorWantsControl())
            )
        );

        // Shoot Speaker has highest RPM for motors - set arm angle in shuffleboard
        controller.operatorY().ifPresent(
            trigger -> trigger.onTrue(
                new InstantCommand(() -> {
                    this.shooterSubsystem.addAction(ShooterSubsystem.Action.SHOOT_SPEAKER);
                })
                .until(controller.operatorWantsControl())
            )
        );

        // Intake Bindings
        controller.operatorLeftBumper().ifPresent(
            trigger -> trigger.onTrue(
                new InstantCommand(() -> {
                    this.intakeSubsystem.addAction(IntakeSubsystem.Action.IDLE);
                })
                .until(controller.operatorWantsControl())
            )
        );

        controller.operatorRightBumper().ifPresent(
            trigger -> trigger.onTrue(
                new InstantCommand(() -> {
                    this.intakeSubsystem.addAction(IntakeSubsystem.Action.INTAKE);
                    this.shooterSubsystem.addAction(ShooterSubsystem.Action.INTAKE);
                })
                .until(controller.operatorWantsControl())
            )
        );
    }
}
