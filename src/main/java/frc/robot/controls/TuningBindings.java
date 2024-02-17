package frc.robot.controls;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.DeployGamePieceCommand;
import frc.robot.commands.DriveToBestTagCommand;
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
                    Commands.print("*** And Then ***"),
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

        controller.operatorY().ifPresent(
            trigger -> trigger.onTrue(
                new InstantCommand(() -> {
                    this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_STAGE);
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
