package frc.robot.controls;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class OperatorBindings {
    /* Subsystems */
    private final PoseEstimatorSubsystem poseEstimator;
    private final ArmSubsystem armSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final SwerveDriveSubsystem swerveDrive;
    private final VisionSubsystem visionSubsystem;

    /**
     * 
     */
    public OperatorBindings(SwerveDriveSubsystem swerveDrive, PoseEstimatorSubsystem poseEstimator, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem) {
        this.swerveDrive = swerveDrive;
        this.poseEstimator = poseEstimator;
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.visionSubsystem = visionSubsystem;
    }

    /**
     * 
     */
    public void configureButtonBindings(XBoxControlBindings controller) {
        // Start/Stop Climb
        controller.operatorStart().ifPresent(
            trigger -> trigger.onTrue(
                runOnce(() -> this.armSubsystem.addAction(ArmSubsystem.Action.CLIMB))
            )
            .onFalse(
                runOnce(() -> this.armSubsystem.addAction(ArmSubsystem.Action.STOP))
            )
        );
    }
}
