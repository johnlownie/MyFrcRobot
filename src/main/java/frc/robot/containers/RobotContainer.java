package frc.robot.containers;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.lib.util.Timer;
import frc.robot.Constants.FieldConstants;
import frc.robot.autonomous.AutoBuilder;
import frc.robot.commands.DriveFromBestTagCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.modules.gyro.GyroModule;
import frc.robot.modules.intake.IntakeModule;
import frc.robot.modules.shooter.ShooterModule;
import frc.robot.modules.swerve.SwerveModule;
import frc.robot.modules.vision.VisionModule;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utils.AllianceFlipUtil;

/**
 * 
 */
abstract public class RobotContainer {
   /* Autonomous */

    /* Commands */
    protected TeleopDriveCommand teleopDriveCommand;

    /* Controllers */
    protected final XBoxControlBindings driverController;
    protected final XBoxControlBindings operatorController;

    /* Modules */
    protected GyroModule gyroModule;
    protected SwerveModule[] swerveModules;
    protected VisionModule visionModule;

    /* Subsystems */
    protected ArmSubsystem armSubsystem;
    protected IntakeSubsystem intakeSubsystem;
    protected ShooterSubsystem shooterSubsystem;
    protected PneumaticSubsystem pneumaticSubsystem;
    protected PoseEstimatorSubsystem poseEstimator;
    protected SwerveDriveSubsystem swerveDrive;

    /* Autonomous */
    SendableChooser<Command> autonomousChooser;

    private final Timer reseedTimer = new Timer();

    /**
     * 
     */
    public RobotContainer() {
        /* Subsystems with no simulated modules */
        this.pneumaticSubsystem = new PneumaticSubsystem();
        this.intakeSubsystem = new IntakeSubsystem(new IntakeModule());
        this.shooterSubsystem = new ShooterSubsystem(new ShooterModule());
        
        this.driverController = new XBoxControlBindings();
        this.operatorController = new XBoxControlBindings();

        this.reseedTimer.start();
    }

    /**
     * 
     */
    protected void configureAutonomous() {
        AutoBuilder autoBuilder = new AutoBuilder(this.poseEstimator, this.swerveDrive, this.armSubsystem, this.intakeSubsystem, this.shooterSubsystem);
        autoBuilder.configureAutonomous();
        this.autonomousChooser = autoBuilder.getAutonomousChooser();
    }

    /**
     * 
     */
    protected void configureButtonBindings() {
        /* Driver Buttons */
        // POV up for field oriented drive
        this.driverController.driveTeleop().ifPresent(
            trigger -> trigger.onTrue(runOnce(() -> this.swerveDrive.setDefaultCommand(this.teleopDriveCommand))
                .andThen(new ScheduleCommand(this.teleopDriveCommand))));

        // reset the robot pose
        this.driverController.resetPose().ifPresent(trigger -> trigger.onTrue(runOnce(this::resetPose)));

        // Start button reseeds the steer motors to fix dead wheel
        this.driverController.reseedSteerMotors()
            .ifPresent(trigger -> trigger.onTrue(this.swerveDrive.runOnce(this.swerveDrive::reseedSteerMotorOffsets)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)));

        // 
        this.driverController.driveToSpeakerRight()
            .ifPresent(
                trigger -> trigger.onTrue(
                    new DriveFromBestTagCommand(
                        this.swerveDrive,
                        this.visionModule,
                        this.poseEstimator::getCurrentPose,
                        FieldConstants.SPEAKER_POSE_TRANSLATIONS[0],
                        FieldConstants.SPEAKER_POSE_ROTATIONS[0],
                        false)
                    .andThen(
                        // new DeployGamePieceMidCommand(this.armSubsystem)
                        // .until(this.driverController.driverWantsControl())
                    )
                    .until(this.driverController.driverWantsControl())
                )
            );

        // 
        this.driverController.driveToSpeakerCenter()
            .ifPresent(
                trigger -> trigger.onTrue(
                    new DriveFromBestTagCommand(
                        this.swerveDrive,
                        this.visionModule,
                        this.poseEstimator::getCurrentPose,
                        FieldConstants.SPEAKER_POSE_TRANSLATIONS[1],
                        FieldConstants.SPEAKER_POSE_ROTATIONS[1],
                        false)
                    .andThen(
                        // new DeployGamePieceMidCommand(this.armSubsystem)
                        // .until(this.driverController.driverWantsControl())
                    )
                    .until(this.driverController.driverWantsControl())
                )
            );

        // 
        this.driverController.driveToSpeakerLeft()
            .ifPresent(
                trigger -> trigger.onTrue(
                    new DriveFromBestTagCommand(
                        this.swerveDrive,
                        this.visionModule,
                        this.poseEstimator::getCurrentPose,
                        FieldConstants.SPEAKER_POSE_TRANSLATIONS[2],
                        FieldConstants.SPEAKER_POSE_ROTATIONS[2],
                        false)
                    .andThen(
                        // new DeployGamePieceMidCommand(this.armSubsystem)
                        // .until(this.driverController.driverWantsControl())
                    )
                    .until(this.driverController.driverWantsControl())
                )
            );

        // X-Stance Pose
        // this.driverController.xStance().ifPresent(trigger -> trigger.onTrue(
        //     run(this.swerveDrive::setXStance)
        //     .until(this.driverController.driverWantsControl())
        // ));

        /* Operator Buttons */
    }

    /**
     * 
     */
    public void disabledPeriodic() {
        // Reseed the motor offset continuously when the robot is disabled to help solve dead wheel issue
        if (this.reseedTimer.advanceIfElapsed(1.0)) {
            this.swerveDrive.reseedSteerMotorOffsets();
        }
    }
    /**
     * 
     */
    abstract public void disable();
    abstract public void enable();

    /**
     * 
     */
    public Command getAutonomousCommand() {
        return this.autonomousChooser.getSelected();
    }
        
    /**
     * Called when the alliance reported by the driverstation/FMS changes.
     * @param alliance new alliance value
     */
    public void onAllianceChanged(Alliance alliance, int location) {
        location -= 1;
        
        if (alliance.name().equalsIgnoreCase("Red")) {
            location = Math.abs(location - 2);
        }

        Pose2d pose2d = AllianceFlipUtil.apply(FieldConstants.SPEAKER_POSES[location]);
        this.poseEstimator.setCurrentPose(pose2d);
    }

    /**
     * 
     */
    public void resetPose() {
        this.poseEstimator.resetPose(new Pose2d(1, 1, new Rotation2d()));
    }

    /**
     * 
     */
    protected void setCommands() {
        this.teleopDriveCommand = new TeleopDriveCommand(
            this.swerveDrive,
            () -> this.poseEstimator.getCurrentPose().getRotation(),
            this.driverController.translationX(),
            this.driverController.translationY(),
            this.driverController.omega()
        );
        
        this.swerveDrive.setDefaultCommand(this.teleopDriveCommand);
    }
}
