package frc.robot.containers;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.lib.util.Timer;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.commands.DriveFromBestTagCommand;
import frc.robot.commands.DriveFromPoseCommand;
import frc.robot.commands.DriveToBestTagCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.modules.gyro.GyroModule;
import frc.robot.modules.swerve.SwerveModule;
import frc.robot.modules.vision.VisionModule;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
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
        
        this.driverController = new XBoxControlBindings();
        this.operatorController = new XBoxControlBindings();

        this.reseedTimer.start();
    }

    /**
     * 
     */
    protected void configureAutonomous() {
        AutoBuilder.configureHolonomic(
            this.poseEstimator::getCurrentPose,
            this.poseEstimator::resetPose,
            this.swerveDrive::getChassisSpeeds,
            this.swerveDrive::drive,
            new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0),
                SwerveModuleConstants.MAX_VELOCITY_METERS_PER_SECOND,
                DriveTrainConstants.TRACK_WIDTH_METERS,
                new ReplanningConfig()
            ), 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }

                return false;
            },
            this.swerveDrive
        );

        this.autonomousChooser = AutoBuilder.buildAutoChooser();

        ShuffleboardTab tab = Shuffleboard.getTab("Driver");
        tab.add("Autonomous", this.autonomousChooser).withSize(2, 1).withPosition(0, 0);
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

        Pose2d pose2d = AllianceFlipUtil.apply(FieldConstants.ALLIANCE_POSES[location]);
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
