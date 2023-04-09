package frc.robot.containers;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.lib.util.Timer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.autonomous.AutonomousBuilder;
import frc.robot.commands.DeployGamePieceMidCommand;
import frc.robot.commands.DriveFromPoseCommand;
import frc.robot.commands.DriveToBestTagCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.modules.drawer.DrawerModule;
import frc.robot.modules.gyro.GyroModule;
import frc.robot.modules.swerve.SwerveModule;
import frc.robot.modules.vision.VisionModule;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrawerSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utils.AllianceFlipUtil;

abstract public class RobotContainer {
   /* Autonomous */
    protected AutonomousBuilder autonomousBuilder;

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
    protected DrawerSubsystem drawerSubsystem;
    protected PneumaticSubsystem pneumaticSubsystem;
    protected PoseEstimatorSubsystem poseEstimator;
    protected SwerveDriveSubsystem swerveDrive;

    private final Timer reseedTimer = new Timer();

    /**
     * 
     */
    public RobotContainer() {
        /* Subsystems with no simulated modules */
        this.pneumaticSubsystem = new PneumaticSubsystem();
        this.drawerSubsystem = new DrawerSubsystem(new DrawerModule(this.pneumaticSubsystem));
        
        this.driverController = new XBoxControlBindings();
        this.operatorController = new XBoxControlBindings();

        this.reseedTimer.start();
    }

    /**
     * 
     */
    protected void configureButtonBindings() {
        /* Driver Buttons */
        // switch from robot relative to field relative
        this.driverController.driveType().ifPresent(trigger -> trigger.toggleOnTrue(
            either(
                runOnce(this.swerveDrive::disableFieldRelative, this.swerveDrive),
                runOnce(this.swerveDrive::enableFieldRelative, this.swerveDrive),
                this.swerveDrive::isFieldRelative)
        ));

        // POV up for field oriented drive
        this.driverController.driveTeleop().ifPresent(
            trigger -> trigger.onTrue(runOnce(() -> this.swerveDrive.setDefaultCommand(this.teleopDriveCommand))
                .andThen(new ScheduleCommand(this.teleopDriveCommand))));

        // drive to tag and strafe
        this.driverController.driveToPoleLeft().ifPresent(
            trigger -> trigger.onTrue(
                new DriveToBestTagCommand(this.swerveDrive, this.visionModule.getCamera(), this.poseEstimator::getCurrentPose)
                .andThen(
                    new DriveFromPoseCommand(this.swerveDrive, this.poseEstimator::getCurrentPose, 0.0, -FieldConstants.STRAFE_DISTANCE, 0.0)
                    .andThen(
                        new DeployGamePieceMidCommand(this.armSubsystem)
                    )
                    .until(this.driverController.driverWantsControl())
                )
                .until(this.driverController.driverWantsControl())
            )
        );

        // drive to tag and strafe
        this.driverController.driveToPoleRight().ifPresent(
            trigger -> trigger.onTrue(
                new DriveToBestTagCommand(this.swerveDrive, this.visionModule.getCamera(), this.poseEstimator::getCurrentPose)
                .andThen(
                    new DriveFromPoseCommand(this.swerveDrive, this.poseEstimator::getCurrentPose, 0.0, FieldConstants.STRAFE_DISTANCE, 0.0)
                    .andThen(
                        new DeployGamePieceMidCommand(this.armSubsystem)
                    )
                    .until(this.driverController.driverWantsControl())
                )
                .until(this.driverController.driverWantsControl())
            )
        );

        // reset the robot pose
        this.driverController.resetPose().ifPresent(trigger -> trigger.onTrue(runOnce(this.poseEstimator::resetFieldPosition)));

        // Start button reseeds the steer motors to fix dead wheel
        this.driverController.reseedSteerMotors()
            .ifPresent(trigger -> trigger.onTrue(this.swerveDrive.runOnce(this.swerveDrive::reseedSteerMotorOffsets)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)));

        // X-Stance Pose
        this.driverController.xStance().ifPresent(trigger -> trigger.onTrue(
            run(this.swerveDrive::setXStance)
            .until(this.driverController.driverWantsControl())
        ));

        /* Operator Buttons */
        this.operatorController.closeGripper()
            .ifPresent(trigger -> trigger.onTrue(
                runOnce(this.armSubsystem::closeGripper)
            )
        );

        this.operatorController.openGripper()
            .ifPresent(trigger -> trigger.onTrue(
                runOnce(this.armSubsystem::openGripper)
            )
        );

        this.operatorController.extendDrawer()
            .ifPresent(trigger -> trigger.onTrue(
                runOnce(this.drawerSubsystem::extend)
            )
        );

        this.operatorController.retractDrawer()
            .ifPresent(trigger -> trigger.onTrue(
                runOnce(this.drawerSubsystem::retract)
            )
        );
    }

    /**
     * 
     */
    protected void configureDashboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Driver");
        this.autonomousBuilder.addDashboardWidgets(tab);
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
        return this.autonomousBuilder.getAutonomousCommand();
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
