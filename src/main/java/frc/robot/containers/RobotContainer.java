package frc.robot.containers;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.FieldConstants;
import frc.robot.autonomous.AutonomousBuilder;
import frc.robot.autonomous.TwoPieceBalance;
import frc.robot.commands.DeployGamePieceMidCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.DriveToTagCommand;
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
    private DriveToTagCommand driveToTagLeftCommand;
    private DriveToTagCommand driveToTagRightCommand;
    
    private DriveToPoseCommand driveToPoleLeftCommand;
    private DriveToPoseCommand driveToPoleRightCommand;

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

    /**
     * 
     */
    public RobotContainer() {
        /* Subsystems with no simulated modules */
        this.pneumaticSubsystem = new PneumaticSubsystem();
        this.drawerSubsystem = new DrawerSubsystem(new DrawerModule(this.pneumaticSubsystem));
        
        this.driverController = new XBoxControlBindings();
        this.operatorController = new XBoxControlBindings();
    }

    /**
     * 
     */
    protected void configureButtonBindings() {
        /* Driver Buttons */
        // switch from field relative to field oriented
        this.driverController.driveType().ifPresent(trigger -> trigger.toggleOnTrue(
            either(
                runOnce(this.swerveDrive::disableFieldOriented, this.swerveDrive),
                runOnce(this.swerveDrive::enableFieldOriented, this.swerveDrive),
                this.swerveDrive::isFieldOriented)
        ));

        // POV up for field oriented drive
        this.driverController.driveTeleop().ifPresent(
            trigger -> trigger.onTrue(runOnce(() -> this.swerveDrive.setDefaultCommand(this.teleopDriveCommand))
                .andThen(new ScheduleCommand(this.teleopDriveCommand))));

        // drive to tag and strafe
        this.driverController.driveToPoleLeft().ifPresent(
            trigger -> trigger.onTrue(
                this.driveToTagLeftCommand
                    .andThen(
                        this.driveToPoleLeftCommand
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
                this.driveToTagRightCommand
                    .andThen(
                        this.driveToPoleRightCommand
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
    abstract public void disable();
    abstract public void enable();

    /**
     * 
     */
    public Command getAutonomousCommand() {
        // return this.autonomousBuilder.getAutonomousCommand();
        return new TwoPieceBalance(this.swerveDrive, this.poseEstimator, this.armSubsystem);
    }
        
    /**
     * Called when the alliance reported by the driverstation/FMS changes.
     * @param alliance new alliance value
     */
    public void onAllianceChanged(Alliance alliance, int location) {
        Pose2d pose2d = AllianceFlipUtil.apply(FieldConstants.ALLIANCE_POSES[location - 1]);
        // int allianceStation = alliance.name().equalsIgnoreCase("blue") ? 0 : 1;
        // this.poseEstimator.setAlliance(alliance, this.alliancePoses[allianceStation][location - 1]);
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
        
        this.driveToTagLeftCommand = new DriveToTagCommand(this.swerveDrive, this.visionModule.getCamera(), this.poseEstimator::getCurrentPose);
        this.driveToPoleLeftCommand = new DriveToPoseCommand(this.swerveDrive, this.poseEstimator::getCurrentPose, 0.0, -FieldConstants.STRAFE_DISTANCE, 0.0);

        this.driveToTagRightCommand = new DriveToTagCommand(this.swerveDrive, this.visionModule.getCamera(), this.poseEstimator::getCurrentPose);
        this.driveToPoleRightCommand = new DriveToPoseCommand(this.swerveDrive, this.poseEstimator::getCurrentPose, 0.0, FieldConstants.STRAFE_DISTANCE, 0.0);
    }
}
