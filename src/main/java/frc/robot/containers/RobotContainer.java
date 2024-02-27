package frc.robot.containers;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Timer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.autonomous.AutoBuilder;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.TuningCommand;
import frc.robot.controls.DriverBindings;
import frc.robot.controls.OperatorBindings;
import frc.robot.controls.TuningBindings;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.modules.gyro.GyroModule;
import frc.robot.modules.intake.IntakeModule;
import frc.robot.modules.shooter.ShooterModule;
import frc.robot.modules.swerve.SwerveModule;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.AllianceFlipUtil;

/**
 * 
 */
abstract public class RobotContainer {
   /* Autonomous */

    /* Commands */
    protected TeleopDriveCommand teleopDriveCommand;
    protected TuningCommand tuningCommand;

    /* Controllers */
    protected final XBoxControlBindings driverController;
    protected final XBoxControlBindings operatorController;

    /* Modules */
    protected GyroModule gyroModule;
    protected SwerveModule[] swerveModules;
    
    /* Subsystems */
    protected ArmSubsystem armSubsystem;
    protected IntakeSubsystem intakeSubsystem;
    protected ShooterSubsystem shooterSubsystem;
    protected PneumaticSubsystem pneumaticSubsystem;
    protected PoseEstimatorSubsystem poseEstimator;
    protected SwerveDriveSubsystem swerveDrive;
    protected VisionSubsystem visionSubsystem;

    /* Autonomous */
    LoggedDashboardChooser<Command> autonomousChooser;

    private final Timer reseedTimer = new Timer();

    /**
     * 
     */
    public RobotContainer() {
        /* Subsystems with no simulated modules */
        this.pneumaticSubsystem = new PneumaticSubsystem();
        this.intakeSubsystem = new IntakeSubsystem(new IntakeModule());
        
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
        /* Use specific button configuration if tuning the robot */
        if (RobotConstants.TUNING_MODE) {
            TuningBindings tuningBindings = new TuningBindings(this.swerveDrive, this.poseEstimator, this.armSubsystem, this.intakeSubsystem, this.shooterSubsystem, this.visionSubsystem, this.teleopDriveCommand);
            tuningBindings.configureDriverButtonBindings(this.driverController);
            tuningBindings.configureOperatorButtonBindings(this.operatorController);
            return;
        }

        /* Driver Buttons */
        DriverBindings driverBindings = new DriverBindings(this.swerveDrive, this.poseEstimator, this.armSubsystem, this.intakeSubsystem, this.shooterSubsystem, this.visionSubsystem, this.teleopDriveCommand);
        driverBindings.configureButtonBindings(this.driverController);

        /* Operator Buttons */
        OperatorBindings operatorBindings = new OperatorBindings(this.swerveDrive, this.poseEstimator, this.armSubsystem, this.intakeSubsystem, this.shooterSubsystem, this.visionSubsystem);
        operatorBindings.configureButtonBindings(this.operatorController);
    }

    /**
     * 
     */
    public void disabledPeriodic() {
        // Reseed the motor offset continuously when the robot is disabled to help solve dead wheel issue
        if (this.reseedTimer.advanceIfElapsed(1.0)) {
            // this.swerveDrive.reseedSteerMotorOffsets();
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
        return this.autonomousChooser.get();
    }
        
    /**
     * Called when the alliance reported by the driverstation/FMS changes.
     * @param alliance new alliance value
     */
    public void onAllianceChanged(Alliance alliance, int location) {
        location -= 1;

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

        this.tuningCommand = new TuningCommand(
            this.swerveDrive,
            this.armSubsystem,
            this.poseEstimator,
            this.driverController
        );

        this.swerveDrive.setDefaultCommand(!RobotConstants.TUNING_MODE ? this.teleopDriveCommand : this.tuningCommand);
    }
}
