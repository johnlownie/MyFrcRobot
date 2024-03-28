package frc.robot.containers;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Timer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.DriveModeType;
import frc.robot.autonomous.AutoBuilder;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.TuningCommand;
import frc.robot.controls.DriverBindings;
import frc.robot.controls.OperatorBindings;
import frc.robot.controls.TuningBindings;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.modules.gyro.GyroModule;
import frc.robot.modules.swerve.SwerveModuleTalonFX;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
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
    /* Commands */
    protected TeleopDriveCommand teleopDriveCommand;
    protected TuningCommand tuningCommand;

    /* Controllers */
    protected final XBoxControlBindings driverController;
    protected final XBoxControlBindings operatorController;

    /* Modules */
    protected GyroModule gyroModule;
    protected SwerveModuleTalonFX[] swerveModules;
    
    /* Subsystems */
    protected ArmSubsystem armSubsystem;
    protected IntakeSubsystem intakeSubsystem;
    protected LEDSubsystem ledSubsystem;
    protected PneumaticSubsystem pneumaticSubsystem;
    protected PoseEstimatorSubsystem poseEstimator;
    protected ShooterSubsystem shooterSubsystem;
    protected SwerveDriveSubsystem swerveDrive;
    protected VisionSubsystem visionSubsystem;

    /* Autonomous */
    LoggedDashboardChooser<Command> autonomousChooser;

    /* Variables */
    DriverBindings driverBindings;
    OperatorBindings operatorBindings;
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
        AutoBuilder autoBuilder = new AutoBuilder(this.poseEstimator, this.swerveDrive, this.armSubsystem, this.intakeSubsystem, this.shooterSubsystem, this.visionSubsystem);
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
        }
        else {
            this.driverBindings = new DriverBindings(this.swerveDrive, this.poseEstimator, this.armSubsystem, this.intakeSubsystem, this.shooterSubsystem, this.visionSubsystem, this.teleopDriveCommand);
            this.driverBindings.configureButtonBindings(this.driverController);

            this.operatorBindings = new OperatorBindings(this.swerveDrive, this.poseEstimator, this.armSubsystem, this.intakeSubsystem, this.shooterSubsystem, this.visionSubsystem);
            this.operatorBindings.configureButtonBindings(this.operatorController);
        }
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
     * 
     */
    public String getDriveModeTypeAsString() {
        if (this.driverBindings == null) return "";

        return this.driverBindings.getDriveModeType().get() == DriveModeType.SPEAKER ? "SPEAKER" : "AMP";
    }
        
    /**
     * Called when the alliance reported by the driverstation/FMS changes.
     * @param alliance new alliance value
     */
    public void onAllianceChanged(Alliance alliance, int location) {
        location -= 1;

        Pose2d pose2d = AllianceFlipUtil.apply(RobotConstants.TUNING_MODE ? FieldConstants.TUNING_POSES[location] : FieldConstants.SPEAKER_POSES[location]);
        this.poseEstimator.setCurrentPose(pose2d);
    }

    /**
     * 
     */
    public void resetPose() {
        this.poseEstimator.resetPose(new Pose2d(0, 0, new Rotation2d()));
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
