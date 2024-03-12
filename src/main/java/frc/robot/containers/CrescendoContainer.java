package frc.robot.containers;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.SwerveModuleConstants.Mod0;
import frc.robot.Constants.SwerveModuleConstants.Mod1;
import frc.robot.Constants.SwerveModuleConstants.Mod2;
import frc.robot.Constants.SwerveModuleConstants.Mod3;
import frc.robot.modules.gyro.GyroModuleNavx;
import frc.robot.modules.gyro.GyroModuleSimulator;
import frc.robot.modules.intake.IntakeModule;
import frc.robot.modules.shooter.ShooterModule;
import frc.robot.modules.swerve.SwerveModule;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * 
 */
public class CrescendoContainer extends RobotContainer {
    /**
     * 
     */
    public CrescendoContainer() {
        super();

        SwerveModule frontLeftModule  = new SwerveModule(0, Mod0.DRIVE_MOTOR_ID, Mod0.ANGLE_MOTOR_ID, Mod0.CANCODER_ID, Mod0.ANGLE_OFFSET);
        SwerveModule frontRightModule = new SwerveModule(1, Mod1.DRIVE_MOTOR_ID, Mod1.ANGLE_MOTOR_ID, Mod1.CANCODER_ID, Mod1.ANGLE_OFFSET);
        SwerveModule rearLeftModule   = new SwerveModule(2, Mod2.DRIVE_MOTOR_ID, Mod2.ANGLE_MOTOR_ID, Mod2.CANCODER_ID, Mod2.ANGLE_OFFSET);
        SwerveModule rearRightModule  = new SwerveModule(3, Mod3.DRIVE_MOTOR_ID, Mod3.ANGLE_MOTOR_ID, Mod3.CANCODER_ID, Mod3.ANGLE_OFFSET);

        this.swerveModules = new SwerveModule[] { frontLeftModule, frontRightModule, rearLeftModule, rearRightModule};
        try {
            
            this.gyroModule = new GyroModuleNavx();

        } catch (RuntimeException e) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + e.getMessage(), true);
            this.gyroModule = new GyroModuleSimulator();
        }
        
        this.swerveDrive = new SwerveDriveSubsystem(this.swerveModules, this.gyroModule);
        this.intakeSubsystem = new IntakeSubsystem(new IntakeModule());
        this.shooterSubsystem = new ShooterSubsystem(new ShooterModule());
        this.visionSubsystem = new VisionSubsystem(VisionConstants.CAMERAS);
        this.poseEstimator = new PoseEstimatorSubsystem(this.swerveDrive::getModulePositions, this.swerveDrive::getRotation, this.visionSubsystem);

        setCommands();

        configureAutonomous();
        configureButtonBindings();
    }

    @Override
    public void disable() {

    }

    @Override
    public void enable() {
        
    }
}
