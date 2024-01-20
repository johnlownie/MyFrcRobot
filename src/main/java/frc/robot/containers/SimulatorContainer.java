package frc.robot.containers;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.SwerveModuleConstants.Mod0;
import frc.robot.Constants.SwerveModuleConstants.Mod1;
import frc.robot.Constants.SwerveModuleConstants.Mod2;
import frc.robot.Constants.SwerveModuleConstants.Mod3;
import frc.robot.modules.gyro.GyroModuleSimulator;
import frc.robot.modules.swerve.SwerveModule;
import frc.robot.modules.swerve.SwerveModuleSimulator;
import frc.robot.modules.vision.VisionModule;
import frc.robot.modules.vision.VisionModuleSimulator;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class SimulatorContainer extends RobotContainer {
    /**
     * 
     */
    public SimulatorContainer() {
        super();

        SwerveModuleSimulator frontLeftModule  = new SwerveModuleSimulator(0, Mod0.DRIVE_MOTOR_ID, Mod0.ANGLE_MOTOR_ID, Mod0.CANCODER_ID, Mod0.ANGLE_OFFSET);
        SwerveModuleSimulator frontRightModule = new SwerveModuleSimulator(1, Mod1.DRIVE_MOTOR_ID, Mod1.ANGLE_MOTOR_ID, Mod1.CANCODER_ID, Mod1.ANGLE_OFFSET);
        SwerveModuleSimulator rearLeftModule   = new SwerveModuleSimulator(2, Mod2.DRIVE_MOTOR_ID, Mod2.ANGLE_MOTOR_ID, Mod2.CANCODER_ID, Mod2.ANGLE_OFFSET);
        SwerveModuleSimulator rearRightModule  = new SwerveModuleSimulator(3, Mod3.DRIVE_MOTOR_ID, Mod3.ANGLE_MOTOR_ID, Mod3.CANCODER_ID, Mod3.ANGLE_OFFSET);
        
        this.swerveModules = new SwerveModuleSimulator[] { frontLeftModule, frontRightModule, rearLeftModule, rearRightModule};
        this.gyroModule = new GyroModuleSimulator();
        this.visionModule = new VisionModuleSimulator();
        // this.visionModule = new VisionModule();
        
        this.swerveDrive = new SwerveDriveSubsystem(this.swerveModules, DriveTrainConstants.SWERVE_DRIVE_KINEMATICS, this.gyroModule);
        this.poseEstimator = new PoseEstimatorSubsystem(this.swerveDrive::getModulePositions, this.swerveDrive::getRotation, this.visionModule);

        setCommands();

        configureButtonBindings();
        configureDashboard();
    }

    /**
     * 
     */
    public void disable() {
    }

    /**
     * 
     */
    public void enable() {
    }
}
