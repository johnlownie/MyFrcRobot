package frc.robot.containers;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.SwerveModuleConstants.Mod0;
import frc.robot.Constants.SwerveModuleConstants.Mod1;
import frc.robot.Constants.SwerveModuleConstants.Mod2;
import frc.robot.Constants.SwerveModuleConstants.Mod3;
import frc.robot.modules.arm.ArmModuleSimulator;
import frc.robot.modules.gyro.GyroModuleSimulator;
import frc.robot.modules.swerve.SwerveModuleSimulator;
import frc.robot.modules.vision.VisionModuleSimulator;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

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
        
        this.swerveDrive = new SwerveDriveSubsystem(this.swerveModules, DriveTrainConstants.SWERVE_DRIVE_KINEMATICS, this.gyroModule);
        this.visionSubsystem = new VisionSubsystem(new VisionModuleSimulator());
        this.poseEstimator = new PoseEstimatorSubsystem(this.swerveDrive::getModulePositions, this.swerveDrive::getRotation, this.visionSubsystem);
        this.armSubsystem = new ArmSubsystem(new ArmModuleSimulator());

        setCommands();

        configureAutonomous();
        configureButtonBindings();
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
