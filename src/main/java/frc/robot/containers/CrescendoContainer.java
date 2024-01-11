package frc.robot.containers;

import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.modules.gyro.GyroModuleNavx;
import frc.robot.modules.swerve.SwerveModuleTalonFX;
import frc.robot.modules.vision.VisionModulePhotonVision;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class CrescendoContainer extends RobotContainer {
    /**
     * 
     */
    public CrescendoContainer() {
        super();

        SwerveModuleTalonFX frontLeftModule  = new SwerveModuleTalonFX(0, 11, 12, 1, 183.603);
        SwerveModuleTalonFX frontRightModule = new SwerveModuleTalonFX(1, 31, 32, 3, 235.107);
        SwerveModuleTalonFX rearLeftModule   = new SwerveModuleTalonFX(2, 21, 22, 2, 299.001);
        SwerveModuleTalonFX rearRightModule  = new SwerveModuleTalonFX(3, 41, 42, 4, 231.152);

        this.swerveModules = new SwerveModuleTalonFX[] { frontLeftModule, frontRightModule, rearLeftModule, rearRightModule};
        this.gyroModule = new GyroModuleNavx();
        this.visionModule = new VisionModulePhotonVision(AprilTagFields.k2024Crescendo);
        
        this.swerveDrive = new SwerveDriveSubsystem(this.swerveModules, DriveTrainConstants.SWERVE_DRIVE_KINEMATICS, this.gyroModule);
        this.poseEstimator = new PoseEstimatorSubsystem(this.swerveDrive::getModulePositions, this.swerveDrive::getRotation, this.visionModule);

        setCommands();

        configureButtonBindings();
        configureDashboard();
    }

    @Override
    public void disable() {

    }

    @Override
    public void enable() {
        
    }
}
