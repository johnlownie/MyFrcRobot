package frc.robot.containers;

import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.modules.arm.ArmModuleTalonSRX;
import frc.robot.modules.gyro.GyroModuleNavx;
import frc.robot.modules.swerve.SwerveModuleTalonFX;
import frc.robot.modules.vision.VisionModulePhotonVision;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class ChargedUpContainer extends RobotContainer {
    /**
     * 
     */
    public ChargedUpContainer() {
        super();

        SwerveModuleTalonFX frontLeftModule = new SwerveModuleTalonFX(0, 0, 0, 0, 0);
        SwerveModuleTalonFX frontRightModule = new SwerveModuleTalonFX(0, 0, 0, 0, 0);
        SwerveModuleTalonFX rearLeftModule = new SwerveModuleTalonFX(0, 0, 0, 0, 0);
        SwerveModuleTalonFX rearRightModule = new SwerveModuleTalonFX(0, 0, 0, 0, 0);

        this.swerveModules = new SwerveModuleTalonFX[] { frontLeftModule, frontRightModule, rearLeftModule, rearRightModule};
        this.gyroModule = new GyroModuleNavx();
        this.visionModule = new VisionModulePhotonVision(AprilTagFields.k2023ChargedUp);
        
        this.swerveDrive = new SwerveDriveSubsystem(this.swerveModules, DriveTrainConstants.SWERVE_DRIVE_KINEMATICS, this.gyroModule);
        this.poseEstimator = new PoseEstimatorSubsystem(this.swerveDrive::getModulePositions, this.swerveDrive::getRotation, this.visionModule);
        this.armSubsystem = new ArmSubsystem(new ArmModuleTalonSRX(this.pneumaticSubsystem));

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
