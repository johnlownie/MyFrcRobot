package frc.robot.containers;

import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.modules.gyro.GyroModuleSimulator;
import frc.robot.modules.swerve.SwerveModuleSimulator;
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
        
        SwerveModuleSimulator frontLeftModule = new SwerveModuleSimulator(0);
        SwerveModuleSimulator frontRightModule = new SwerveModuleSimulator(1);
        SwerveModuleSimulator rearLeftModule = new SwerveModuleSimulator(2);
        SwerveModuleSimulator rearRightModule = new SwerveModuleSimulator(3);
        
        this.swerveModules = new SwerveModuleSimulator[] { frontLeftModule, frontRightModule, rearLeftModule, rearRightModule};
        this.gyroModule = new GyroModuleSimulator();
        this.visionModule = new VisionModuleSimulator(AprilTagFields.k2024Crescendo);
        
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
