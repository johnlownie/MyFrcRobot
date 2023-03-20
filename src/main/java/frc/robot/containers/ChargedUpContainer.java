package frc.robot.containers;

import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.modules.gyro.GyroModuleNavx;
import frc.robot.modules.swerve.SwerveModuleTalonFX;
import frc.robot.modules.vision.VisionModulePhotonVision;

/**
 * 
 */
public class ChargedUpContainer extends RobotContainer {
    /* Modules */
    private final SwerveModuleTalonFX[] swerveModules;
    private final GyroModuleNavx gyro;
    private final VisionModulePhotonVision visionModule;

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
        this.gyro = new GyroModuleNavx();
        this.visionModule = new VisionModulePhotonVision(AprilTagFields.k2023ChargedUp);

        setCommands();

        configureButtonBindings();
        configureDashboard();
    }
}
