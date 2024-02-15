package frc.robot.modules.vision;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

/**
 * 
 */
public class VisionModuleSimulator extends VisionModule {
    private PhotonCameraSim frontCameraSim;
    private PhotonCameraSim rearCameraSim;
    private VisionSystemSim frontVisionSystemSim;
    private VisionSystemSim rearVisionSystemSim;

    /**
     * 
     */
    public VisionModuleSimulator() {
        super();
        
        SimCameraProperties simCameraProperties = new SimCameraProperties();
        simCameraProperties.setCalibration(VisionConstants.IMG_WIDTH, VisionConstants.IMG_HEIGHT, Rotation2d.fromDegrees(VisionConstants.DIAGONAL_FOV));
        simCameraProperties.setCalibError(0.35, 0.10);
        simCameraProperties.setFPS(15);
        simCameraProperties.setAvgLatencyMs(50);
        simCameraProperties.setLatencyStdDevMs(15);
        
        this.frontCameraSim = new PhotonCameraSim(this.frontCamera, simCameraProperties);

        this.frontVisionSystemSim = new VisionSystemSim(VisionConstants.FRONT_CAMERA_NAME);
        this.frontVisionSystemSim.addAprilTags(FieldConstants.TAG_FIELD_LAYOUT);
        this.frontVisionSystemSim.addCamera(this.frontCameraSim, VisionConstants.ROBOT_TO_FRONT_CAMERA);

        this.rearCameraSim = new PhotonCameraSim(this.rearCamera, simCameraProperties);

        this.rearVisionSystemSim = new VisionSystemSim(VisionConstants.REAR_CAMERA_NAME);
        this.rearVisionSystemSim.addAprilTags(FieldConstants.TAG_FIELD_LAYOUT);
        this.rearVisionSystemSim.addCamera(this.rearCameraSim, VisionConstants.ROBOT_TO_REAR_CAMERA);
    }

    @Override
    protected void processFrame(Pose2d pose2d) {
        this.frontVisionSystemSim.update(pose2d);
        this.rearVisionSystemSim.update(pose2d);
    }
    
    @Override
    public void resetFieldPosition(Pose2d pose2d) {
        this.frontVisionSystemSim.resetRobotPose(pose2d);
        this.rearVisionSystemSim.resetRobotPose(pose2d);
    }
}
