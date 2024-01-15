package frc.robot.modules.vision;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    public VisionModuleSimulator(AprilTagFields aprilTagFields) {
        super(aprilTagFields);
        
        SimCameraProperties simCameraProperties = new SimCameraProperties();
        simCameraProperties.setCalibration(VisionConstants.IMG_WIDTH, VisionConstants.IMG_HEIGHT, Rotation2d.fromDegrees(VisionConstants.DIAGONAL_FOV));
        simCameraProperties.setCalibError(0.35, 0.10);
        simCameraProperties.setFPS(15);
        simCameraProperties.setAvgLatencyMs(50);
        simCameraProperties.setLatencyStdDevMs(15);
        
        this.frontCameraSim = new PhotonCameraSim(this.frontCamera, simCameraProperties);
        this.frontCameraSim.enableDrawWireframe(true);
        this.frontVisionSystemSim = new VisionSystemSim(VisionConstants.FRONT_CAMERA_NAME);
        this.frontVisionSystemSim.addAprilTags(aprilTagFieldLayout);
        this.frontVisionSystemSim.addCamera(this.frontCameraSim, VisionConstants.ROBOT_TO_FRONT_CAMERA);

        this.rearCameraSim = new PhotonCameraSim(this.rearCamera, simCameraProperties);
        this.rearCameraSim.enableDrawWireframe(true);
        this.rearVisionSystemSim = new VisionSystemSim(VisionConstants.REAR_CAMERA_NAME);
        this.rearVisionSystemSim.addAprilTags(aprilTagFieldLayout);
        this.rearVisionSystemSim.addCamera(this.rearCameraSim, VisionConstants.ROBOT_TO_REAR_CAMERA);
    }

    @Override
    protected void processFrame(Pose2d pose) {
        this.frontVisionSystemSim.update(pose);
        this.rearVisionSystemSim.update(pose);
    }
}
