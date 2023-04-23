package frc.robot.modules.vision;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.photonvision.SimVisionSystem;
import frc.robot.Constants.VisionConstants;

/**
 * 
 */
public class VisionModuleSimulator extends VisionModule {
    private SimVisionSystem frontSimVisionSystem;
    private SimVisionSystem rearSimVisionSystem;

    /**
     * 
     */
    public VisionModuleSimulator(AprilTagFields aprilTagFields) {
        super(aprilTagFields);
        
        this.frontSimVisionSystem = new SimVisionSystem(VisionConstants.FRONT_CAMERA_NAME, VisionConstants.DIAGONAL_FOV, VisionConstants.ROBOT_TO_FRONT_CAMERA, 9000, VisionConstants.IMG_WIDTH, VisionConstants.IMG_HEIGHT, 0);
        this.rearSimVisionSystem = new SimVisionSystem(VisionConstants.REAR_CAMERA_NAME, VisionConstants.DIAGONAL_FOV, VisionConstants.ROBOT_TO_REAR_CAMERA, 9000, VisionConstants.IMG_WIDTH, VisionConstants.IMG_HEIGHT, 0);
        
        setTargets();
    }
    
    @Override
    protected PhotonPipelineResult getFrontCameraResults() {
        return this.frontSimVisionSystem.cam.getLatestResult();
    }
    
    @Override
    protected PhotonPipelineResult getRearCameraResults() {
        return this.rearSimVisionSystem.cam.getLatestResult();
    }

    @Override
    protected void processFrame(Pose2d pose) {
        this.frontSimVisionSystem.processFrame(pose);
        this.rearSimVisionSystem.processFrame(pose);
    }

    /**
     * 
     */
    private void setTargets() {
        this.frontSimVisionSystem.addVisionTargets(this.aprilTagFieldLayout);
        this.rearSimVisionSystem.addVisionTargets(this.aprilTagFieldLayout);
    }
}
