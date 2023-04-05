package frc.robot.modules.vision;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.photonvision.SimVisionSystem;
import frc.robot.Constants.VisionConstants;

public class VisionModuleSimulator extends VisionModule {
    private SimVisionSystem simVisionSystem;

    /**
     * 
     */
    public VisionModuleSimulator(AprilTagFields aprilTagFields) {
        super(aprilTagFields);
        
        this.simVisionSystem = new SimVisionSystem(VisionConstants.REAR_CAMERA_NAME, VisionConstants.DIAGONAL_FOV, VisionConstants.ROBOT_TO_REAR_CAMERA, 9000, VisionConstants.IMG_WIDTH, VisionConstants.IMG_HEIGHT, 0);
        setTargets();
    }
    
    @Override
    protected PhotonPipelineResult getResults() {
        return this.simVisionSystem.cam.getLatestResult();
    }

    @Override
    protected void processFrame(Pose2d pose) {
        this.simVisionSystem.processFrame(pose);
    }

    /**
     * 
     */
    private void setTargets() {
        this.simVisionSystem.addVisionTargets(this.aprilTagFieldLayout);
    }
}
