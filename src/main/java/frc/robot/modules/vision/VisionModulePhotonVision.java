package frc.robot.modules.vision;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

public class VisionModulePhotonVision extends VisionModule {
    /**
     * 
     */
    public VisionModulePhotonVision(AprilTagFields aprilTagFields) {
        super(aprilTagFields);
    }
    
    @Override
    protected PhotonPipelineResult getResults() {
        return this.photonCamera.getLatestResult();
    }

    @Override
    protected void processFrame(Pose2d pose) {
        // Nothing to do here as this is for simulated vision
    }
}
