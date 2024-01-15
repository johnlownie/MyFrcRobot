package frc.robot.modules.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * 
 */
public class VisionModulePhotonVision extends VisionModule {
    /**
     * 
     */
    public VisionModulePhotonVision(AprilTagFields aprilTagFields) {
        super(aprilTagFields);
    }

    @Override
    protected void processFrame(Pose2d pose) {
        // Nothing to do here as this is for simulated vision
    }
}
