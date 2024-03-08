package frc.robot.modules.vision;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.note.NoteLoader;
import frc.lib.camera.Camera;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

/**
 * 
 */
public class VisionModuleSimulator extends VisionModule {
    private final PhotonCameraSim photonCameraSim;
    private final VisionSystemSim visionSystemSim;

    protected Supplier<Pose2d> poseSupplier;

    /**
     * 
     */
    public VisionModuleSimulator(Camera camera) {
        super(camera);
        
        SimCameraProperties simCameraProperties = new SimCameraProperties();
        simCameraProperties.setCalibration(VisionConstants.IMG_WIDTH, VisionConstants.IMG_HEIGHT, Rotation2d.fromDegrees(VisionConstants.DIAGONAL_FOV));
        simCameraProperties.setCalibError(0.35, 0.10);
        simCameraProperties.setFPS(15);
        simCameraProperties.setAvgLatencyMs(50);
        simCameraProperties.setLatencyStdDevMs(15);
        
        this.photonCameraSim = new PhotonCameraSim(this.photonCamera, simCameraProperties);

        this.visionSystemSim = new VisionSystemSim(this.camera.getName());
        this.visionSystemSim.addCamera(this.photonCameraSim, this.camera.getRobotToCamera());

        if (this.camera.getType() == Camera.Type.APRILTAG) {
            this.visionSystemSim.addAprilTags(FieldConstants.TAG_FIELD_LAYOUT);

        }
        else if (this.camera.getType() == Camera.Type.COLOURED_SHAPE) {
            var visionTargetSims = NoteLoader.getVisionTargets();
            for (VisionTargetSim visionTargetSim : visionTargetSims) {
                this.visionSystemSim.addVisionTargets("note", visionTargetSim);
            }
        }

        this.poseSupplier = null;
    }

    @Override
    public void process() {
        if (this.camera == null || this.photonPoseEstimator == null ||  this.poseSupplier == null) {
            return;
        }

        processFrame(this.poseSupplier.get());
        
        processResults();
    }

    /**
     * 
     */
    protected void processFrame(Pose2d pose2d) {
        this.visionSystemSim.update(pose2d);
    }
    
    @Override
    public void resetFieldPosition(Pose2d pose2d) {
        this.visionSystemSim.resetRobotPose(pose2d);
    }

    /**
     * Getters and Setters
     */
    public void setPoseSupplier(Supplier<Pose2d> poseSupplier) { this.poseSupplier = poseSupplier; }
}
