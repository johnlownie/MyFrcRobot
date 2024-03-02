package frc.robot.modules.vision;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.note.NoteLoader;
import frc.lib.util.Timer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

/**
 * 
 */
public class VisionModuleSimulator extends VisionModule {
    private final PhotonCameraSim frontCameraSim;
    private final PhotonCameraSim rearCameraSim;
    private final PhotonCameraSim noteCameraSim;
    private final VisionSystemSim frontVisionSystemSim;
    private final VisionSystemSim rearVisionSystemSim;
    private final VisionSystemSim noteVisionSystemSim;

    protected Supplier<Pose2d> poseSupplier;

    // Simulating auto pickup of note
    private double simYaw;
    private Timer timer = new Timer();
    private boolean isFirstCall = true;

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

        this.noteCameraSim = new PhotonCameraSim(this.noteCamera, simCameraProperties);

        this.noteVisionSystemSim = new VisionSystemSim(VisionConstants.NOTE_CAMERA_NAME);
        var visionTargetSims = NoteLoader.getVisionTargets();
        for (VisionTargetSim visionTargetSim : visionTargetSims) {
            this.noteVisionSystemSim.addVisionTargets("note", visionTargetSim);
        }
        this.noteVisionSystemSim.addCamera(this.noteCameraSim, VisionConstants.ROBOT_TO_NOTE_CAMERA);

        this.poseSupplier = null;
    }

    @Override
    public void process() {
        if (this.frontCameraPhotonPoseEstimator == null || this.rearCameraPhotonPoseEstimator == null || this.poseSupplier == null || this.rearCamera == null) {
            return;
        }

        processFrame(this.poseSupplier.get());
        
        processResults(VisionConstants.FRONT_CAMERA_NAME, getFrontCameraResults(), this.frontCameraPhotonPoseEstimator, this.atomicFrontEstimatedRobotPose);
        processResults(VisionConstants.REAR_CAMERA_NAME, getRearCameraResults(), this.rearCameraPhotonPoseEstimator, this.atomicRearEstimatedRobotPose);
        processResults(VisionConstants.NOTE_CAMERA_NAME, getNoteCameraResults(), this.noteCameraPhotonPoseEstimator, this.atomicNoteEstimatedRobotPose);
    }

    /**
     * 
     */
    protected void processFrame(Pose2d pose2d) {
        this.frontVisionSystemSim.update(pose2d);
        this.rearVisionSystemSim.update(pose2d);
        this.noteVisionSystemSim.update(pose2d);
    }
    
    @Override
    public void resetFieldPosition(Pose2d pose2d) {
        this.frontVisionSystemSim.resetRobotPose(pose2d);
        this.rearVisionSystemSim.resetRobotPose(pose2d);
    }

    /**
     * Getters and Setters
     */
    public void setPoseSupplier(Supplier<Pose2d> poseSupplier) { this.poseSupplier = poseSupplier; }
}
