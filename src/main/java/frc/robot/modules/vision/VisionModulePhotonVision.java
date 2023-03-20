package frc.robot.modules.vision;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class VisionModulePhotonVision extends VisionModule {
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();
  
    /**
     * 
     */
    public VisionModulePhotonVision(AprilTagFields aprilTagFields) {
        super(aprilTagFields);

        this.photonCamera = new PhotonCamera(VisionConstants.REAR_CAMERA_NAME);
        this.photonPoseEstimator = new PhotonPoseEstimator(this.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, photonCamera, VisionConstants.ROBOT_TO_REAR_CAMERA);
    }

    /**
     * 
     */
    public String getFormattedFiducialId() {
        PhotonPipelineResult result = this.photonCamera.getLatestResult();
        PhotonTrackedTarget target = null;

        if (result.hasTargets()) {
            target = result.getBestTarget();
        }

        return target != null ? String.format("%d", target.getFiducialId()) : "";
    }

    /**
     * Gets the latest robot pose. Calling this will only return the pose once. If it returns a non-null value, it is a
     * new estimate that hasn't been returned before.
     * This pose will always be for the BLUE alliance. It must be flipped if the current alliance is RED.
     * @return latest estimated pose
     */
    public EstimatedRobotPose grabLatestEstimatedPose() {
        return atomicEstimatedRobotPose.getAndSet(null);
    }

    /**
     * 
     */
    @Override
    public void run() {
        if (this.photonPoseEstimator == null || this.photonCamera == null || RobotState.isAutonomous()) {
            return;
        }

        PhotonPipelineResult results = this.photonCamera.getLatestResult();
        if (!results.hasTargets() || (results.targets.size() > 1 && results.targets.get(0).getPoseAmbiguity() > VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD)) {
            return;
        }

        this.photonPoseEstimator.update(results).ifPresent(estimatedRobotPose -> {
            var estimatedPose = estimatedRobotPose.estimatedPose;

            // Make sure the measurement is on the field
            if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FieldConstants.LENGTH_METERS
                && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FieldConstants.WIDTH_METERS) {
              atomicEstimatedRobotPose.set(estimatedRobotPose);
            }
        });

        if (results.hasTargets()) {
            PhotonTrackedTarget target = results.getBestTarget();

            Logger.getInstance().recordOutput("Vision/FiducialId", String.format("%d", target.getFiducialId()));
        }
    }

    @Override
    public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {}
}
