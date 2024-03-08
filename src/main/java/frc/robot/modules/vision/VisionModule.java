package frc.robot.modules.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.lib.camera.Camera;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

/**
 * 
 */
public class VisionModule {
    protected final Camera camera;
    protected final PhotonCamera photonCamera;

    protected final PhotonPoseEstimator photonPoseEstimator;
    protected final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose;
    
    /**
     * 
     */
    public VisionModule(Camera camera) {
        this.camera = camera;
        this.photonCamera = new PhotonCamera(camera.getName());
        this.photonCamera.setPipelineIndex(camera.getPipelineIndex());

        this.photonPoseEstimator = new PhotonPoseEstimator(FieldConstants.TAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.photonCamera, this.camera.getRobotToCamera());
        this.photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);

        this.atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();
    }

    /**
     * 
     */
    public void process() {
        if (this.photonCamera == null || this.photonPoseEstimator == null) {
            return;
        }

        processResults();
    }

    /**
     * Gets the latest robot pose. Calling this will only return the pose once. If it returns a non-null value, it is a
     * new estimate that hasn't been returned before.
     * This pose will always be for the BLUE alliance. It must be flipped if the current alliance is RED.
     * @return latest estimated pose
     */
    public EstimatedRobotPose getBestLatestEstimatedPose() {
        PhotonTrackedTarget bestTarget = getBestTarget();

        if (bestTarget == null) return null;

        return this.atomicEstimatedRobotPose.getAndSet(null);
    }

    /**
     * 
     */
    public PhotonTrackedTarget getBestTarget() {
        PhotonPipelineResult results = getLatestResult();

        return results.hasTargets() ? results.getBestTarget() : null;
    }

    /**
     * 
     */
    public String getCameraName() {
        return this.camera.getName();
    }

    /**
     * 
     */
    public Transform3d getCameraRobotToCamera() {
        return this.camera.getRobotToCamera();
    }

    /**
     * 
     */
    public Camera.Type getCameraType() {
        return this.camera.getType();
    }

    /**
     * 
     */
    public double getCameraYaw() {
        return this.camera.getYaw();
    }

    /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        Matrix<N3, N1> estimatedStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
        List<PhotonTrackedTarget> targets = getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        
        for (PhotonTrackedTarget target : targets) {
            Optional<Pose3d> tagPose = this.photonPoseEstimator.getFieldTags().getTagPose(target.getFiducialId());
            
            if (tagPose.isEmpty()) continue;
            
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
            numTags++;
        }
        if (numTags == 0) return estimatedStdDevs;
        
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estimatedStdDevs = VisionConstants.MULTI_TAG_STD_DEVS;
        
        // Increase std devs based on (average) distance
        avgDist /= numTags;
        if (numTags == 1 && avgDist > 4)
            estimatedStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else 
            estimatedStdDevs = estimatedStdDevs.times(1 + (avgDist * avgDist / 30));

        return estimatedStdDevs;
    }

    /**
     * 
     */
    protected PhotonPipelineResult getLatestResult() {
        return this.photonCamera.getLatestResult();
    }

    /**
     * 
     */
    protected void processResults() {
        PhotonPipelineResult results = getLatestResult();

        Logger.recordOutput("Subsystems/Vision/" + this.camera.getName() + "/hasTargets", results.hasTargets());
        Logger.recordOutput("Subsystems/Vision/" + this.camera.getName() + "/TargetCount", results.hasTargets() ? results.getTargets().size() : 0);

        if (!results.hasTargets() || (results.targets.size() > 1 && results.targets.get(0).getPoseAmbiguity() > VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD)) {
            return;
        }

        this.photonPoseEstimator.update(results).ifPresent(estimatedRobotPose -> {
            Pose3d estimatedPose = estimatedRobotPose.estimatedPose;

            // Make sure the measurement is on the field
            if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FieldConstants.LENGTH_METERS
                && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FieldConstants.WIDTH_METERS
                && estimatedPose.getZ() > 0.0 && estimatedPose.getZ() <= 0.1) {
                atomicEstimatedRobotPose.set(estimatedRobotPose);
                Logger.recordOutput("Subsystems/Vision/" + this.camera.getName() + "/Pose", estimatedRobotPose.estimatedPose);
            }
        });

        List<String> targetIds = new ArrayList<String>();
        for (PhotonTrackedTarget target : results.getTargets()) {
            targetIds.add("" + target.getFiducialId());
        }
        Logger.recordOutput("Subsystems/Vision/" + this.camera.getName() + "/Target Ids", targetIds.toString());

        if (results.hasTargets()) {
            PhotonTrackedTarget target = results.getBestTarget();
            
            Logger.recordOutput("Subsystems/Vision/" + this.camera.getName() + "/Best Target Id", String.format("%d", target.getFiducialId()));
            Logger.recordOutput("Subsystems/Vision/" + this.camera.getName() + "/Best Target Pitch", target.getPitch());
            Logger.recordOutput("Subsystems/Vision/" + this.camera.getName() + "/Best Target Skew", target.getSkew());
            Logger.recordOutput("Subsystems/Vision/" + this.camera.getName() + "/Best Target Yaw", target.getYaw());
            Logger.recordOutput("Subsystems/Vision/" + this.camera.getName() + "/Best Camera to Target", target.getBestCameraToTarget());
        }
    }

    /**
     * Only used in simulation
     */
    public void resetFieldPosition(Pose2d pose2d) {}

    /**
     * Only used in simulation
     */
    public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {}
}
