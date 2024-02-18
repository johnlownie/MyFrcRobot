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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

/**
 * 
 */
public class VisionModule { //implements Runnable {
    protected final PhotonPoseEstimator frontCameraPhotonPoseEstimator;
    protected final PhotonPoseEstimator rearCameraPhotonPoseEstimator;
    protected final AtomicReference<EstimatedRobotPose> atomicFrontEstimatedRobotPose;
    protected final AtomicReference<EstimatedRobotPose> atomicRearEstimatedRobotPose;

    protected PhotonCamera frontCamera;
    protected PhotonCamera rearCamera;
    protected Supplier<Pose2d> poseSupplier;
    
    /**
     * 
     */
    public VisionModule() {
        //  NetworkTableInstance.kDefaultPort3;
        this.frontCamera = new PhotonCamera(VisionConstants.FRONT_CAMERA_NAME);
        this.rearCamera = new PhotonCamera(VisionConstants.REAR_CAMERA_NAME);

        this.frontCameraPhotonPoseEstimator = new PhotonPoseEstimator(FieldConstants.TAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.frontCamera, VisionConstants.ROBOT_TO_FRONT_CAMERA);
        this.frontCameraPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);

        this.rearCameraPhotonPoseEstimator = new PhotonPoseEstimator(FieldConstants.TAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.rearCamera, VisionConstants.ROBOT_TO_REAR_CAMERA);
        this.rearCameraPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);

        this.atomicFrontEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();
        this.atomicRearEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();
        this.poseSupplier = null;
    }

    // @Override
    public void process() {
        if (this.frontCameraPhotonPoseEstimator == null || this.rearCameraPhotonPoseEstimator == null || this.poseSupplier == null || this.rearCamera == null) {
            return;
        }

        processFrame(this.poseSupplier.get());
        
        processResults("FRONT_CAMERA", getFrontCameraResults(), this.frontCameraPhotonPoseEstimator, this.atomicFrontEstimatedRobotPose);
        processResults("REAR_CAMERA", getRearCameraResults(), this.rearCameraPhotonPoseEstimator, this.atomicRearEstimatedRobotPose);
    }

    /**
     * Gets the latest robot pose. Calling this will only return the pose once. If it returns a non-null value, it is a
     * new estimate that hasn't been returned before.
     * This pose will always be for the BLUE alliance. It must be flipped if the current alliance is RED.
     * @return latest estimated pose
     */
    public EstimatedRobotPose getBestLatestEstimatedPose() {
        PhotonTrackedTarget frontTarget = getBestTarget(true);
        PhotonTrackedTarget rearTarget = getBestTarget(false);

        if (frontTarget == null && rearTarget == null) return null;

        if (frontTarget == null) return this.atomicRearEstimatedRobotPose.getAndSet(null);

        if (rearTarget == null) return this.atomicFrontEstimatedRobotPose.getAndSet(null);

        if (frontTarget.getArea() > rearTarget.getArea()) {
            return this.atomicFrontEstimatedRobotPose.getAndSet(null);
        }

        return this.atomicRearEstimatedRobotPose.getAndSet(null);
    }

    /**
     * 
     */
    public PhotonTrackedTarget getBestTarget(boolean fromFrontCamera) {
        PhotonPipelineResult results = fromFrontCamera ? getFrontCameraResults() : getRearCameraResults();

        return results.hasTargets() ? results.getBestTarget() : null;
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
        List<PhotonTrackedTarget> targets = getFrontCameraResults().getTargets();
        int numTags = 0;
        double avgDist = 0;
        
        for (PhotonTrackedTarget target : targets) {
            Optional<Pose3d> tagPose = this.frontCameraPhotonPoseEstimator.getFieldTags().getTagPose(target.getFiducialId());
            
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
    protected PhotonPipelineResult getFrontCameraResults() {
        return this.frontCamera.getLatestResult();
    }

    /**
     * 
     */
    protected PhotonPipelineResult getRearCameraResults() {
        return this.rearCamera.getLatestResult();
    }
 
    /**
     * Only used in simulation
     */
    protected void processFrame(Pose2d pose) {}

    /**
     * 
     */
    private void processResults(String camera, PhotonPipelineResult results, PhotonPoseEstimator poseEstimator, AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose) {
        Logger.recordOutput("Subsystems/Vision/" + camera + "/hasTargets", results.hasTargets());
        Logger.recordOutput("Subsystems/Vision/" + camera + "/TargetCount", results.hasTargets() ? results.getTargets().size() : 0);

        if (!results.hasTargets() || (results.targets.size() > 1 && results.targets.get(0).getPoseAmbiguity() > VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD)) {
            return;
        }

        poseEstimator.update(results).ifPresent(estimatedRobotPose -> {
            Pose3d estimatedPose = estimatedRobotPose.estimatedPose;

            // Make sure the measurement is on the field
            if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FieldConstants.LENGTH_METERS
                && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FieldConstants.WIDTH_METERS
                && estimatedPose.getZ() > 0.0 && estimatedPose.getZ() <= 0.1) {
                atomicEstimatedRobotPose.set(estimatedRobotPose);
                Logger.recordOutput("Subsystems/Vision/" + camera + "/Pose", estimatedRobotPose.estimatedPose);
            }
        });

        List<String> targetIds = new ArrayList<String>();
        for (PhotonTrackedTarget target : results.getTargets()) {
            targetIds.add("" + target.getFiducialId());
        }
        Logger.recordOutput("Subsystems/Vision/" + camera + "/Target Ids", targetIds.toString());

        if (results.hasTargets()) {
            PhotonTrackedTarget target = results.getBestTarget();
            
            Logger.recordOutput("Subsystems/Vision/" + camera + "/Best Target Id", String.format("%d", target.getFiducialId()));
            Logger.recordOutput("Subsystems/Vision/" + camera + "/Target Pitch", target.getPitch());
            Logger.recordOutput("Subsystems/Vision/" + camera + "/Target Skew", target.getSkew());
            Logger.recordOutput("Subsystems/Vision/" + camera + "/Target Yaw", target.getYaw());
            Logger.recordOutput("Subsystems/Vision/" + camera + "/Best Camera to Target", target.getBestCameraToTarget());
        }
    }

    /**
     * Only used in simulation
     */
    public void resetFieldPosition(Pose2d pose2d) {}

    /**
     * Getters and Setters
     */
    public void setPoseSupplier(Supplier<Pose2d> poseSupplier) { this.poseSupplier = poseSupplier; }
}
