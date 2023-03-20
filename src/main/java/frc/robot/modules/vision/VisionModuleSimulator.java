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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotState;
import frc.lib.util.SimVisionSystem;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class VisionModuleSimulator extends VisionModule {
    private SimVisionSystem simVisionSystem;
    private Supplier<Pose2d> poseSupplier;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();

    private int counter = 0;

    /**
     * 
     */
    public VisionModuleSimulator(AprilTagFields aprilTagFields) {
        super(aprilTagFields);

        setCamera(new PhotonCamera(VisionConstants.REAR_CAMERA_NAME));
        
        this.simVisionSystem = new SimVisionSystem(VisionConstants.REAR_CAMERA_NAME, VisionConstants.DIAGONAL_FOV, VisionConstants.ROBOT_TO_REAR_CAMERA, 9000, 1280, 1024, 0);
        this.photonPoseEstimator = new PhotonPoseEstimator(this.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, getCamera(), VisionConstants.ROBOT_TO_REAR_CAMERA);

        setTargets();
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
        this.simVisionSystem.processFrame(this.poseSupplier.get());
        
        if (this.photonPoseEstimator == null || RobotState.isAutonomous()) {
            return;
        }

        PhotonPipelineResult results = this.simVisionSystem.cam.getLatestResult();
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

        Logger.getInstance().recordOutput("Vision/hasTargets", results.hasTargets());
        if (results.hasTargets()) {
            PhotonTrackedTarget target = results.getBestTarget();
            
            counter ++;
            if (counter == 50) {
                System.out.println("Best target: " + target.getFiducialId());
                counter = 0;
            }

            Logger.getInstance().recordOutput("Vision/FiducialId", String.format("%d", target.getFiducialId()));
            Logger.getInstance().recordOutput("Vision/Yaw", target.getYaw());
            Logger.getInstance().recordOutput("Vision/Pose", grabLatestEstimatedPose() != null ? grabLatestEstimatedPose().estimatedPose : new Pose3d());
        }
        else {
            System.out.println("No targets");
        }
    }

    /**
     * 
     */
    public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    /**
     * 
     */
    private void setTargets() {
        this.simVisionSystem.addVisionTargets(this.aprilTagFieldLayout);
    }
}
